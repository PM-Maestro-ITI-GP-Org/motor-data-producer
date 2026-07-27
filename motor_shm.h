/*
 * motor_shm.h
 * ----------------------------------------------------------------------------
 * Pi-ONLY shared-memory contract: the layout of the POSIX shm region the QNX
 * controller publishes into and the Qt / SOME/IP consumers read from. Uses C11
 * atomics and cache-line alignment. The STM32 has no shared memory and must NOT
 * compile this file -- it lives only in the Pi source tree.
 *
 * QNX: shm_open/mmap/sem_/pthread/clock_* and C11 atomics all live in libc.
 *      Do NOT link -lrt or -lpthread.
 * ----------------------------------------------------------------------------
 */
#ifndef MOTOR_SHM_H
#define MOTOR_SHM_H

#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include "motor_wire.h"   /* motor_row_t, frame_header_t, MOTOR_* sizes + version */

#define MOTOR_RING_DEPTH  16u   /* block-ring slots; 16 @ ~100 Hz ~= 160 ms of history */

/* ============================ shared-memory contract ====================== */
#define MOTOR_SHM_NAME    "/motor_ctrl"
#define MOTOR_SHM_MAGIC   0x4D435452u   /* "MCTR" */

#ifndef MOTOR_CACHELINE
#define MOTOR_CACHELINE   64u           /* aarch64 line; isolates writers/readers */
#endif

/* ---- seqlock snapshot: latest values, for the Qt dashboard ----------------
 * Single most-recent row. Reader never blocks the producer; on the rare torn
 * read it simply retries. seqlock: even = stable, odd = write in progress.    */
typedef struct {
    _Alignas(MOTOR_CACHELINE) _Atomic uint32_t seqlock;
    uint32_t     producer_seq;   /* frame seq this snapshot came from */
    uint64_t     timestamp;
    uint16_t     flags;
    uint16_t     _pad;
    motor_row_t  row;            /* the latest row */
} shm_snapshot_t;

/* ---- one ring slot --------------------------------------------------------
 * Holds a whole block. Per-slot seqlock lets a consumer detect that the
 * producer overwrote a slot mid-copy (i.e. the consumer was lapped).          */
typedef struct {
    _Alignas(MOTOR_CACHELINE) _Atomic uint32_t seq; /* odd while producer writes */
    uint32_t     producer_seq;   /* frame seq stored in this slot */
    uint64_t     timestamp;
    uint16_t     n_rows;
    uint16_t     flags;
    motor_row_t  rows[MOTOR_MAX_ROWS_PER_BLOCK];
} shm_block_t;

/* ---- lock-free block ring: every block, for the SOME/IP publisher ----------
 * Producer advances write_pos monotonically. Each consumer keeps its OWN local
 * read cursor, so adding/removing consumers never touches the producer.        */
typedef struct {
    _Alignas(MOTOR_CACHELINE) _Atomic uint64_t write_pos; /* committed-block count */
    uint32_t     depth;          /* == MOTOR_RING_DEPTH */
    uint32_t     _pad;
    shm_block_t  slots[MOTOR_RING_DEPTH];
} shm_block_ring_t;

/* ---- top-level region (one shm object, both publish paths) ----------------- */
typedef struct {
    uint32_t  magic;             /* == MOTOR_SHM_MAGIC */
    uint16_t  version;           /* == MOTOR_CONTRACT_VERSION */
    uint16_t  _pad0;
    uint32_t  row_size;          /* == sizeof(motor_row_t), runtime sanity check */
    uint32_t  reserved;
    _Alignas(MOTOR_CACHELINE) shm_snapshot_t   snapshot;
    _Alignas(MOTOR_CACHELINE) shm_block_ring_t ring;
} shm_region_t;

/* ============================ region lifecycle ============================ */
static inline void motor_shm_region_init(shm_region_t *r)
{
    r->magic    = MOTOR_SHM_MAGIC;
    r->version  = (uint16_t)MOTOR_CONTRACT_VERSION;
    r->_pad0    = 0u;
    r->row_size = (uint32_t)sizeof(motor_row_t);
    r->reserved = 0u;
    atomic_store_explicit(&r->snapshot.seqlock, 0u, memory_order_relaxed);
    r->ring.depth = MOTOR_RING_DEPTH;
    atomic_store_explicit(&r->ring.write_pos, 0u, memory_order_relaxed);
    for (uint32_t i = 0; i < MOTOR_RING_DEPTH; ++i)
        atomic_store_explicit(&r->ring.slots[i].seq, 0u, memory_order_relaxed);
}

static inline bool motor_shm_region_valid(const shm_region_t *r)
{
    return r->magic    == MOTOR_SHM_MAGIC
        && r->version  == (uint16_t)MOTOR_CONTRACT_VERSION
        && r->row_size == (uint32_t)sizeof(motor_row_t);
}

/* ============================ snapshot helpers ============================ */
/* PRODUCER */
static inline void motor_snapshot_publish(shm_snapshot_t *s,
                                          const motor_row_t *row,
                                          uint32_t producer_seq,
                                          uint64_t timestamp,
                                          uint16_t flags)
{
    uint32_t s0 = atomic_load_explicit(&s->seqlock, memory_order_relaxed);
    atomic_store_explicit(&s->seqlock, s0 + 1u, memory_order_relaxed); /* -> odd */
    atomic_thread_fence(memory_order_release);
    s->row          = *row;
    s->producer_seq = producer_seq;
    s->timestamp    = timestamp;
    s->flags        = flags;
    atomic_thread_fence(memory_order_release);
    atomic_store_explicit(&s->seqlock, s0 + 2u, memory_order_relaxed); /* -> even (commit) */
}

/* CONSUMER (Qt). Returns false only under pathological contention; caller keeps last. */
static inline bool motor_snapshot_read(const shm_snapshot_t *s,
                                       motor_row_t *out_row,
                                       uint32_t *out_seq,
                                       uint64_t *out_ts,
                                       uint16_t *out_flags)
{
    for (int tries = 0; tries < 64; ++tries) {
        uint32_t s1 = atomic_load_explicit(&s->seqlock, memory_order_acquire);
        if (s1 & 1u) continue;                 /* write in progress */
        *out_row = s->row;
        if (out_seq)   *out_seq   = s->producer_seq;
        if (out_ts)    *out_ts    = s->timestamp;
        if (out_flags) *out_flags = s->flags;
        atomic_thread_fence(memory_order_acquire);
        uint32_t s2 = atomic_load_explicit(&s->seqlock, memory_order_relaxed);
        if (s1 == s2) return true;             /* consistent snapshot */
    }
    return false;
}

/* ============================ block-ring helpers ========================== */
/* PRODUCER: write one block into the next slot, then publish it. */
static inline void motor_ring_publish(shm_block_ring_t *ring,
                                      const frame_header_t *hdr,
                                      const motor_row_t *rows)
{
    uint64_t pos  = atomic_load_explicit(&ring->write_pos, memory_order_relaxed);
    shm_block_t *slot = &ring->slots[pos % ring->depth];

    uint32_t s0 = atomic_load_explicit(&slot->seq, memory_order_relaxed);
    atomic_store_explicit(&slot->seq, s0 + 1u, memory_order_relaxed); /* -> odd */
    atomic_thread_fence(memory_order_release);

    uint16_t n = hdr->n_rows;
    if (n > MOTOR_MAX_ROWS_PER_BLOCK) n = MOTOR_MAX_ROWS_PER_BLOCK;
    slot->producer_seq = hdr->seq;
    slot->timestamp    = hdr->timestamp;
    slot->n_rows       = n;
    slot->flags        = hdr->flags;
    for (uint16_t i = 0; i < n; ++i) slot->rows[i] = rows[i];

    atomic_thread_fence(memory_order_release);
    atomic_store_explicit(&slot->seq, s0 + 2u, memory_order_relaxed); /* -> even */

    atomic_thread_fence(memory_order_release);
    atomic_store_explicit(&ring->write_pos, pos + 1u, memory_order_release); /* publish */
}

/* CONSUMER: copy the slot at `pos` into `out`. Returns false if the producer
 * overwrote it mid-copy (consumer was lapped -> caller counts a drop).        */
static inline bool motor_ring_read_slot(const shm_block_ring_t *ring,
                                        uint64_t pos,
                                        shm_block_t *out)
{
    const shm_block_t *slot = &ring->slots[pos % ring->depth];
    for (int tries = 0; tries < 64; ++tries) {
        uint32_t s1 = atomic_load_explicit(&slot->seq, memory_order_acquire);
        if (s1 & 1u) continue;                 /* producer writing this slot */
        uint16_t n = slot->n_rows;
        if (n > MOTOR_MAX_ROWS_PER_BLOCK) n = MOTOR_MAX_ROWS_PER_BLOCK;
        out->producer_seq = slot->producer_seq;
        out->timestamp    = slot->timestamp;
        out->n_rows       = n;
        out->flags        = slot->flags;
        for (uint16_t i = 0; i < n; ++i) out->rows[i] = slot->rows[i];
        atomic_thread_fence(memory_order_acquire);
        uint32_t s2 = atomic_load_explicit(&slot->seq, memory_order_relaxed);
        if (s1 == s2) return true;             /* not overwritten mid-copy */
    }
    return false;
}

/* Latest published block index (one past the newest). A consumer reads this
 * with acquire semantics, then walks its local cursor up to it.               */
static inline uint64_t motor_ring_write_pos(const shm_block_ring_t *ring)
{
    return atomic_load_explicit(&ring->write_pos, memory_order_acquire);
}

#endif /* MOTOR_SHM_H */
