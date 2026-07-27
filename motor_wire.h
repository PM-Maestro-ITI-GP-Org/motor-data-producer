/*
 * motor_wire.h
 * ----------------------------------------------------------------------------
 * SHARED wire contract: the exact byte layout that crosses the STM32 <-> Pi SPI
 * link. Compiled into BOTH binaries (STM32 firmware and the QNX controller), so
 * this is the one piece of source that must be identical on both sides. Keep it
 * in a shared location (monorepo common/ dir, or a git submodule) -- never two
 * hand-edited copies.
 *
 * Pure data layout only: no atomics, no shared memory, no OS calls. Safe to
 * compile on the bare-metal STM32. Assumes a C11 compiler (for _Static_assert);
 * on a C99 firmware build, swap the _Static_assert lines for an equivalent
 * compile-time-assert macro.
 *
 * v2 + cfg1: data frame layout unchanged, command/ACK protocol added.
 * ----------------------------------------------------------------------------
 */
#ifndef MOTOR_WIRE_H
#define MOTOR_WIRE_H

#include <stdint.h>
#include <stddef.h>

/* ============================ contract version ============================ */
/* Bump whenever ANY wire/shm struct layout changes (esp. motor_row_t). Stamped
 * into every frame header and into the shm region; the producer and every
 * consumer compare it and refuse to run on a mismatch.                         */
#define MOTOR_CONTRACT_VERSION   4u   /* v4: row grew to 8 currents (8-ch ADC scan); MAX_ROWS reduced */

/* ============================ wire framing ================================ */
#define MOTOR_FRAME_MAGIC        0x4D4F5452u  /* "MOTR" little-endian resync marker */

/* Generous compile-time maximum -- FIXED. The runtime block size is a config
 * parameter and MUST be <= MOTOR_MAX_ROWS_PER_BLOCK. Buffers are sized to the
 * max so a config change never requires recompilation.
 *
 * Reduced 512(v2) -> 300(v3, 3 currents) -> 200(v4, 8 currents). The row
 * doubled to 24 B at v4, so the ceiling drops again to keep the frame buffers
 * inside the F401CC's 64 KB SRAM and the worst-case transfer under the block
 * period. Worst-case frame at MAX rows: 24 + 200*24 + 4 = 4828 bytes = 4.83 ms
 * transfer @ 8 MHz SPI -- the link was raised 4 -> 8 MHz at v4 to carry the
 * doubled payload (8ch*20kHz*24B = 3.84 Mbit/s exceeds a 4 MHz link) -- well
 * under a 10 ms (100 Hz) block period.                                       */
#define MOTOR_MAX_ROWS_PER_BLOCK 200u

/* ---- the per-timestep "wide row" -----------------------------------------
 *  Sensor set: 8 ADC channels (PA0..PA7 = IN0..IN7), vibration (3 axes), speed.
 *  Raw counts on the wire; scaling to engineering units is applied downstream
 *  from the config constants. If the sensor set ever changes, edit these
 *  fields, update the static_assert, and bump MOTOR_CONTRACT_VERSION.
 */
#pragma pack(push, 1)
typedef struct {
    uint16_t current[8]; /* eight ADC1 channels, PA0..PA7 = IN0..IN7 (scan order) */
    int16_t  vib_x;      /* MPU6050 over I2C, ZOH-held at imu_rate_hz            */
    int16_t  vib_y;
    int16_t  vib_z;
    uint16_t rpm;        /* speed, from timer input-capture (tach pulses)        */
} motor_row_t;
#pragma pack(pop)

_Static_assert(sizeof(motor_row_t) == 24,
               "motor_row_t layout changed: update this size and bump MOTOR_CONTRACT_VERSION");

/* ---- frame header (prepended to every block on the wire) ------------------
 *
 * `flags` and `_reserved` are now ALSO used to carry ACKs back to the Pi:
 *   - flags bits MOTOR_FLAG_ACK_* / MOTOR_FLAG_NACK_* / MOTOR_FLAG_CONFIG_APPLIED
 *   - _reserved = low 16 bits of the cmd_seq of the most recent command the
 *                 STM32 has processed (apply or reject). Latched -- the same
 *                 value rides every frame until a new command supersedes it.
 */
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;      /* == MOTOR_FRAME_MAGIC (resync after a desync)          */
    uint32_t seq;        /* monotonic frame counter; gaps => dropped blocks       */
    uint64_t timestamp;  /* STM32 timebase, microseconds (epoch/units TBD in fw)  */
    uint16_t version;    /* == MOTOR_CONTRACT_VERSION                             */
    uint16_t flags;      /* ACK/NACK bits + status (see MOTOR_FLAG_* below)       */
    uint16_t n_rows;     /* valid rows this frame; <= MOTOR_MAX_ROWS_PER_BLOCK    */
    uint16_t _reserved;  /* low 16 bits of last-processed cmd_seq (ACK target)    */
} frame_header_t;
#pragma pack(pop)

_Static_assert(sizeof(frame_header_t) == 24, "frame_header_t layout changed");

/* CRC covers exactly [frame_header_t][n_rows * motor_row_t]. CRC-32/MPEG-2:
 * init 0xFFFFFFFF, poly 0x04C11DB7, MSB-first, no reflection, no final XOR.
 * The SAME CRC routine is reused for command frames (see below).               */
typedef uint32_t frame_crc_t;

/* On-wire data frame (variable length):
 *     [ frame_header_t ][ motor_row_t rows[n_rows] ][ frame_crc_t ]
 * Worst-case size, for fixed RX buffer allocation:                            */
#define MOTOR_MAX_FRAME_BYTES \
    (sizeof(frame_header_t) + (size_t)MOTOR_MAX_ROWS_PER_BLOCK * sizeof(motor_row_t) + sizeof(frame_crc_t))

/* ============================ flags / ACK bits ============================
 *
 * Carried in frame_header_t.flags. Multiple bits may be set in one frame
 * (e.g. ACK_OK | CONFIG_APPLIED on the first frame using a newly-applied
 * config). The Pi matches the ACK to its in-flight command via
 * frame_header_t._reserved == (cmd_seq & 0xFFFF).
 */
#define MOTOR_FLAG_ACK_OK         0x0001u  /* last command applied OK              */
#define MOTOR_FLAG_ACK_NACK       0x0002u  /* last command rejected (see reason)   */
#define MOTOR_FLAG_NACK_RANGE     0x0004u  /* a payload value was out of range     */
#define MOTOR_FLAG_NACK_CRC       0x0008u  /* command CRC mismatch                 */
#define MOTOR_FLAG_NACK_VER       0x0010u  /* schema_version not supported by fw   */
#define MOTOR_FLAG_NACK_CMD       0x0020u  /* unknown cmd opcode                   */
#define MOTOR_FLAG_CONFIG_APPLIED 0x0040u  /* this is the FIRST frame using the    */
                                           /* newly applied config                 */
/* future producer flags (ZOH-stale, overrun, ...) take the upper bits         */

/* ============================ command / config protocol ==================
 *
 * Commands flow Pi -> STM32 by piggybacking on the data SPI exchange: the Pi
 * fills its outbound buffer (tx) with a command frame whenever it has one to
 * send, and zeros it otherwise. The STM32 already DMAs every byte of tx into
 * its rx buffer, so no separate channel or extra GPIO is needed.
 *
 * One command rides one SPI transfer. The STM32 inspects the first 4 bytes;
 * if magic == MOTOR_CMD_MAGIC it parses the rest, validates CRC, queues the
 * command, and applies it at the NEXT motor_on_block_ready boundary (never
 * mid-DMA). The ACK rides back in the very next outbound frame_header_t.
 *
 * The Pi keeps the same command in tx until it sees a matching ACK (by
 * cmd_seq in _reserved), then clears tx back to zeros.
 */
#define MOTOR_CMD_MAGIC              0x434D4452u  /* "CMDR" -- distinct from MOTR */

#define MOTOR_CMD_NONE               0u  /* tx all-zero => no command           */
#define MOTOR_CMD_SET_CONFIG         1u
#define MOTOR_CMD_PING               2u

#define MOTOR_CONFIG_SCHEMA_VERSION  2u  /* bump on config_payload_t change     */

/* config_payload_t.source */
#define MOTOR_SOURCE_SYNTH           0u  /* deprecated; firmware no longer builds it */
#define MOTOR_SOURCE_ADC             1u

/* config_payload_t.run_state */
#define MOTOR_RUN_STOP               0u
#define MOTOR_RUN_RUN                1u

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;          /* == MOTOR_CMD_MAGIC; else the whole tx is ignored  */
    uint16_t cmd;            /* MOTOR_CMD_*                                       */
    uint16_t schema_version; /* == MOTOR_CONFIG_SCHEMA_VERSION (for SET_CONFIG)   */
    uint16_t cmd_seq;        /* low 16 bits echoed in frame_header_t._reserved   */
    uint16_t _pad;           /* keep 8-byte alignment of payload                  */
} cmd_header_t;

typedef struct {
    uint16_t block_rows;     /* 1..MOTOR_MAX_ROWS_PER_BLOCK                       */
    uint16_t _reserved0;     /* was 'synth_cycles' in v1; ignored in v2          */
    uint16_t source;         /* MOTOR_SOURCE_* (only ADC valid in v2)            */
    uint16_t run_state;      /* MOTOR_RUN_*                                       */
    uint32_t sample_rate_hz; /* ADC current sample rate; 100..100000 Hz          */
                             /* (block cadence = sample_rate_hz / block_rows)    */
    uint32_t imu_rate_hz;    /* MPU6050 poll rate; 10..1000 Hz                   */
                             /* (1000 = max useful for MPU6050 accel)            */
    uint8_t  reserved[12];   /* MUST be zeroed by the sender                     */
} config_payload_t;
#pragma pack(pop)

_Static_assert(sizeof(cmd_header_t) == 12, "cmd_header_t layout changed");
_Static_assert(sizeof(config_payload_t) == 28, "config_payload_t layout changed");

/* On-wire command frame (fixed length):
 *     [ cmd_header_t ][ config_payload_t ][ frame_crc_t ]
 * CRC32/MPEG-2 over [cmd_header_t][config_payload_t].
 *
 * The whole structure must fit within MOTOR_MAX_FRAME_BYTES (it does, by a
 * huge margin) because it rides the same buffer as the data frame's tx side.
 */
#define MOTOR_CMD_FRAME_BYTES \
    (sizeof(cmd_header_t) + sizeof(config_payload_t) + sizeof(frame_crc_t))

_Static_assert(MOTOR_CMD_FRAME_BYTES <= MOTOR_MAX_FRAME_BYTES,
               "command frame must fit in the SPI exchange buffer");

#endif /* MOTOR_WIRE_H */