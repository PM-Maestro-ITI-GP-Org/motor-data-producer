/*
 * motor_monitor.c
 * ----------------------------------------------------------------------------
 * Pi-side terminal monitor: attaches READ-ONLY to the shm region and prints the
 * latest snapshot row plus ring stats. It's a real consumer, so it verifies the
 * shm contract and the read helpers -- and it's durable: point it at the fake
 * producer now, or at the real controller later, unchanged.
 *
 * Build (QNX): qcc -std=gnu11 -O2 motor_monitor.c -o monitor   (libc only)
 * Run after (or alongside) a producer.
 * ----------------------------------------------------------------------------
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <inttypes.h>
#include <sys/mman.h>

#include "motor_wire.h"
#include "motor_shm.h"

static volatile sig_atomic_t g_running = 1;
static void on_signal(int s) { (void)s; g_running = 0; }

int main(void)
{
    /* Wait for the producer to create + init the region. */
    const shm_region_t *r = MAP_FAILED;
    for (int tries = 0; tries < 100 && r == MAP_FAILED; ++tries) {
        int fd = shm_open(MOTOR_SHM_NAME, O_RDONLY, 0);
        if (fd != -1) {
            r = mmap(NULL, sizeof(shm_region_t), PROT_READ, MAP_SHARED, fd, 0);
            close(fd);
        }
        if (r == MAP_FAILED) { struct timespec d = {0, 100*1000*1000L}; nanosleep(&d, NULL); }
    }
    if (r == MAP_FAILED) {
        fprintf(stderr, "monitor: could not open %s -- is a producer running?\n",
                MOTOR_SHM_NAME);
        return 1;
    }
    while (!motor_shm_region_valid(r)) {            /* wait for init / version match */
        struct timespec d = {0, 50*1000*1000L};
        nanosleep(&d, NULL);
    }

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    fprintf(stderr, "[monitor] attached to %s (Ctrl-C to stop)\n", MOTOR_SHM_NAME);

    /* Consume the ring from "now" so we measure live throughput/drops. */
    uint64_t read_pos    = motor_ring_write_pos(&r->ring);
    uint64_t blocks_seen = 0;
    uint64_t drops       = 0;
    static shm_block_t blk;   /* scratch copy target */

    while (g_running) {
        /* snapshot (latest row) */
        motor_row_t row;
        uint32_t s_seq = 0; uint64_t s_ts = 0; uint16_t s_fl = 0;
        int ok = motor_snapshot_read(&r->snapshot, &row, &s_seq, &s_ts, &s_fl);

        /* drain the ring up to the producer's write_pos, counting drops */
        uint64_t wp = motor_ring_write_pos(&r->ring);
        uint32_t depth = r->ring.depth;
        if (wp - read_pos > depth) {                 /* producer lapped us */
            drops += (wp - read_pos) - depth;
            read_pos = wp - depth;
        }
        uint32_t last_first_current = 0;
        while (read_pos < wp) {
            if (motor_ring_read_slot(&r->ring, read_pos, &blk)) {
                blocks_seen++;
                if (blk.n_rows > 0) last_first_current = blk.rows[0].current[0];
            } else {
                drops++;                              /* overwritten mid-copy */
            }
            read_pos++;
        }

        if (ok) {
            printf("snap seq=%-8" PRIu32 " ts=%-12" PRIu64
                   " cur=%u/%u/%u vph=%u/%u/%u vdc=%u vspd=%u"
                   " vib=(%d,%d,%d) rpm=%u | "
                   "ring wp=%" PRIu64 " seen=%" PRIu64 " drops=%" PRIu64
                   " (blk[0].cur=%u)\n",
                   s_seq, s_ts,
                   row.current[0], row.current[1], row.current[2],
                   row.current[3], row.current[4], row.current[5],
                   row.current[6], row.current[7],
                   row.vib_x, row.vib_y, row.vib_z,
                   row.rpm,
                   wp, blocks_seen, drops, last_first_current);
        } else {
            printf("snap: read contended (retrying) | ring wp=%" PRIu64
                   " seen=%" PRIu64 " drops=%" PRIu64 "\n", wp, blocks_seen, drops);
        }
        fflush(stdout);

        struct timespec d = {0, 200*1000*1000L};     /* 5 Hz refresh */
        nanosleep(&d, NULL);
    }

    munmap((void *)r, sizeof(shm_region_t));
    fprintf(stderr, "[monitor] stopped: blocks_seen=%" PRIu64 " drops=%" PRIu64 "\n",
            blocks_seen, drops);
    return 0;
}