/*
 * motor_controller.c
 * ----------------------------------------------------------------------------
 * QNX controller / producer node for the predictive-maintenance pipeline.
 * INTERRUPT-DRIVEN build: reads are driven by the data-ready GPIO edge, so every
 * read lands on a fresh, complete frame (no free-running poll => no clock drift,
 * no duplicate/gap churn from being unsynced to the STM32).
 *
 * Single SCHED_FIFO thread that, per data-ready interrupt:
 *   1. reads one fixed-size frame over SPI (rpi_spi driver),
 *   2. validates magic / version / size / CRC and accounts for sequence gaps,
 *   3. publishes the latest row to the seqlock snapshot (for Qt) and the whole
 *      block to the lock-free ring (for the SOME/IP publisher).
 *
 * A TimerTimeout bounds the pulse wait, and on every wake-up the controller
 * re-checks the data-ready pin LEVEL: it reads whenever the line is high, using
 * the rising-edge pulse only as a fast wake-up. This is robust to a missed edge
 * (line already/still high) -- it can never deadlock waiting for an edge that
 * already happened. A low line simply means idle.
 *
 * GPIO ACCESS POLICY -- IMPORTANT:
 *   All GPIO access goes through the rpi_gpio resource manager (the client API
 *   in rpi_gpio.c / rpi_gpio.h). The server owns the RP1 hardware; this process
 *   does NOT map or poke RP1 registers directly. A previous revision tried to
 *   bypass the server with hard-coded MAP_PHYS pokes at 0x400d0000 / 0x400e0000
 *   to "force" a pull-down and read a "live" pad level. Those addresses and the
 *   pull bit-encoding were wrong for the Pi 5 / RP1 (io_bank0 is at 0x400d0000,
 *   the pad/pull block is elsewhere, and RP1 uses separate PDE/PUE enable bits,
 *   not a 2-bit field). The result was that the pull-down never landed (the pin
 *   stayed pulled UP) and the bogus "live level" read never went high, so the
 *   controller never accepted a single frame (ok stayed at 0). The fix is to use
 *   the supported server API for the pull and the level, which is what this file
 *   now does.
 *
 *   Idle/disconnected rejection is handled the correct way: an internal
 *   pull-DOWN requested through the server, plus -- strongly recommended -- an
 *   external 10 kOhm pull-down between the data-ready GPIO and GND on the header.
 *
 * Build (QNX): libc only -- do NOT link -lrt or -lpthread. Compile as C11.
 *   qcc -V<target> -std=gnu11 -O2 motor_controller.c rpi_gpio.c rpi_spi.c \
 *       -lrpi_spi -o motor_controller
 *   (rpi_gpio.c + rpi_gpio.h: the client API from the hardware-component-samples repo)
 * Requires root to raise SCHED_FIFO priority. The rpi_gpio resource manager must
 * already be running (stock image: /dev/gpio is present).
 *
 * HARD PREREQUISITE: the rpi_gpio resource manager must be running (the stock
 * image registers /dev/gpio) and cfg.dataready_pin must be a free header GPIO
 * (0..27, not your SPI pins). Without the server the controller refuses to start
 * (by design -- there is no paced fallback).
 *
 * STILL TODO (tracked on the checklist):
 *   - Load runtime params from JSON and push the STM32 subset via SET_CONFIG.
 * ----------------------------------------------------------------------------
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/neutrino.h>   /* ChannelCreate, ConnectAttach, MsgReceivePulse, TimerTimeout */

#include "motor_wire.h"
#include "motor_shm.h"
#include "rpi_spi.h"        /* rpi_spi_*, SPI_SUCCESS (your existing driver) */
#include "rpi_gpio.h"       /* rpi_gpio_* client API (hardware-component-samples) */

/* ============================ runtime config ============================== */
typedef struct {
    int      spi_bus;
    int      spi_dev;
    int      spi_mode;
    uint32_t spi_clock_hz;
    uint16_t block_rows;
    long     period_ns;
    int      rt_priority;
    int      dataready_pin;
} controller_config_t;

static const controller_config_t DEFAULT_CFG = {
    .spi_bus       = 0,
    .spi_dev       = 0,
    .spi_mode      = 0,
    .spi_clock_hz  = 1000000u,   /* DEBUG: 1 MHz to rule out F401 slave timing */
    .block_rows    = 200,
    .period_ns     = 10L * 1000L * 1000L,
    .rt_priority   = 30,
    .dataready_pin = 17,
};

/* ============================ diagnostics ================================= */
typedef struct {
    uint64_t frames_ok;
    uint64_t seq_drops;
    uint64_t crc_err;
    uint64_t magic_err;
    uint64_t version_err;
    uint64_t size_err;
    uint64_t duplicates;
    uint64_t resets;
    uint64_t timeouts;
    uint64_t spi_err;
} controller_stats_t;

/* ============================ CRC ========================================= */
/* CRC-32/MPEG-2: init 0xFFFFFFFF, poly 0x04C11DB7, MSB-first, no reflection,
 * no final XOR. Byte-for-byte identical to the STM32 table-driven crc32_mpeg2()
 * in motor_send.c (verified). Both sides MUST stay in lockstep. */
static uint32_t frame_crc_compute(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint32_t)data[i] << 24;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x80000000u) ? (crc << 1) ^ 0x04C11DB7u : (crc << 1);
    }
    return crc;
}

/* ============================ scheduling ================================== */
static int set_realtime_priority(int prio)
{
    struct sched_param sp;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = prio;
    return pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
}

/* ============================ shared memory =============================== */
static shm_region_t *shm_setup(void)
{
    int fd = shm_open(MOTOR_SHM_NAME, O_RDWR | O_CREAT, 0666);
    if (fd == -1) return NULL;
    if (ftruncate(fd, (off_t)sizeof(shm_region_t)) == -1) { close(fd); return NULL; }
    shm_region_t *r = mmap(NULL, sizeof(shm_region_t),
                           PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (r == MAP_FAILED) return NULL;
    motor_shm_region_init(r);
    return r;
}

/* ============================ data-ready (edge pulse via rpi_gpio) ======== */
enum { WR_OK, WR_TIMEOUT, WR_ERROR };

#define DR_EVENT_ID    1

typedef struct {
    int      pin;
    int      chid;
    int      coid;
    uint64_t poll_ns;
} dataready_t;

/*
 * Read the current data-ready level through the rpi_gpio resource manager.
 * RPI_GPIO_READ returns the live hardware level, so this reflects the actual
 * pad voltage; there is no need (and no correct way from user space here) to
 * touch RP1 registers directly. Returns 1 for high, 0 for low/error.
 */
static int dataready_read_level(dataready_t *d)
{
    unsigned level = GPIO_LOW;
    if (rpi_gpio_input(d->pin, &level) != GPIO_SUCCESS)
        return 0;
    return (level == GPIO_HIGH) ? 1 : 0;
}

static int dataready_init(dataready_t *d, const controller_config_t *cfg)
{
    d->pin     = cfg->dataready_pin;
    d->chid    = -1;
    d->coid    = -1;
    d->poll_ns = 2u * 1000u * 1000u;   /* 2 ms safety poll; the edge is the fast path */

    d->chid = ChannelCreate(0);
    if (d->chid == -1) {
        fprintf(stderr, "error: ChannelCreate failed: %s\n", strerror(errno));
        return -1;
    }
    d->coid = ConnectAttach(0, 0, d->chid, _NTO_SIDE_CHANNEL, 0);
    if (d->coid == -1) {
        fprintf(stderr, "error: ConnectAttach failed: %s\n", strerror(errno));
        return -1;
    }

    /* Configure the pin as an input (through the server). */
    if (rpi_gpio_setup(d->pin, GPIO_IN) != GPIO_SUCCESS) {
        fprintf(stderr, "error: rpi_gpio_setup(pin=%d) failed\n", d->pin);
        return -1;
    }

    /*
     * Arm the rising-edge detector. add_event_detect may re-touch the pad and
     * leave the pull at the BSP default, so we set the pull AFTER it, making the
     * pull-down request the last writer to the pad.
     */
    if (rpi_gpio_add_event_detect(d->pin, d->coid, GPIO_RISING, DR_EVENT_ID)
            != GPIO_SUCCESS) {
        fprintf(stderr, "error: rpi_gpio_add_event_detect(pin=%d) failed\n", d->pin);
        return -1;
    }

    /*
     * Request an internal pull-DOWN through the server so a disconnected/idle
     * data-ready line reads low instead of floating high. This is the supported
     * path; combine with an external 10 kOhm pull-down on the header for a line
     * that is reliably low when the STM32 is not driving it.
     */
    if (rpi_gpio_setup_pull(d->pin, GPIO_IN, GPIO_PUD_DOWN) != GPIO_SUCCESS) {
        fprintf(stderr, "warning: rpi_gpio_setup_pull(pin=%d, DOWN) failed -- "
                        "fit an external pull-down resistor\n", d->pin);
    }

    /* Sanity check: with no STM32 driving it, the line should read low now. */
    if (dataready_read_level(d) == 1) {
        fprintf(stderr, "WARNING: GPIO%d reads HIGH at startup -- check wiring / "
                        "pull-down (spurious reads possible)\n", d->pin);
    } else {
        fprintf(stderr, "[ctrl] GPIO%d idle level low (pull-down active)\n", d->pin);
    }

    return 0;
}

static int dataready_wait(dataready_t *d, controller_stats_t *st)
{
    (void)st;
    uint64_t to = d->poll_ns;
    struct _pulse pulse;

    /* Block for a rising-edge pulse, but never longer than poll_ns so a missed
     * edge (line already high) is still picked up by the level re-check below. */
    TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, NULL, &to, NULL);
    int rc = MsgReceivePulse(d->chid, &pulse, sizeof pulse, NULL);
    if (rc == -1 && errno != ETIMEDOUT)
        return WR_ERROR;

    /*
     * Authoritative decision is the LEVEL, not the edge. The STM32 holds the
     * data-ready line high for the entire SPI transfer window (it only drops it
     * from the transfer-complete ISR, after the Pi has clocked the whole frame),
     * so a single live read is sufficient and correct: high => a frame is
     * waiting, read it now; low => idle.
     */
    return dataready_read_level(d) ? WR_OK : WR_TIMEOUT;
}

static void dataready_cleanup(dataready_t *d)
{
    rpi_gpio_cleanup();
    if (d->coid != -1) ConnectDetach(d->coid);
    if (d->chid != -1) ChannelDestroy(d->chid);
}

/* ============================ shutdown ==================================== */
static volatile sig_atomic_t g_running = 1;
static void on_signal(int sig) { (void)sig; g_running = 0; }

/* ============================ main ======================================== */
int main(void)
{
    controller_config_t cfg = DEFAULT_CFG;

    if (set_realtime_priority(cfg.rt_priority) != 0)
        fprintf(stderr, "warning: SCHED_FIFO prio %d not set (need privilege): %s\n",
                cfg.rt_priority, strerror(errno));

    shm_region_t *region = shm_setup();
    if (!region) { perror("shm_setup"); return 1; }

    if (rpi_spi_configure_device(cfg.spi_bus, cfg.spi_dev, cfg.spi_mode,
                                 cfg.spi_clock_hz) != SPI_SUCCESS) {
        fprintf(stderr, "rpi_spi_configure_device failed\n");
        munmap(region, sizeof(shm_region_t));
        shm_unlink(MOTOR_SHM_NAME);
        return 1;
    }

    dataready_t dr;
    if (dataready_init(&dr, &cfg) != 0) {
        dataready_cleanup(&dr);
        rpi_spi_cleanup_device(cfg.spi_bus, cfg.spi_dev);
        munmap(region, sizeof(shm_region_t));
        shm_unlink(MOTOR_SHM_NAME);
        return 1;
    }
    fprintf(stderr, "[ctrl] edge-pulse mode on GPIO%d (via rpi_gpio)\n", dr.pin);

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    const size_t frame_bytes = sizeof(frame_header_t)
                             + (size_t)cfg.block_rows * sizeof(motor_row_t)
                             + sizeof(frame_crc_t);

    static _Alignas(8) uint8_t rx[MOTOR_MAX_FRAME_BYTES];
    static _Alignas(8) uint8_t tx[MOTOR_MAX_FRAME_BYTES];
    memset(tx, 0, sizeof tx);

    controller_stats_t st = {0};
    uint32_t last_seq = 0;
    int      have_last = 0;
    struct timespec t_log;
    clock_gettime(CLOCK_MONOTONIC, &t_log);

    while (g_running) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec != t_log.tv_sec) {
            t_log = now;
            fprintf(stderr,
                "[ctrl] ok=%" PRIu64 " drops=%" PRIu64 " crc=%" PRIu64
                " magic=%" PRIu64 " ver=%" PRIu64 " size=%" PRIu64
                " dup=%" PRIu64 " rst=%" PRIu64 " to=%" PRIu64
                " spi=%" PRIu64 "\n",
                st.frames_ok, st.seq_drops, st.crc_err, st.magic_err,
                st.version_err, st.size_err, st.duplicates, st.resets,
                st.timeouts, st.spi_err);
        }

        int w = dataready_wait(&dr, &st);
        if (w == WR_TIMEOUT) { st.timeouts++; continue; }
        if (w == WR_ERROR)   { if (errno == EINTR) continue; st.timeouts++; continue; }

        if (rpi_spi_write_read_data(cfg.spi_bus, cfg.spi_dev, tx, rx,
                                    frame_bytes) != SPI_SUCCESS) {
            st.spi_err++;
            continue;
        }

        const frame_header_t *h = (const frame_header_t *)rx;

        if (h->magic != MOTOR_FRAME_MAGIC) {
            /* DEBUG: dump the first 16 received bytes ~once per second so we can
             * tell a dead line (all 0xFF / all 0x00) from real-but-misframed
             * data. Remove once frames are flowing. */
            if ((st.magic_err % 500u) == 0u) {
                fprintf(stderr, "[ctrl] magic miss; rx[0..15]=");
                for (int i = 0; i < 16; ++i) fprintf(stderr, " %02x", rx[i]);
                fprintf(stderr, "  (want magic=%08x)\n", (unsigned)MOTOR_FRAME_MAGIC);
            }
            st.magic_err++;
            continue;
        }
        if (h->version != MOTOR_CONTRACT_VERSION)   { st.version_err++; continue; }
        if (h->n_rows == 0 || h->n_rows != cfg.block_rows) { st.size_err++; continue; }

        size_t covered = sizeof(frame_header_t) + (size_t)h->n_rows * sizeof(motor_row_t);
        if (covered + sizeof(frame_crc_t) > frame_bytes) { st.size_err++; continue; }

        uint32_t rx_crc;
        memcpy(&rx_crc, rx + covered, sizeof rx_crc);
        if (frame_crc_compute(rx, covered) != rx_crc) { st.crc_err++; continue; }

        int publish = 1;
        if (!have_last) {
            have_last = 1;
            last_seq  = h->seq;
        } else {
            int32_t diff = (int32_t)(h->seq - last_seq);
            if      (diff == 0) { st.duplicates++; publish = 0; }
            else if (diff == 1) { last_seq = h->seq; }
            else if (diff  > 1) { st.seq_drops += (uint64_t)(diff - 1); last_seq = h->seq; }
            else                { st.resets++; last_seq = h->seq; }
        }

        if (publish) {
            const motor_row_t *rows = (const motor_row_t *)(rx + sizeof(frame_header_t));
            motor_snapshot_publish(&region->snapshot, &rows[h->n_rows - 1],
                                   h->seq, h->timestamp, h->flags);
            motor_ring_publish(&region->ring, h, rows);
            st.frames_ok++;
        }
    }

    dataready_cleanup(&dr);
    rpi_spi_cleanup_device(cfg.spi_bus, cfg.spi_dev);
    munmap(region, sizeof(shm_region_t));
    shm_unlink(MOTOR_SHM_NAME);
    fprintf(stderr, "[ctrl] shutdown: ok=%" PRIu64 " drops=%" PRIu64
                    " crc=%" PRIu64 "\n", st.frames_ok, st.seq_drops, st.crc_err);
    return 0;
}