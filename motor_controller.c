/*
 * motor_controller.c
 * ----------------------------------------------------------------------------
 * QNX controller / producer node for the predictive-maintenance pipeline.
 * LEVEL-POLL build: dataready_wait polls the data-ready GPIO line every 100 us
 * with a 100 ms safety timeout. The STM32's DR pulse is too narrow (~100 ns) for
 * the Pi's GPIO debounce filter to detect as a rising edge, so edge-triggered
 * interrupts would always time out -- level polling is the reliable baseline.
 *
 * Single SCHED_FIFO thread that, per data-ready interrupt:
 *   1. reads one fixed-size frame over SPI (rpi_spi driver), full-duplex so the
 *      same transfer also DELIVERS any pending SET_CONFIG command to the STM32,
 *   2. validates magic / version / size / CRC and accounts for sequence gaps,
 *   3. publishes the latest row to the seqlock snapshot (for Qt) and the whole
 *      block to the lock-free ring (for the SOME/IP publisher),
 *   4. inspects frame_header_t.flags / _reserved for an ACK to any command in
 *      flight, and re-queues / clears it accordingly.
 *
 * Configuration is loaded from a JSON file at startup; SIGHUP (or, with -w, an
 * mtime change) triggers a reload. Live pi bits (rt priority, scaling) apply in
 * place; SPI/GPIO-tier pi fields reinitialise the bus (close -> rewrite spi.conf
 * + bounce spi-dwc -> reopen, plus a data-ready re-subscribe); STM-tier fields
 * go out as a SET_CONFIG command and are not considered active here until the
 * STM32 ACKs.
 *
 * GPIO ACCESS POLICY -- IMPORTANT:
 *   All GPIO access goes through the rpi_gpio resource manager (the client API
 *   in rpi_gpio.c / rpi_gpio.h). The server owns the RP1 hardware; this process
 *   does NOT map or poke RP1 registers directly. See the long comment in the
 *   pre-config revision for the history; the conclusion stands.
 *
 * Build (QNX): libc only -- do NOT link -lrt or -lpthread. Compile as C11.
 *   qcc -V<target> -std=gnu11 -O2 motor_controller.c config.c cJSON.c \
 *       rpi_gpio.c rpi_spi.c -lrpi_spi -o motor_controller
 *
 *   Drop cJSON.c + cJSON.h next to the controller source -- single-file
 *   library, no further deps. https://github.com/DaveGamble/cJSON
 *
 * Requires root to raise SCHED_FIFO priority. The rpi_gpio resource manager
 * must already be running (stock image: /dev/gpio is present).
 *
 * Usage:
 *   motor_controller [-w|--watch] [/path/to/config.json]
 *                                             (default: /etc/motor/config.json)
 *   kill -HUP $(pidof motor_controller)       (reload after the file changes)
 *   -w / --watch: also auto-reload when the config file's mtime changes (~1 Hz
 *                 poll). Off by default; SIGHUP always works regardless.
 * ----------------------------------------------------------------------------
 */
#ifndef _QNX_SOURCE
#define _QNX_SOURCE   /* expose nanosleep() under -std=gnu11 (as in config.c) */
#endif
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
#include <sys/stat.h>       /* stat() for the optional config-file watcher   */
#include <sys/neutrino.h>   /* ChannelCreate, ConnectAttach, MsgReceivePulse, TimerTimeout */

#include "motor_wire.h"
#include "motor_shm.h"
#include "rpi_spi.h"        /* rpi_spi_*, SPI_SUCCESS                       */
#include "rpi_gpio.h"       /* rpi_gpio_* client API                        */
#include "config.h"         /* full_config_t, JSON loader, SIGHUP plumbing  */

#ifndef DEFAULT_CONFIG_PATH
#define DEFAULT_CONFIG_PATH "/etc/motor/config.json"
#endif

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
    uint64_t cfg_reloads;
    uint64_t cfg_acks_ok;
    uint64_t cfg_nacks;
} controller_stats_t;

/* ============================ CRC-32/MPEG-2 ==============================
 * init 0xFFFFFFFF, poly 0x04C11DB7, MSB-first, no reflection, no final XOR.
 * Byte-for-byte identical to the STM32 crc32_mpeg2() in motor_send.c, and
 * reused for command frames going the other direction.                       */
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

static void pin_to_cpu(int cpu)
{
    if (cpu < 0) return;
    uint64_t runmask = 1ULL << cpu;
    ThreadCtl(_NTO_TCTL_RUNMASK, &runmask);
    fprintf(stderr, "[ctrl] pinned to CPU %d\n", cpu);
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

/* ============================ SPI bus lifecycle ==========================
 * open_spi() / close_spi() wrap the rpi_spi per-device configure/cleanup so
 * the exact same path is used at startup AND on a runtime SPI-tier reload.
 *
 * rpi_spi addresses the device by (bus,dev) rather than by a fd we hold, so we
 * remember which pair we actually configured. If a reload changes bus/dev,
 * close_spi() must still tear down the device we really opened, not the new
 * target -- hence the cached bus/dev here rather than reading cfg.pi at close
 * time. `configured` makes both calls idempotent. */
typedef struct {
    int  bus;
    int  dev;
    bool configured;
} spi_link_t;

static int open_spi(spi_link_t *s, const pi_config_t *pi)
{
    if (rpi_spi_configure_device(pi->spi_bus, pi->spi_dev, pi->spi_mode,
                                 pi->spi_clock_hz) != SPI_SUCCESS) {
        fprintf(stderr, "[ctrl] rpi_spi_configure_device(bus=%d dev=%d) failed\n",
                pi->spi_bus, pi->spi_dev);
        s->configured = false;
        return -1;
    }
    s->bus        = pi->spi_bus;
    s->dev        = pi->spi_dev;
    s->configured = true;
    return 0;
}

static void close_spi(spi_link_t *s)
{
    if (s->configured) {
        rpi_spi_cleanup_device(s->bus, s->dev);
        s->configured = false;
    }
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

static int dataready_read_level(dataready_t *d)
{
    unsigned level = GPIO_LOW;
    if (rpi_gpio_input(d->pin, &level) != GPIO_SUCCESS)
        return 0;
    return (level == GPIO_HIGH) ? 1 : 0;
}

static void dataready_teardown(dataready_t *d)
{
    /* Full teardown -- SHUTDOWN / STARTUP-ABORT ONLY. rpi_gpio_cleanup() closes
     * this client's shared connection to the rpi_gpio resource manager, and on
     * this driver that connection is opened lazily once and is NOT re-openable
     * in-process: any rpi_gpio_* call after it returns EBADF. So this must never
     * be called on a live reload path -- see dataready_repoint() for moving the
     * subscription at runtime. Fields are reset to -1 so it stays idempotent. */
    rpi_gpio_cleanup();
    if (d->coid != -1) { ConnectDetach(d->coid);  d->coid = -1; }
    if (d->chid != -1) { ChannelDestroy(d->chid); d->chid = -1; }
}

/* Arm the rising-edge subscription on cfg->dataready_pin against the pulse
 * channel already stored in *d (d->coid must be valid). Shared by first-time
 * setup and by a runtime pin move. Does NOT create/destroy the channel and does
 * NOT call rpi_gpio_cleanup(). Returns 0 on success, -1 on hard failure. */
static int dataready_arm_pin(dataready_t *d, const pi_config_t *cfg)
{
    d->pin = cfg->dataready_pin;

    if (rpi_gpio_setup(d->pin, GPIO_IN) != GPIO_SUCCESS) {
        fprintf(stderr, "error: rpi_gpio_setup(pin=%d) failed\n", d->pin);
        return -1;
    }
    if (rpi_gpio_setup_pull(d->pin, GPIO_IN, GPIO_PUD_DOWN) != GPIO_SUCCESS) {
        fprintf(stderr, "warning: rpi_gpio_setup_pull(pin=%d, DOWN) failed -- "
                        "fit an external pull-down resistor\n", d->pin);
    }
    if (rpi_gpio_add_event_detect(d->pin, d->coid, GPIO_RISING, DR_EVENT_ID)
            != GPIO_SUCCESS) {
        fprintf(stderr, "error: rpi_gpio_add_event_detect(pin=%d) failed\n", d->pin);
        return -1;
    }
    if (dataready_read_level(d) == 1) {
        fprintf(stderr, "WARNING: GPIO%d reads HIGH -- check wiring / pull-down "
                        "(spurious reads possible)\n", d->pin);
    } else {
        fprintf(stderr, "[ctrl] GPIO%d idle level low (pull-down active)\n", d->pin);
    }
    return 0;
}

static int dataready_setup(dataready_t *d, const pi_config_t *cfg)
{
    d->pin     = cfg->dataready_pin;
    d->chid    = -1;
    d->coid    = -1;
    d->poll_ns = 60000000ULL;          /* 60 ms safety timeout (> 40 ms STM32 block interval) */

    d->chid = ChannelCreate(0);
    if (d->chid == -1) {
        fprintf(stderr, "error: ChannelCreate failed: %s\n", strerror(errno));
        goto fail;
    }
    d->coid = ConnectAttach(0, 0, d->chid, _NTO_SIDE_CHANNEL, 0);
    if (d->coid == -1) {
        fprintf(stderr, "error: ConnectAttach failed: %s\n", strerror(errno));
        goto fail;
    }
    if (dataready_arm_pin(d, cfg) != 0)
        goto fail;
    return 0;

fail:
    /* Startup/abort path only -- full teardown (incl. rpi_gpio_cleanup) is safe
     * here because we are bailing out, not continuing to run. */
    dataready_teardown(d);
    return -1;
}

/* Move the data-ready edge subscription to cfg->dataready_pin at RUNTIME,
 * reusing the existing pulse channel (chid/coid stay put). Crucially this does
 * NOT call rpi_gpio_cleanup() -- the rpi_gpio connection stays open, so the new
 * rpi_gpio_setup()/add_event_detect() land on a live fd. The previous pin's
 * registration is left in place: it points at the same still-valid coid and,
 * with the STM's data-ready line physically moved to the new pin, simply goes
 * idle. Harmless -- dataready_wait() always re-reads the live level on the
 * current pin. Returns 0 on success, -1 on failure (old pin stays armed). */
static int dataready_repoint(dataready_t *d, const pi_config_t *cfg)
{
    return dataready_arm_pin(d, cfg);
}

/* Busy-wait for approximately `us` microseconds using CLOCK_MONOTONIC.
 * This bypasses the 1 ms QNX system-tick granularity that nanosleep
 * cannot escape (every sub-ms sleep rounds up to one full tick).          */
static void busy_nsleep(unsigned us)
{
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);
    uint64_t target = (uint64_t)start.tv_sec * 1000000000ULL
                    + start.tv_nsec + (uint64_t)us * 1000;
    do {
        clock_gettime(CLOCK_MONOTONIC, &now);
        uint64_t n = (uint64_t)now.tv_sec * 1000000000ULL + now.tv_nsec;
        if (n >= target) break;
    } while (1);
}

/* Tracks the monotonic timestamp of the last completed SPI transfer.
 * Used by dataready_wait to ensure minimum CS de-assert (settling)
 * between transfers without adding unnecessary fixed delays.           */
static uint64_t last_spi_ns = 0;

/* Safe monotonic-time delta: end_ns - start_ns in nanoseconds, handling
 * timespec nsec wraps correctly. */
static inline uint64_t ts_delta_ns(struct timespec end, struct timespec start)
{
    uint64_t ns = (uint64_t)(end.tv_sec - start.tv_sec) * 1000000000ULL;
    if (end.tv_nsec >= start.tv_nsec)
        ns += (uint64_t)(end.tv_nsec - start.tv_nsec);
    else
        ns -= (uint64_t)(start.tv_nsec - end.tv_nsec);
    return ns;
}

/* ---- self-tuning rate limiter -------------------------------------------
 * Normal operation: one SPI read every ~10 ms (matching the STM32 block
 * period). When a read returns garbage (magic mismatch), the controller
 * enters probe mode and retries at short intervals (500 us sleep) to
 * rapidly re-sync with the STM32's frame boundary. Once a valid frame
 * is found, probe mode exits and we return to the 10 ms cadence. The
/* (rate limiter moved to bottom of main loop; fixed 9.966 ms target) */
static int dataready_wait(dataready_t *d, controller_stats_t *st)
{
    (void)st;
    /* Wait for the RISING EDGE, not the level.
     *
     * The level is ambiguous. The STM32 holds data-ready high for the whole
     * transfer and the line only dips between frames, so a poll sampling about
     * once a millisecond sees one continuous high and cannot tell frame N from
     * N+1. Reads issued on that stale high land before the slave has armed its
     * DMA, and a dump of a failing read shows exactly that: a run of one
     * repeated byte -- the SPI data register handing back the same stale value
     * for every clock -- then the real frame, intact, 1 to 386 bytes in.
     *
     * The edge is unambiguous: one pulse per armed frame. The subscription
     * already exists (rpi_gpio_add_event_detect with GPIO_RISING, delivering to
     * this channel); it was created and then never actually waited on.
     *
     * MsgReceivePulse returns 0 on a real pulse, -1 when the timer wins.
     *
     * The fallback matters: if an edge is ever missed, waiting for it forever
     * would stall on a frame that has already passed. After DR_MISS_LIMIT
     * consecutive timeouts with the line still high, take the level instead --
     * the old behaviour, kept as a floor rather than as the normal path. */
    enum { DR_MISS_LIMIT = 2 };
    static int misses = 0;

    uint64_t to = 250000ULL;           /* 250 us: prefer the edge, but do not
                                        * spend a whole block period waiting */
    struct _pulse pulse;
    TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, NULL, &to, NULL);

    if (MsgReceivePulse(d->chid, &pulse, sizeof pulse, NULL) == 0) {
        misses = 0;
        return WR_OK;                  /* a frame was just armed */
    }

    if (++misses >= DR_MISS_LIMIT && dataready_read_level(d)) {
        misses = 0;
        return WR_OK;                  /* edge missed; fall back to the level */
    }
    return WR_TIMEOUT;
}
/* ============================ command transmitter ========================
 *
 * One outstanding command at a time. The same frame rides every SPI transfer
 * until it is ACK'd, NACK'd, or the retry budget is exhausted. State machine:
 *
 *   IDLE  --cmd_queue()-->  PENDING
 *   PENDING  --ACK_OK-->    IDLE  (cfg.stm copied into "active")
 *   PENDING  --NACK-->      IDLE  (active unchanged, error logged)
 *   PENDING  --retries gone--> IDLE  (active unchanged, error logged)
 *
 * Only the controller's main loop touches g_cmd. No locking needed (single
 * thread).
 */
#define CMD_MAX_RETRIES   200u  /* ~2-5 s margin at 25-37 Hz read rate */

typedef enum { CMD_IDLE = 0, CMD_PENDING = 1 } cmd_state_t;

static struct {
    cmd_state_t state;
    uint16_t    seq;            /* monotonic, sent in cmd_header_t.cmd_seq    */
    uint16_t    retries_left;
    uint8_t     frame[MOTOR_CMD_FRAME_BYTES];
    config_payload_t payload;   /* what we asked for (in flight)              */
} g_cmd;

static void cmd_init(void)
{
    memset(&g_cmd, 0, sizeof g_cmd);
    g_cmd.state = CMD_IDLE;
    g_cmd.seq   = 0;
}

/* Build a SET_CONFIG frame in g_cmd.frame for the given payload. */
static void cmd_build_set_config(const config_payload_t *p)
{
    memset(g_cmd.frame, 0, sizeof g_cmd.frame);
    cmd_header_t *h = (cmd_header_t *)g_cmd.frame;
    h->magic          = MOTOR_CMD_MAGIC;
    h->cmd            = MOTOR_CMD_SET_CONFIG;
    h->schema_version = MOTOR_CONFIG_SCHEMA_VERSION;
    h->cmd_seq        = ++g_cmd.seq;
    h->_pad           = 0;

    config_payload_t *body = (config_payload_t *)(g_cmd.frame + sizeof(cmd_header_t));
    *body = *p;
    memset(body->reserved, 0, sizeof body->reserved);

    size_t covered = sizeof(cmd_header_t) + sizeof(config_payload_t);
    uint32_t crc = frame_crc_compute(g_cmd.frame, covered);
    memcpy(g_cmd.frame + covered, &crc, sizeof crc);
}

static void cmd_queue_set_config(const config_payload_t *p)
{
    g_cmd.payload      = *p;
    cmd_build_set_config(p);
    g_cmd.state        = CMD_PENDING;
    g_cmd.retries_left = CMD_MAX_RETRIES;
    fprintf(stderr, "[ctrl] SET_CONFIG queued seq=%u (block_rows=%u, run=%u)\n",
            g_cmd.seq, g_cmd.payload.block_rows, g_cmd.payload.run_state);
}

/* Fill the outbound tx for one SPI exchange. Does NOT decrement retries --
 * that happens only after an exchange actually completes (see main loop). */
static void cmd_fill_tx(uint8_t *tx, size_t tx_len)
{
    memset(tx, 0, tx_len);
    if (g_cmd.state == CMD_PENDING) {
        memcpy(tx, g_cmd.frame, sizeof g_cmd.frame);
    }
}

/* Call once per successful SPI exchange (i.e. the command was actually
 * clocked out). Decrements the retry budget for the in-flight command.   */
static void cmd_count_attempt(void)
{
    if (g_cmd.state == CMD_PENDING && g_cmd.retries_left > 0)
        g_cmd.retries_left--;
}

/* Called right after the SPI bus is bounced/reopened. No exchanges happen
 * during a bounce, so the retry budget isn't consumed there -- but a command
 * that was already near its limit shouldn't be abandoned just because we lost
 * a few seconds to the restart. Hand any in-flight command a full budget again
 * so the bounce window itself can never cause a SET_CONFIG to be given up on.
 * The frame stays byte-identical (same cmd_seq), so the STM re-ACKs it
 * idempotently once exchanges resume -- no ACK is lost. */
static void cmd_refresh_after_reinit(void)
{
    if (g_cmd.state == CMD_PENDING)
        g_cmd.retries_left = CMD_MAX_RETRIES;
}

/* Inspect an incoming frame header for an ACK. Returns true and updates
 * *active_stm on a successful apply (so the caller can lock its expectations
 * to the new config). */
static bool cmd_observe(const frame_header_t *h,
                        config_payload_t *active_stm,
                        controller_stats_t *st)
{
    if (g_cmd.state != CMD_PENDING) return false;
    if (h->_reserved != g_cmd.seq) {
        /* Not our ACK (could be a stale ack from a previous seq). Check the
         * retry budget. */
        if (g_cmd.retries_left == 0) {
            fprintf(stderr, "[ctrl] SET_CONFIG seq=%u: no ACK after retries, giving up\n",
                    g_cmd.seq);
            st->cfg_nacks++;
            g_cmd.state = CMD_IDLE;
        }
        return false;
    }
    /* This ACK matches our in-flight command. */
    if (h->flags & MOTOR_FLAG_ACK_OK) {
        *active_stm = g_cmd.payload;       /* now authoritative on the Pi side */
        g_cmd.state = CMD_IDLE;
        st->cfg_acks_ok++;
        fprintf(stderr, "[ctrl] SET_CONFIG seq=%u ACK_OK%s\n", g_cmd.seq,
                (h->flags & MOTOR_FLAG_CONFIG_APPLIED) ? " (config applied)" : "");
        return true;
    }
    if (h->flags & MOTOR_FLAG_ACK_NACK) {
        fprintf(stderr, "[ctrl] SET_CONFIG seq=%u NACK%s%s%s%s\n", g_cmd.seq,
                (h->flags & MOTOR_FLAG_NACK_RANGE) ? " RANGE"  : "",
                (h->flags & MOTOR_FLAG_NACK_CRC)   ? " CRC"    : "",
                (h->flags & MOTOR_FLAG_NACK_VER)   ? " VER"    : "",
                (h->flags & MOTOR_FLAG_NACK_CMD)   ? " CMD"    : "");
        st->cfg_nacks++;
        g_cmd.state = CMD_IDLE;
        return false;
    }
    /* ACK seq matches but no ack bits set yet -- the apply is still in
     * progress. Keep the command in tx; we'll see the bits in a later frame. */
    return false;
}

/* ============================ pi-tier apply ==============================
 * The pi tier splits in two:
 *
 *   - Fields that apply live, with no bus teardown -- handled HERE. Currently
 *     real-time priority (and the scaling constants, which are just exposed to
 *     downstream consumers via shm).
 *
 *   - Fields that require reinitialising the SPI bus and/or moving the
 *     data-ready GPIO subscription (spi_bus/dev/mode/clock_hz/cpha/word_width/
 *     idle_insert, dataready_pin) -- handled by handle_reload(): the SPI wire
 *     subset (config_spi_wire_differs) drives close_spi -> spi_apply_conf ->
 *     open_spi, and a dataready_pin change independently drives
 *     dataready_repoint(). Keep this function's field list complementary to
 *     those.  */
static void pi_apply_live(const pi_config_t *pi, const pi_config_t *prev)
{
    if (pi->rt_priority != prev->rt_priority) {
        if (set_realtime_priority(pi->rt_priority) != 0) {
            fprintf(stderr, "[ctrl] rt_priority %d not set: %s\n",
                    pi->rt_priority, strerror(errno));
        } else {
            fprintf(stderr, "[ctrl] rt_priority -> %d\n", pi->rt_priority);
        }
    }
    (void)pi; (void)prev;
    /* Scaling constants are read by downstream consumers from shm; the
     * publisher just needs to expose them. (Hookup TBD; not on the wire.) */
    (void)prev;
}

/* ============================ shutdown ==================================== */
static volatile sig_atomic_t g_running = 1;
static void on_signal(int sig) { (void)sig; g_running = 0; }

/* ============================ config-file watch ==========================
 * Optional (off by default, enable with -w/--watch): stat the config file about
 * once a second and trigger the SAME reload path as SIGHUP when its mtime or
 * size changes. Deliberately an integrated tick in the main loop rather than a
 * separate thread, so the single-threaded, lock-free command state machine
 * (see g_cmd) stays exactly that -- no new synchronization is introduced.
 *
 * (A thread variant is trivial if you ever want one: have it stat + set a
 * sig_atomic_t flag that the loop OR-s into reload_due, keeping the actual
 * reload work on the main thread. The in-loop poll below avoids even that.)  */
typedef struct {
    bool             enabled;
    const char      *path;
    time_t           last_stat_sec;    /* loop-monotonic sec of last stat()   */
    struct timespec  mtime;            /* last observed file mtime            */
    off_t            size;             /* last observed file size             */
    bool             have_baseline;
    bool             trigger_on_first; /* started on defaults: load ASAP once
                                          the file appears/parses             */
} cfg_watch_t;

static bool cfg_stat(const char *path, struct timespec *mtime, off_t *size)
{
    struct stat stt;
    if (stat(path, &stt) != 0) return false;
#if defined(__QNX__)
    *mtime = stt.st_mtim;              /* nanosecond mtime on QNX             */
#else
    mtime->tv_sec  = stt.st_mtime;     /* portable fallback (host builds)     */
    mtime->tv_nsec = 0;
#endif
    *size = stt.st_size;
    return true;
}

/* Record the current on-disk state as the baseline. Call after any (re)load so
 * a reload we just performed doesn't immediately re-trigger on the next tick. */
static void cfg_watch_sync(cfg_watch_t *w)
{
    if (!w->enabled) return;
    if (cfg_stat(w->path, &w->mtime, &w->size)) w->have_baseline = true;
}

/* Returns true at most ~once per second, and only when mtime or size moved
 * since the baseline. `now_sec` is the loop's CLOCK_MONOTONIC second, used
 * purely to rate-limit the stat(). Updates the baseline when it fires. */
static bool cfg_watch_fired(cfg_watch_t *w, time_t now_sec)
{
    if (!w->enabled) return false;
    if (now_sec == w->last_stat_sec) return false;   /* one stat per second   */
    w->last_stat_sec = now_sec;

    struct timespec mt; off_t sz;
    if (!cfg_stat(w->path, &mt, &sz)) {
        /* File momentarily absent -- e.g. an atomic rename mid-swap. Skip this
         * tick; the replacement is caught next second. */
        return false;
    }
    if (!w->have_baseline) {
        w->mtime = mt; w->size = sz; w->have_baseline = true;
        if (w->trigger_on_first) { w->trigger_on_first = false; return true; }
        return false;
    }
    if (mt.tv_sec != w->mtime.tv_sec || mt.tv_nsec != w->mtime.tv_nsec ||
        sz != w->size) {
        w->mtime = mt; w->size = sz;
        return true;
    }
    return false;
}

/* ============================ reload driver ==============================
 * Applies a freshly loaded config. MUST be called only at a safe point in the
 * main loop -- never from a signal handler, and never with an SPI exchange in
 * flight (the loop guarantees this: the reload flag is polled at the top of the
 * iteration, after the previous exchange has fully returned). Steps:
 *
 *   - live pi bits (rt priority, scaling)                       -> always
 *   - SPI bus reinit (close_spi -> spi_apply_conf -> open_spi)  -> iff an SPI
 *                                                                  wire field
 *                                                                  changed
 *   - data-ready GPIO move (dataready_repoint)                  -> iff the
 *                                                                  dataready_pin
 *                                                                  changed
 *   - SET_CONFIG to the STM                                     -> iff the
 *                                                                  STM-tier subset
 *                                                                  changed
 *
 * The SPI bus and the data-ready GPIO are independent resource managers, so
 * they are reinitialised independently: a clock/mode change bounces spi-dwc but
 * leaves the edge subscription untouched, and a pin change moves the edge
 * subscription without disturbing the bus.
 *
 * Returns 0 if a config was applied, -1 if the new file was rejected (running
 * config left untouched -- previous known-good stays in force). */
static int handle_reload(const char *cfg_path,
                         full_config_t      *cfg,
                         spi_link_t          *spi,
                         dataready_t        *dr,
                         controller_stats_t *st,
                         int                *have_last)
{
    full_config_t next;
    if (config_load_file(cfg_path, &next) != 0) {
        fprintf(stderr, "[ctrl] reload failed; keeping previous config\n");
        return -1;
    }

    const bool spi_changed = config_spi_wire_differs(&next.pi, &cfg->pi);
    const bool pin_changed = (next.pi.dataready_pin != cfg->pi.dataready_pin);
    const bool hw          = spi_changed || pin_changed;

    /* Live pi bits first -- safe irrespective of bus state. */
    pi_apply_live(&next.pi, &cfg->pi);

    /* --- SPI wire tier: bounce the bus only if a wire parameter changed. The
     * data-ready GPIO is a *separate* resource manager and is deliberately left
     * alone here -- bouncing spi-dwc does not affect edge delivery. --------- */
    if (spi_changed) {
        fprintf(stderr, "[ctrl] reload: SPI wire change -> reopening bus\n");

        /* Clean boundary: no transfer is in flight (the reload flag is polled at
         * the top of the loop, after the previous exchange fully returned). The
         * spi-dwc restart invalidates the SPI fd, so close before the bounce. */
        close_spi(spi);

        /* Write new SPI params to spi.conf and bounce spi-dwc so the change
         * takes effect immediately. The STM32 is about to be re-synced via
         * SET_CONFIG (or the seq baseline reset via *have_last = 0), so a
         * brief bus glitch during the bounce is tolerated here -- unlike the
         * startup path where the STM32 has been running independently.      */
        (void)0;

        /* Reopen on the (possibly new) bus/dev/mode/clock. spi-dwc may still be
         * settling right after a restart, so give the open a few tries before
         * giving up for this reload (the loop keeps running and a later reload
         * can recover). */
        int tries = 0;
        while (open_spi(spi, &next.pi) != 0) {
            if (++tries >= 5) {
                fprintf(stderr, "[ctrl] ERROR: SPI reopen failed after bounce; "
                                "bus stays down until the next reload\n");
                break;
            }
            const struct timespec backoff = { .tv_sec = 0, .tv_nsec = 200*1000*1000 };
            nanosleep(&backoff, NULL);
        }

        /* The STM ran free through the bounce; its frame seq advanced while we
         * were deaf. Drop our seq baseline so the first post-bounce frame just
         * re-establishes it, instead of logging one giant spurious gap. */
        *have_last = 0;

        /* Don't let the exchange-less bounce window burn a pending command. */
        cmd_refresh_after_reinit();
    }

    /* --- Data-ready GPIO tier: move the edge subscription only if the pin
     * changed. Reuses the existing pulse channel and never calls
     * rpi_gpio_cleanup(), so the rpi_gpio connection stays alive. --------- */
    if (pin_changed) {
        fprintf(stderr, "[ctrl] reload: data-ready pin %d -> %d\n",
                cfg->pi.dataready_pin, next.pi.dataready_pin);
        if (dataready_repoint(dr, &next.pi) != 0) {
            fprintf(stderr, "[ctrl] ERROR: data-ready re-point to GPIO%d failed; "
                            "still armed on GPIO%d\n",
                    next.pi.dataready_pin, dr->pin);
        }
    }

    /* Adopt the new pi config now that the hardware reflects it. */
    cfg->pi = next.pi;

    /* STM tier: push a SET_CONFIG only if the STM-visible subset changed. A
     * command already pending (including one that just survived the bounce)
     * stays pending and keeps riding every exchange until ACK'd -- no ACK is
     * lost across the reinit. */
    if (config_stm_differs(&next.stm, &cfg->stm)) {
        cfg->stm = next.stm;
        cmd_queue_set_config(&cfg->stm);
    } else if (!hw) {
        fprintf(stderr, "[ctrl] reload: pi-live-only changes\n");
    }

    st->cfg_reloads++;
    return 0;
}

/* ============================ main ======================================== */
int main(int argc, char **argv)
{
    /* ---- args: [-w|--watch] [/path/to/config.json] ---- */
    const char *cfg_path      = DEFAULT_CONFIG_PATH;
    bool        watch_enabled = false;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-w") == 0 || strcmp(argv[i], "--watch") == 0) {
            watch_enabled = true;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            fprintf(stderr,
                "usage: %s [-w|--watch] [/path/to/config.json]\n"
                "  -w, --watch   poll the config file's mtime (~1 Hz) and reload\n"
                "                on change, in addition to SIGHUP. Off by default\n"
                "                so SIGHUP-only deployments are unaffected.\n",
                argv[0]);
            return 0;
        } else if (argv[i][0] == '-') {
            fprintf(stderr, "%s: unknown option '%s' (try --help)\n", argv[0], argv[i]);
            return 2;
        } else {
            cfg_path = argv[i];   /* first non-flag arg is the config path */
        }
    }

    /* ---- load config (fall back to compiled defaults on failure) ---- */
    full_config_t cfg;
    bool startup_loaded_ok = (config_load_file(cfg_path, &cfg) == 0);
    if (startup_loaded_ok) {
        fprintf(stderr, "[ctrl] loaded config: %s\n", cfg_path);
    } else {
        fprintf(stderr, "[ctrl] config load failed; using built-in defaults\n");
        cfg = CONFIG_DEFAULTS;
    }

    /* The "active" STM config is what we believe the STM32 is currently
     * running. It starts as a sentinel (block_rows = 0) meaning "unknown" --
     * we will sync via SET_CONFIG before locking down the size check.        */
    config_payload_t active_stm = {0};

    if (set_realtime_priority(cfg.pi.rt_priority) != 0)
        fprintf(stderr, "warning: SCHED_FIFO prio %d not set (need privilege): %s\n",
                cfg.pi.rt_priority, strerror(errno));

    pin_to_cpu(cfg.pi.cpu_affinity);

    (void)0;

    config_install_sighup();

    shm_region_t *region = shm_setup();
    if (!region) { perror("shm_setup"); return 1; }

    /* Sync spi.conf so the driver clock matches. Safe here because no SPI
     * exchange is in progress yet (open_spi is called next).                */
    (void)spi_apply_conf(&cfg.pi);

    spi_link_t spi = { .configured = false };
    if (open_spi(&spi, &cfg.pi) != 0) {
        munmap(region, sizeof(shm_region_t));
        shm_unlink(MOTOR_SHM_NAME);
        return 1;
    }

    {
        spi_devinfo_t devinfo;
        if (rpi_spi_get_device_info(cfg.pi.spi_bus, cfg.pi.spi_dev, &devinfo) == SPI_SUCCESS)
            fprintf(stderr, "[ctrl] SPI actual clock: %u Hz (requested %u Hz)\n",
                    devinfo.current_clkrate, cfg.pi.spi_clock_hz);
    }

    dataready_t dr;
    if (dataready_setup(&dr, &cfg.pi) != 0) {
        /* dataready_setup() self-cleans on failure; just close the bus. */
        close_spi(&spi);
        munmap(region, sizeof(shm_region_t));
        shm_unlink(MOTOR_SHM_NAME);
        return 1;
    }
    fprintf(stderr, "[ctrl] edge-pulse mode on GPIO\n");

    /* ---- optional config-file watcher ---- */
    cfg_watch_t watch = { .enabled = watch_enabled, .path = cfg_path };
    if (watch.enabled) {
        if (startup_loaded_ok) {
            cfg_watch_sync(&watch);            /* baseline = the good file we loaded */
        } else {
            watch.trigger_on_first = true;     /* on defaults: grab the file when it lands */
        }
        fprintf(stderr, "[ctrl] config-file watch enabled: polling %s (~1 Hz)\n",
                cfg_path);
    }

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    struct sigaction sa_ign;
    memset(&sa_ign, 0, sizeof sa_ign);
    sa_ign.sa_handler = SIG_IGN;
    sigaction(SIGHUP, &sa_ign, NULL);

    /* The SPI exchange size is the WORST case -- buffers are sized to MAX so
     * a runtime block_rows change does not require a reallocation. We always
     * clock MOTOR_MAX_FRAME_BYTES; the actual valid payload length comes from
     * h->n_rows inside the frame. This also gives the command frame plenty of
     * headroom on the tx side (it's far smaller than MAX). The STM32 SPI slave
     * requires the full clock cycle with CS asserted; truncating causes sync
     * loss so this stays at MAX regardless of the active block_rows.         */
    const size_t xfer_bytes = MOTOR_MAX_FRAME_BYTES;

    static _Alignas(8) uint8_t rx[MOTOR_MAX_FRAME_BYTES];
    static _Alignas(8) uint8_t tx[MOTOR_MAX_FRAME_BYTES];

    /* ---- queue an initial SET_CONFIG so the STM32 matches what we loaded - */
    cmd_init();
    (void)cmd_queue_set_config;

    controller_stats_t st = {0};
    uint32_t last_seq = 0;
    int      have_last = 0;
    uint16_t last_n_rows = 0;
    uint16_t last_h_flags = 0;       /* diagnostic: last received header  */
    uint16_t last_h_reserved = 0;
    /* diagnostic tap for sensor values -- helps eyeball whether real data is
     * flowing without having to attach a debugger or open shm from another
     * process. Updated on every published frame; printed in the status log. */
    int16_t  last_vib_x = 0, last_vib_y = 0, last_vib_z = 0;
    uint16_t last_cur0 = 0, last_cur1 = 0, last_cur2 = 0;
    uint16_t last_rpm = 0;
    struct timespec t_log;
    clock_gettime(CLOCK_MONOTONIC, &t_log);

    /* ---- timing instrumentation (diagnostic) ---- */
    uint64_t t_spi_ns = 0, t_pub_ns = 0;
    uint64_t t_spi_max = 0, t_pub_max = 0;
    uint32_t t_samples = 0;
    struct timespec t_spi_start, t_spi_end;
    uint64_t loop_count = 0;            /* raw iterations per interval               */
    int bad_read_count = 0;             /* consecutive SPI devctl failures          */
    uint64_t wait_ns_sum = 0, wait_ns_min = UINT64_MAX, wait_ns_max = 0;
    uint32_t wait_samples = 0;

    while (g_running) {
        loop_count++;
        struct timespec t_loop_start;
        clock_gettime(CLOCK_MONOTONIC, &t_loop_start);

        /* ---- periodic status log ---- */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec != t_log.tv_sec) {
            t_log = now;
            fprintf(stderr,
                "[ctrl] ok=%" PRIu64 " drops=%" PRIu64 " crc=%" PRIu64
                " magic=%" PRIu64 " ver=%" PRIu64 " size=%" PRIu64
                " dup=%" PRIu64 " rst=%" PRIu64 " to=%" PRIu64
                " spi=%" PRIu64 " cfg(rld=%" PRIu64 " ack=%" PRIu64 " nack=%" PRIu64 ")"
                 " last(flags=0x%04x rsv=%u rows=%u)"
                 " sens(cur=%u/%u/%u vib=%d/%d/%d rpm=%u)"
                " bad=%d"
                 " timing(wait=%" PRIu64 "/%" PRIu64 " spi=%" PRIu64 "/%" PRIu64 " pub=%" PRIu64 "/%" PRIu64
                 " us)\n",
                st.frames_ok, st.seq_drops, st.crc_err, st.magic_err,
                st.version_err, st.size_err, st.duplicates, st.resets,
                st.timeouts, st.spi_err,
                st.cfg_reloads, st.cfg_acks_ok, st.cfg_nacks,
                last_h_flags, last_h_reserved, last_n_rows,
                last_cur0, last_cur1, last_cur2,
                last_vib_x, last_vib_y, last_vib_z, last_rpm,
                bad_read_count,
                wait_samples ? wait_ns_sum / wait_samples / 1000 : 0,
                wait_ns_max / 1000,
                t_samples ? t_spi_ns  / t_samples / 1000 : 0,
                t_samples ? t_spi_max / 1000 : 0,
                t_samples ? t_pub_ns  / t_samples / 1000 : 0,
                t_samples ? t_pub_max / 1000 : 0);
            t_spi_ns = t_pub_ns = 0;
            t_spi_max = t_pub_max = 0;
            t_samples = 0;
            wait_ns_sum = 0; wait_ns_min = UINT64_MAX; wait_ns_max = 0; wait_samples = 0;
        }

        /* ---- reload trigger: SIGHUP and/or config-file mtime change ----
         * Both funnel into the identical handle_reload() path. The watcher poll
         * is rate-limited to ~1 Hz internally (uses this loop's monotonic sec).
         * handle_reload runs here, at the top of the iteration, so it can never
         * interleave with an in-flight SPI exchange further down. */
        bool reload_due = config_reload_requested();
        if (cfg_watch_fired(&watch, now.tv_sec)) {
            fprintf(stderr, "[ctrl] config file changed on disk -> reload\n");
            reload_due = true;
        }
        if (reload_due) {
            if (handle_reload(cfg_path, &cfg, &spi, &dr, &st, &have_last) == 0)
                cfg_watch_sync(&watch);   /* rebase so we don't re-trigger */
        }

        /* ---- prepare tx (carries SET_CONFIG if one is in flight) ---- */
        cmd_fill_tx(tx, sizeof tx);

        /* ---- wait for STM32 data-ready edge ---- */
        {
            struct timespec t_wait_start;
            clock_gettime(CLOCK_MONOTONIC, &t_wait_start);
            int w = dataready_wait(&dr, &st);
            {
                struct timespec t_wait_end;
                clock_gettime(CLOCK_MONOTONIC, &t_wait_end);
                uint64_t w_ns = ts_delta_ns(t_wait_end, t_wait_start);
                wait_ns_sum += w_ns;
                if (w_ns < wait_ns_min) wait_ns_min = w_ns;
                if (w_ns > wait_ns_max) wait_ns_max = w_ns;
                wait_samples++;
            }
            if (w == WR_TIMEOUT) { st.timeouts++; continue; }
            if (w == WR_ERROR)   { st.spi_err++;   continue; }
        }

        /* ---- SPI exchange (timed) ---- */
        clock_gettime(CLOCK_MONOTONIC, &t_spi_start);
        if (rpi_spi_write_read_data(cfg.pi.spi_bus, cfg.pi.spi_dev, tx, rx,
                                    xfer_bytes) != SPI_SUCCESS) {
            if (errno == EINTR) continue;   /* interrupted (e.g. SIGHUP): clean abort */
            st.spi_err++;
            bad_read_count++;
            if (bad_read_count >= 3) {
                fprintf(stderr, "[ctrl] %d consecutive SPI errors -- resetting bus\n", bad_read_count);
                close_spi(&spi);
                const struct timespec rto = { .tv_sec = 0, .tv_nsec = 100000000 };
                nanosleep(&rto, NULL);
                if (open_spi(&spi, &cfg.pi) != 0)
                    fprintf(stderr, "[ctrl] SPI reopen failed after error recovery\n");
                have_last = 0;
                bad_read_count = 0;
            }
            continue;
        }
        bad_read_count = 0;
        clock_gettime(CLOCK_MONOTONIC, &t_spi_end);
        last_spi_ns = (uint64_t)t_spi_end.tv_sec * 1000000000ULL + t_spi_end.tv_nsec;
        {
            uint64_t ns = ts_delta_ns(t_spi_end, t_spi_start);
            t_spi_ns += ns;
            if (ns > t_spi_max) t_spi_max = ns;
        }

        const frame_header_t *h = (const frame_header_t *)rx;

        /* ---- frame validation ----
         * NO validation failure skips the rate limiter at the bottom of the
         * loop.  The old probe-mode design used `continue` on bad magic,
         * consuming frames at high speed and making it impossible for the
         * STM32 to ever recover from a boundary slip.  With the CS-wait
         * recovery on the STM32 side, every frame after a bad one is cleanly
         * aligned; the correct response is to pace reads at the 10 ms block
         * cadence and pick up the next good frame.  A bad frame is simply
         * not published.                                                      */
        int frame_valid = 1;

        if (h->magic != MOTOR_FRAME_MAGIC) { st.magic_err++;   have_last = 0; frame_valid = 0; }
        if (h->version != MOTOR_CONTRACT_VERSION)            { st.version_err++; have_last = 0; frame_valid = 0; }
        if (h->n_rows == 0 || h->n_rows > MOTOR_MAX_ROWS_PER_BLOCK) {
            st.size_err++; have_last = 0; frame_valid = 0;
        }
        if (frame_valid && active_stm.block_rows != 0
            && h->n_rows != active_stm.block_rows
            && !(h->flags & MOTOR_FLAG_CONFIG_APPLIED)) {
            st.size_err++; have_last = 0; frame_valid = 0;
        }

        size_t covered = 0;
        if (frame_valid) {
            covered = sizeof(frame_header_t) + (size_t)h->n_rows * sizeof(motor_row_t);
            if (covered + sizeof(frame_crc_t) > xfer_bytes) {
                st.size_err++; have_last = 0; frame_valid = 0;
            }
        }

        if (frame_valid) {
            uint32_t rx_crc;
            memcpy(&rx_crc, rx + covered, sizeof rx_crc);
            if (frame_crc_compute(rx, covered) != rx_crc) {
                st.crc_err++; have_last = 0; frame_valid = 0;
            }
        }

        if (frame_valid) {
            /* Command was actually clocked out AND yielded a valid frame. */
            cmd_count_attempt();

            /* ---- ACK handling: may update active_stm ---- */
            if (cmd_observe(h, &active_stm, &st)) {
                have_last = 0;
            }

            last_h_flags    = h->flags;
            last_h_reserved = h->_reserved;

            if ((h->flags & MOTOR_FLAG_CONFIG_APPLIED) && h->n_rows != last_n_rows)
                fprintf(stderr, "[ctrl] block_rows -> %u\n", h->n_rows);
            last_n_rows = h->n_rows;

            /* ---- sequence accounting ---- */
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

            /* ---- publish ---- */
            if (publish) {
                struct timespec t_pub_start;
                clock_gettime(CLOCK_MONOTONIC, &t_pub_start);
                const motor_row_t *rows = (const motor_row_t *)(rx + sizeof(frame_header_t));
                const motor_row_t *last = &rows[h->n_rows - 1];
                last_cur0  = last->current[0];
                last_cur1  = last->current[1];
                last_cur2  = last->current[2];
                last_vib_x = last->vib_x;
                last_vib_y = last->vib_y;
                last_vib_z = last->vib_z;
                last_rpm   = last->rpm;

                motor_snapshot_publish(&region->snapshot, last,
                                       h->seq, h->timestamp, h->flags);
                motor_ring_publish(&region->ring, h, rows, cfg.stm.sample_rate_hz);
                st.frames_ok++;

                struct timespec t_pub_end;
                clock_gettime(CLOCK_MONOTONIC, &t_pub_end);
                {
                uint64_t ns = ts_delta_ns(t_pub_end, t_pub_start);
                    t_pub_ns += ns;
                    if (ns > t_pub_max) t_pub_max = ns;
                }
            }
        }
        t_samples++;
    }

    dataready_teardown(&dr);
    close_spi(&spi);
    munmap(region, sizeof(shm_region_t));
    shm_unlink(MOTOR_SHM_NAME);
    fprintf(stderr, "[ctrl] shutdown: ok=%" PRIu64 " drops=%" PRIu64
                    " crc=%" PRIu64 " cfg_ack=%" PRIu64 " cfg_nack=%" PRIu64 "\n",
            st.frames_ok, st.seq_drops, st.crc_err,
            st.cfg_acks_ok, st.cfg_nacks);
    return 0;
}