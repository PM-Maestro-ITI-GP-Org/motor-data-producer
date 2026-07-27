/*
 * config.c
 * ----------------------------------------------------------------------------
 * JSON loader for the motor_controller. Uses cJSON (drop cJSON.c / cJSON.h in
 * next to this file -- https://github.com/DaveGamble/cJSON, MIT). Single-pass:
 * parse, range-check EVERY field, then commit. On any error we leave *out
 * untouched and return -1 so the caller can keep the previous known-good
 * config (essential for OTA rollback later).
 *
 * Build:
 *   qcc -V<target> -std=gnu11 -O2 -c config.c
 *   qcc -V<target> -std=gnu11 -O2 -c cJSON.c
 *   ...then link both into motor_controller as usual.
 * ----------------------------------------------------------------------------
 */
#define _QNX_SOURCE   /* expose nanosleep() and friends under -std=gnu11 */

#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>        /* nanosleep() in bounce_spi_dwc()               */
#include <sys/stat.h>

#include "cJSON.h"

/* ============================ defaults ==================================== */
const full_config_t CONFIG_DEFAULTS = {
    .pi = {
        .spi_bus         = 0,
        .spi_dev         = 0,
        .spi_mode        = 0,
        .spi_clock_hz    = 4000000u,
        .spi_cpha        = 0,
        .spi_word_width  = 8,
        .spi_idle_insert = 0,
        .rt_priority     = 30,
        .dataready_pin   = 17,
        .current_scale   = 1.0f,  .current_offset = 0.0f,
        .vib_scale       = 1.0f,  .vib_offset     = 0.0f,
        .rpm_scale       = 1.0f,  .rpm_offset     = 0.0f,
    },
    .stm = {
        .block_rows      = 200,
        ._reserved0      = 0,
        .source          = MOTOR_SOURCE_ADC,
        .run_state       = MOTOR_RUN_RUN,
        .sample_rate_hz  = 20000u,  /* 20 kHz ADC scan of 8 channels */
        .imu_rate_hz     = 1000u,   /* 1 kHz MPU6050 poll (native max) */
        .reserved        = {0},
    },
};

/* ============================ SIGHUP plumbing ============================= */
static volatile sig_atomic_t g_reload = 0;
static void on_sighup(int s) { (void)s; g_reload = 1; }

void config_install_sighup(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_sighup;
    sa.sa_flags   = 0;   /* main loop handles EINTR -- no SA_RESTART needed */
    sigaction(SIGHUP, &sa, NULL);
}

bool config_reload_requested(void)
{
    if (g_reload) { g_reload = 0; return true; }
    return false;
}

/* ============================ diff ======================================== */
bool config_stm_differs(const config_payload_t *a, const config_payload_t *b)
{
    return memcmp(a, b, sizeof *a) != 0;
}

/* True if any Pi field that requires reinitialising the SPI bus or the
 * data-ready GPIO subscription changed. rt_priority and the scaling constants
 * are excluded on purpose -- those are applied live by pi_apply_live() and do
 * not need a bus bounce. Keep this in sync with pi_apply_live(): every pi field
 * is handled either here (needs reinit) or there (applies live). */
bool config_spi_wire_differs(const pi_config_t *a, const pi_config_t *b)
{
    return a->spi_bus         != b->spi_bus
        || a->spi_dev         != b->spi_dev
        || a->spi_mode        != b->spi_mode
        || a->spi_clock_hz    != b->spi_clock_hz
        || a->spi_cpha        != b->spi_cpha
        || a->spi_word_width  != b->spi_word_width
        || a->spi_idle_insert != b->spi_idle_insert;
}

bool config_pi_hw_differs(const pi_config_t *a, const pi_config_t *b)
{
    return config_spi_wire_differs(a, b)
        || a->dataready_pin   != b->dataready_pin;
}

/* ============================ JSON helpers ================================
 * All helpers return 0 on success, -1 if the field is present but invalid.
 * A missing optional field leaves *dst alone (default carried in from caller).
 */
static int j_int(const cJSON *root, const char *name, int *dst,
                 int lo, int hi, bool required)
{
    const cJSON *n = cJSON_GetObjectItemCaseSensitive(root, name);
    if (!n) {
        if (required) { fprintf(stderr, "config: missing '%s'\n", name); return -1; }
        return 0;
    }
    if (!cJSON_IsNumber(n)) {
        fprintf(stderr, "config: '%s' must be a number\n", name);
        return -1;
    }
    int v = (int)n->valuedouble;
    if (v < lo || v > hi) {
        fprintf(stderr, "config: '%s' = %d out of range [%d..%d]\n", name, v, lo, hi);
        return -1;
    }
    *dst = v;
    return 0;
}

static int j_uint(const cJSON *root, const char *name, uint32_t *dst,
                  uint32_t lo, uint32_t hi, bool required)
{
    const cJSON *n = cJSON_GetObjectItemCaseSensitive(root, name);
    if (!n) {
        if (required) { fprintf(stderr, "config: missing '%s'\n", name); return -1; }
        return 0;
    }
    if (!cJSON_IsNumber(n) || n->valuedouble < 0.0) {
        fprintf(stderr, "config: '%s' must be a non-negative number\n", name);
        return -1;
    }
    uint32_t v = (uint32_t)n->valuedouble;
    if (v < lo || v > hi) {
        fprintf(stderr, "config: '%s' = %u out of range [%u..%u]\n",
                name, (unsigned)v, (unsigned)lo, (unsigned)hi);
        return -1;
    }
    *dst = v;
    return 0;
}

static int j_float(const cJSON *root, const char *name, float *dst,
                   float lo, float hi, bool required)
{
    const cJSON *n = cJSON_GetObjectItemCaseSensitive(root, name);
    if (!n) {
        if (required) { fprintf(stderr, "config: missing '%s'\n", name); return -1; }
        return 0;
    }
    if (!cJSON_IsNumber(n)) {
        fprintf(stderr, "config: '%s' must be a number\n", name);
        return -1;
    }
    float v = (float)n->valuedouble;
    if (v < lo || v > hi) {
        fprintf(stderr, "config: '%s' = %g out of range [%g..%g]\n",
                name, (double)v, (double)lo, (double)hi);
        return -1;
    }
    *dst = v;
    return 0;
}

/* "source": "synth" | "adc"  -- accept string OR integer for robustness */
static int j_source(const cJSON *root, uint16_t *dst)
{
    const cJSON *n = cJSON_GetObjectItemCaseSensitive(root, "source");
    if (!n) return 0;  /* keep default */
    if (cJSON_IsString(n)) {
        if (strcmp(n->valuestring, "adc") == 0) {
            *dst = MOTOR_SOURCE_ADC;
            return 0;
        }
        if (strcmp(n->valuestring, "synth") == 0) {
            fprintf(stderr, "config: 'source: synth' is no longer supported "
                    "(removed in schema v2)\n");
            return -1;
        }
        fprintf(stderr, "config: source must be \"adc\"\n");
        return -1;
    }
    if (cJSON_IsNumber(n)) {
        int v = (int)n->valuedouble;
        if (v != MOTOR_SOURCE_ADC) {
            fprintf(stderr, "config: source int must be %u (adc)\n",
                    (unsigned)MOTOR_SOURCE_ADC);
            return -1;
        }
        *dst = (uint16_t)v;
        return 0;
    }
    fprintf(stderr, "config: 'source' has unexpected type\n");
    return -1;
}

static int j_run_state(const cJSON *root, uint16_t *dst)
{
    const cJSON *n = cJSON_GetObjectItemCaseSensitive(root, "run_state");
    if (!n) return 0;
    if (cJSON_IsString(n)) {
        if      (strcmp(n->valuestring, "run")  == 0) *dst = MOTOR_RUN_RUN;
        else if (strcmp(n->valuestring, "stop") == 0) *dst = MOTOR_RUN_STOP;
        else { fprintf(stderr, "config: run_state must be \"run\" or \"stop\"\n"); return -1; }
        return 0;
    }
    if (cJSON_IsNumber(n)) {
        int v = (int)n->valuedouble;
        if (v != 0 && v != 1) { fprintf(stderr, "config: run_state int must be 0 or 1\n"); return -1; }
        *dst = (uint16_t)v;
        return 0;
    }
    fprintf(stderr, "config: 'run_state' has unexpected type\n");
    return -1;
}

/* ============================ loader ====================================== */
int config_load_file(const char *path, full_config_t *out)
{
    if (!path || !out) { errno = EINVAL; return -1; }

    /* Slurp the whole file -- it's tiny (<8 KB realistic). */
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "config: open(%s): %s\n", path, strerror(errno));
        return -1;
    }
    struct stat st;
    if (fstat(fd, &st) != 0 || st.st_size <= 0 || st.st_size > (1 << 20)) {
        fprintf(stderr, "config: bad size for %s\n", path);
        close(fd);
        return -1;
    }
    char *buf = malloc((size_t)st.st_size + 1);
    if (!buf) { close(fd); return -1; }
    ssize_t got = 0, n;
    while (got < st.st_size && (n = read(fd, buf + got, (size_t)(st.st_size - got))) > 0)
        got += n;
    close(fd);
    if (got != st.st_size) { free(buf); return -1; }
    buf[st.st_size] = 0;

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        const char *e = cJSON_GetErrorPtr();
        fprintf(stderr, "config: parse error near '%.32s'\n", e ? e : "(?)");
        return -1;
    }

    /* Stage into a local copy, starting from defaults so missing optional
     * fields take their default values. */
    full_config_t cfg = CONFIG_DEFAULTS;

    /* ----- pi tier ----- */
    const cJSON *pi = cJSON_GetObjectItemCaseSensitive(root, "pi");
    if (pi && cJSON_IsObject(pi)) {
        if (j_int (pi, "spi_bus",         &cfg.pi.spi_bus,         0, 7,        false) < 0) goto bad;
        if (j_int (pi, "spi_dev",         &cfg.pi.spi_dev,         0, 7,        false) < 0) goto bad;
        if (j_int (pi, "spi_mode",        &cfg.pi.spi_mode,        0, 3,        false) < 0) goto bad;
        if (j_uint(pi, "spi_clock_hz",    &cfg.pi.spi_clock_hz,    100000u, 50000000u, false) < 0) goto bad;
        if (j_int (pi, "spi_cpha",        &cfg.pi.spi_cpha,        0, 1,        false) < 0) goto bad;
        if (j_int (pi, "spi_word_width",  &cfg.pi.spi_word_width,  8, 32,       false) < 0) goto bad;
        if (j_int (pi, "spi_idle_insert", &cfg.pi.spi_idle_insert, 0, 1,        false) < 0) goto bad;
        if (j_int (pi, "rt_priority",     &cfg.pi.rt_priority,     1, 63,       false) < 0) goto bad;
        if (j_int (pi, "dataready_pin",   &cfg.pi.dataready_pin,   0, 27,       false) < 0) goto bad;

        /* word_width must be 8, 16, or 32 -- the j_int range check above
         * leaves intermediate values like 13 reachable. */
        if (cfg.pi.spi_word_width != 8 && cfg.pi.spi_word_width != 16 &&
            cfg.pi.spi_word_width != 32) {
            fprintf(stderr, "config: 'spi_word_width' = %d must be 8, 16, or 32\n",
                    cfg.pi.spi_word_width);
            goto bad;
        }

        const cJSON *sc = cJSON_GetObjectItemCaseSensitive(pi, "scaling");
        if (sc && cJSON_IsObject(sc)) {
            if (j_float(sc, "current_scale",  &cfg.pi.current_scale,  -1e6f, 1e6f, false) < 0) goto bad;
            if (j_float(sc, "current_offset", &cfg.pi.current_offset, -1e6f, 1e6f, false) < 0) goto bad;
            if (j_float(sc, "vib_scale",      &cfg.pi.vib_scale,      -1e6f, 1e6f, false) < 0) goto bad;
            if (j_float(sc, "vib_offset",     &cfg.pi.vib_offset,     -1e6f, 1e6f, false) < 0) goto bad;
            if (j_float(sc, "rpm_scale",      &cfg.pi.rpm_scale,      -1e6f, 1e6f, false) < 0) goto bad;
            if (j_float(sc, "rpm_offset",     &cfg.pi.rpm_offset,     -1e6f, 1e6f, false) < 0) goto bad;
        }
    }

    /* ----- stm tier (the subset pushed via SET_CONFIG) ----- */
    const cJSON *stm = cJSON_GetObjectItemCaseSensitive(root, "stm");
    if (stm && cJSON_IsObject(stm)) {
        uint32_t br = cfg.stm.block_rows;
        if (j_uint(stm, "block_rows",      &br, 1u, MOTOR_MAX_ROWS_PER_BLOCK, false) < 0) goto bad;
        cfg.stm.block_rows = (uint16_t)br;

        if (j_source(stm,    &cfg.stm.source)    < 0) goto bad;
        if (j_run_state(stm, &cfg.stm.run_state) < 0) goto bad;

        /* sample_rate_hz: 0 means "leave alone" on the STM side. Range
         * 100 Hz .. 100 kHz keeps us inside ADC + DMA capability and out
         * of pathological territory.                                       */
        if (j_uint(stm, "sample_rate_hz", &cfg.stm.sample_rate_hz,
                   0u, 100000u, false) < 0) goto bad;
        if (cfg.stm.sample_rate_hz != 0u && cfg.stm.sample_rate_hz < 100u) {
            fprintf(stderr, "config: sample_rate_hz too low (min 100 Hz)\n");
            goto bad;
        }

        /* imu_rate_hz: 0 = leave alone. Range 10..1000 Hz (MPU6050 accel
         * native output is 1 kHz -> upper bound). Below 10 Hz is silly.    */
        if (j_uint(stm, "imu_rate_hz", &cfg.stm.imu_rate_hz,
                   0u, 1000u, false) < 0) goto bad;
        if (cfg.stm.imu_rate_hz != 0u && cfg.stm.imu_rate_hz < 10u) {
            fprintf(stderr, "config: imu_rate_hz too low (min 10 Hz)\n");
            goto bad;
        }
    }

    /* Zero the reserved bytes -- both sides require this. */
    memset(cfg.stm.reserved, 0, sizeof cfg.stm.reserved);

    cJSON_Delete(root);
    *out = cfg;
    return 0;

bad:
    cJSON_Delete(root);
    return -1;
}

/* ============================ spi.conf rewrite ============================
 * Edit /system/etc/spi.conf so the keys we care about match the loaded JSON
 * config, then bounce spi-dwc to make the new values stick. If nothing
 * differs, do nothing -- avoids gratuitous driver restarts.
 *
 * Strategy: walk the file line-by-line; for any line whose key matches one
 * of our targets, replace its value; pass everything else through unchanged.
 * Write to a tmp file, then atomic rename. Then `slay spi-dwc` and respawn.
 */
#define SPI_CONF_PATH "/system/etc/spi.conf"

/* A small (key, target value) pair so we can drive both read-diff and rewrite
 * passes with the same data. We only touch numeric integer fields.            */
typedef struct {
    const char *key;
    uint32_t    val;
} spi_kv_t;

static int read_current_kv(const char *key, uint32_t *out)
{
    FILE *f = fopen(SPI_CONF_PATH, "r");
    if (!f) return -1;
    size_t klen = strlen(key);
    char line[256];
    while (fgets(line, sizeof line, f)) {
        const char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (strncmp(p, key, klen) == 0 && p[klen] == '=') {
            unsigned long v = strtoul(p + klen + 1, NULL, 10);
            *out = (uint32_t)v;
            fclose(f);
            return 0;
        }
    }
    fclose(f);
    return -1;
}

static int rewrite_spi_conf(const spi_kv_t *kv, size_t n)
{
    FILE *in = fopen(SPI_CONF_PATH, "r");
    if (!in) {
        fprintf(stderr, "spi.conf: open for read failed: %s\n", strerror(errno));
        return -1;
    }
    char tmp_path[] = SPI_CONF_PATH ".tmp.XXXXXX";
    int tfd = mkstemp(tmp_path);
    if (tfd < 0) {
        fprintf(stderr, "spi.conf: mkstemp failed: %s\n", strerror(errno));
        fclose(in);
        return -1;
    }
    FILE *out = fdopen(tfd, "w");
    if (!out) { close(tfd); fclose(in); return -1; }

    /* Per-key "did we hit this in the file" tracking so we can report any
     * missing keys; the file template should already contain placeholders
     * for everything we care about, but defensive code is cheap.            */
    int hit[16] = {0};
    if (n > 16) n = 16;

    char line[256];
    while (fgets(line, sizeof line, in)) {
        const char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        int matched = 0;
        for (size_t i = 0; i < n; ++i) {
            size_t klen = strlen(kv[i].key);
            if (strncmp(p, kv[i].key, klen) == 0 && p[klen] == '=') {
                fprintf(out, "%s=%u\n", kv[i].key, (unsigned)kv[i].val);
                hit[i] = 1;
                matched = 1;
                break;
            }
        }
        if (!matched) fputs(line, out);
    }
    fclose(in);
    if (fclose(out) != 0) { unlink(tmp_path); return -1; }

    int missing = 0;
    for (size_t i = 0; i < n; ++i) {
        if (!hit[i]) {
            fprintf(stderr, "spi.conf: WARNING -- no '%s=' line found\n", kv[i].key);
            missing = 1;
        }
    }
    if (missing) {
        /* Don't refuse, but flag it: maybe the user has a stripped-down
         * spi.conf and our default for that field is fine.                 */
    }

    if (rename(tmp_path, SPI_CONF_PATH) != 0) {
        fprintf(stderr, "spi.conf: rename failed: %s\n", strerror(errno));
        unlink(tmp_path);
        return -1;
    }
    return 0;
}

static int bounce_spi_dwc(void)
{
    /* Best-effort: slay any running instance, give it a moment, then respawn.
     * If slay returns nonzero because nothing was running, that's fine.    */
    int rc = system("slay spi-dwc 2>/dev/null");
    (void)rc;
    struct timespec ts = { .tv_sec = 0, .tv_nsec = 200 * 1000 * 1000 };
    nanosleep(&ts, NULL);

    rc = system("spi-dwc -c " SPI_CONF_PATH " &");
    if (rc != 0) {
        fprintf(stderr, "spi.conf: respawn of spi-dwc returned %d\n", rc);
        return -1;
    }
    ts.tv_sec = 1; ts.tv_nsec = 0;
    nanosleep(&ts, NULL);
    return 0;
}

int spi_apply_conf(const pi_config_t *pi)
{
    const spi_kv_t want[] = {
        { "clock_rate",  pi->spi_clock_hz                  },
        { "cpha",        (uint32_t)pi->spi_cpha            },
        { "word_width",  (uint32_t)pi->spi_word_width      },
        { "idle_insert", (uint32_t)pi->spi_idle_insert     },
    };
    const size_t N = sizeof want / sizeof want[0];

    int any_diff = 0;
    for (size_t i = 0; i < N; ++i) {
        uint32_t cur = 0;
        if (read_current_kv(want[i].key, &cur) != 0) {
            fprintf(stderr, "[ctrl] spi.conf: cannot read '%s'; will write anyway\n",
                    want[i].key);
            any_diff = 1;
            continue;
        }
        if (cur != want[i].val) {
            fprintf(stderr, "[ctrl] spi.conf: %s: %u -> %u\n",
                    want[i].key, (unsigned)cur, (unsigned)want[i].val);
            any_diff = 1;
        }
    }
    if (!any_diff) return 0;

    if (rewrite_spi_conf(want, N) != 0) return -1;
    if (bounce_spi_dwc() != 0)         return -1;
    return 0;
}