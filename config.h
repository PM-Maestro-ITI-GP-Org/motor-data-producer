/*
 * config.h
 * ----------------------------------------------------------------------------
 * Pi-side runtime configuration: loaded from a JSON file on disk and split
 * into two tiers --
 *
 *   pi:  applied locally by the controller (SPI bus IDs, RT priority,
 *        data-ready pin, scaling constants). Never sent over the wire.
 *
 *   stm: pushed to the STM32 via SET_CONFIG and acknowledged before it
 *        is considered active here. Layout is config_payload_t from
 *        motor_wire.h -- compiled into both binaries.
 *
 * The loader rejects the WHOLE file on any single bad field; we never
 * half-apply.
 * ----------------------------------------------------------------------------
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "motor_wire.h"

/* ---- Pi-only tier --------------------------------------------------------
 * Mirrors the existing controller_config_t in motor_controller.c, plus the
 * downstream scaling constants that motor_wire.h promises live on the Pi.
 *
 * NOTE on the SPI fields (clock_hz, cpha, word_width, idle_insert): QNX's
 * spi-dwc only honors these from its config file (/system/etc/spi.conf),
 * not the runtime DCMD_SPI_SET_CONFIG devctl. To keep the JSON authoritative,
 * the controller edits spi.conf to match on startup and bounces spi-dwc to
 * make the changes stick. See spi_apply_conf() below.                        */
typedef struct {
    int      spi_bus;
    int      spi_dev;
    int      spi_mode;
    uint32_t spi_clock_hz;
    int      spi_cpha;         /* 0 or 1; STM expects 0 (mode 0)              */
    int      spi_word_width;   /* 8, 16, or 32; STM expects 8                 */
    int      spi_idle_insert;  /* 0 or 1                                       */
    int      rt_priority;
    int      dataready_pin;

    /* scaling: engineering units = raw * scale + offset. Applied downstream
     * (Qt / SOME/IP publisher). The controller just stores them in shm so
     * the consumers can pick them up.                                       */
    float    current_scale;  float current_offset;
    float    vib_scale;      float vib_offset;
    float    rpm_scale;      float rpm_offset;
} pi_config_t;

/* ---- the full config the controller carries at runtime ------------------ */
typedef struct {
    pi_config_t       pi;
    config_payload_t  stm;     /* mirrored to the STM32                     */
} full_config_t;

/* ---- defaults (used if no config file is present at startup) ------------ */
extern const full_config_t CONFIG_DEFAULTS;

/* ---- API -----------------------------------------------------------------
 * config_load_file:   parse `path`, validate every field, fill *out.
 *                     Returns 0 on success, -1 on any parse / range error
 *                     (and writes a diagnostic to stderr). On failure *out
 *                     is left untouched -- callers can keep the previous
 *                     known-good config.
 *
 * config_stm_differs: true if the STM-tier subset of `a` differs from `b`.
 *                     Used to decide whether a SIGHUP reload needs to push
 *                     a new SET_CONFIG (vs. only Pi-local changes).
 *
 * config_spi_wire_differs: true if any SPI-side field that requires reopening
 *                     the bus differs -- spi_bus/dev/mode/clock_hz/cpha/
 *                     word_width/idle_insert. Excludes dataready_pin: the GPIO
 *                     edge subscription lives on a *separate* resource manager
 *                     (rpi_gpio) and is not disturbed by bouncing spi-dwc, so a
 *                     wire-only change must NOT tear down GPIO. Used to gate the
 *                     close_spi -> spi_apply_conf -> open_spi path.
 *
 * config_pi_hw_differs: true if any Pi field that requires re-initialising the
 *                     SPI bus OR moving the data-ready GPIO subscription differs
 *                     -- i.e. config_spi_wire_differs() OR dataready_pin.
 *                     rt_priority and the scaling constants are deliberately
 *                     excluded: those apply live (see pi_apply_live) without
 *                     bouncing anything. Convenience gate for "does this reload
 *                     need any hardware reinit at all".
 *
 * config_install_sighup: install the SIGHUP handler that sets the global
 *                     reload flag. Call once from main.
 *
 * config_reload_requested: returns true (and clears the flag) when SIGHUP
 *                     has fired since the last call. Polled by the main
 *                     loop between data-ready waits.
 */
int  config_load_file(const char *path, full_config_t *out);
bool config_stm_differs(const config_payload_t *a, const config_payload_t *b);
bool config_spi_wire_differs(const pi_config_t *a, const pi_config_t *b);
bool config_pi_hw_differs(const pi_config_t *a, const pi_config_t *b);
void config_install_sighup(void);
bool config_reload_requested(void);

/* spi_apply_conf: bring /system/etc/spi.conf into agreement with the SPI
 * fields in `pi` (clock_rate, cpha, word_width, idle_insert) and bounce
 * spi-dwc if anything actually changed. Idempotent: a no-op if the file
 * already matches. Returns 0 on success, -1 on any error.
 *
 * The caller MUST close any open SPI file descriptors before calling this,
 * and reopen them after, because the spi-dwc restart invalidates them.     */
int  spi_apply_conf(const pi_config_t *pi);

#endif /* CONFIG_H */