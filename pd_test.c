/*
 * pd_test.c -- data-ready (GPIO17) level monitor, via the rpi_gpio server
 * ----------------------------------------------------------------------------
 * Standalone diagnostic for the data-ready handshake. It configures the pin as
 * an input with an internal pull-DOWN and then polls the LIVE level through the
 * rpi_gpio resource manager (RPI_GPIO_READ), printing every transition. Start
 * the STM32 and you should see the line toggle as frames are offered; with the
 * STM32 idle/disconnected and the pull-down active, it should stay low.
 *
 * This replaces an earlier version that poked RP1 registers directly at
 * hard-coded physical addresses (0x400d0000 / 0x400e0000) to read a "live pad"
 * level. Those addresses and the pull bit-encoding were wrong for the Pi 5 /
 * RP1 (io_bank0 is at 0x400d0000, the pad/pull block is elsewhere, and RP1 uses
 * separate PDE/PUE pull-enable bits, not a 2-bit field), so it never measured
 * anything real. The supported way to read a pin's level on this platform is
 * through the server, which is what the controller uses -- so this tool now
 * exercises exactly the same path.
 *
 * Build (QNX):
 *   qcc -Vgcc_ntoaarch64le -std=gnu11 -O2 pd_test.c rpi_gpio.c -o pd_test
 *   ./pd_test                 (root not required just to read a level)
 * ----------------------------------------------------------------------------
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

#include "rpi_gpio.h"

#define GPIO_PIN   17
#define POLL_MS    20

static volatile sig_atomic_t g_running = 1;
static void on_signal(int s) { (void)s; g_running = 0; }

static void ms_sleep(int ms)
{
    struct timespec ts = { .tv_sec = ms / 1000,
                           .tv_nsec = (long)(ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

int main(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    if (rpi_gpio_setup(GPIO_PIN, GPIO_IN) != GPIO_SUCCESS) {
        fprintf(stderr, "pd_test: rpi_gpio_setup(pin=%d) failed -- "
                        "is the rpi_gpio server running?\n", GPIO_PIN);
        return 1;
    }
    if (rpi_gpio_setup_pull(GPIO_PIN, GPIO_IN, GPIO_PUD_DOWN) != GPIO_SUCCESS) {
        fprintf(stderr, "pd_test: warning -- could not set pull-down on GPIO%d; "
                        "fit an external 10k pull-down\n", GPIO_PIN);
    }

    printf("[pd_test] monitoring GPIO%d level every %d ms (Ctrl-C to stop)\n",
           GPIO_PIN, POLL_MS);
    printf("[pd_test] idle should read LOW; start motor firmware to see toggles\n\n");

    int      have_prev = 0;
    unsigned prev      = GPIO_LOW;
    uint64_t highs = 0, lows = 0, iter = 0;

    while (g_running) {
        unsigned level = GPIO_LOW;
        if (rpi_gpio_input(GPIO_PIN, &level) != GPIO_SUCCESS) {
            fprintf(stderr, "pd_test: rpi_gpio_input(pin=%d) failed\n", GPIO_PIN);
            ms_sleep(POLL_MS);
            continue;
        }

        if (level == GPIO_HIGH) highs++; else lows++;

        if (!have_prev || level != prev) {
            printf("[pd_test] iter=%-8" PRIu64 " (~%" PRIu64 " ms)  GPIO%d = %s\n",
                   iter, iter * (uint64_t)POLL_MS, GPIO_PIN,
                   (level == GPIO_HIGH) ? "HIGH" : "LOW");
            fflush(stdout);
            have_prev = 1;
            prev = level;
        }

        iter++;
        ms_sleep(POLL_MS);
    }

    rpi_gpio_cleanup();
    printf("\n[pd_test] stopped: samples=%" PRIu64 " high=%" PRIu64
           " low=%" PRIu64 "\n", iter, highs, lows);
    return 0;
}
