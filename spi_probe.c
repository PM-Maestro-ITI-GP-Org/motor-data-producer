/*
 * spi_probe.c -- Pi-side SPI sanity test. NO data-ready, NO frames, NO CRC.
 * ----------------------------------------------------------------------------
 * Clocks 16 bytes over SPI0 (dev0) 50 times and prints what the STM returns.
 * If the link works you'll see the slave's pattern:  01 02 03 ... 10
 * Build (QNX):
 *   qcc -Vgcc_ntoaarch64le -std=gnu11 -O2 spi_probe.c rpi_spi.c -o spi_probe
 * ----------------------------------------------------------------------------
 */
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include "rpi_spi.h"

int main(void)
{
    if (rpi_spi_configure_device(0, 0, 0, 1000000u) != SPI_SUCCESS) {
        fprintf(stderr, "spi_probe: configure failed\n");
        return 1;
    }

    uint8_t tx[16], rx[16];
    for (int i = 0; i < 16; ++i) tx[i] = (uint8_t)(0xA0 + i);   /* A0..AF (what STM should receive) */

    for (int n = 0; n < 50; ++n) {
        if (rpi_spi_write_read_data(0, 0, tx, rx, sizeof tx) != SPI_SUCCESS) {
            fprintf(stderr, "spi_probe: transfer failed\n");
        } else {
            printf("rx:");
            for (int i = 0; i < 16; ++i) printf(" %02x", rx[i]);
            printf("\n");
            fflush(stdout);
        }
        struct timespec d = { 0, 200 * 1000 * 1000L };   /* 5 Hz */
        nanosleep(&d, NULL);
    }

    rpi_spi_cleanup_device(0, 0);
    return 0;
}