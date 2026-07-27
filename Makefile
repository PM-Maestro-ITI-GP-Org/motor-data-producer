CC = qcc
CFLAGS = -Vgcc_ntoaarch64le -Wall -O2

# rpi_gpio.h comes from the rpi-gpio project. In the monorepo this was
# -I../rpi-gpio/resmgr/public, a path to a sibling directory that does not exist
# now each application has its own repository. The build system passes the
# include directory in instead: meta-qnx-hyp DEPENDS on rpi-gpio and points this
# at the sysroot the header is staged into.
CFLAGS += $(EXTRA_CFLAGS)
LDFLAGS = -lm

BUILD_DIR = build
MOTOR_CTRL     = $(BUILD_DIR)/motor_controller
MOTOR_CTRL_DBG = $(BUILD_DIR)/motor_controller_debug
MOTOR_MON      = $(BUILD_DIR)/motor_monitor
SPI_PROBE      = $(BUILD_DIR)/spi_probe
PD_TEST        = $(BUILD_DIR)/pd_test

.PHONY: all clean

all: $(MOTOR_CTRL) $(MOTOR_CTRL_DBG) $(MOTOR_MON) $(SPI_PROBE) $(PD_TEST)

$(BUILD_DIR):
	mkdir -p $@

$(MOTOR_CTRL): motor_controller.c rpi_spi.c rpi_gpio.c config.c cJSON.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(MOTOR_CTRL_DBG): motor_controller_debug.c rpi_spi.c rpi_gpio.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(MOTOR_MON): motor_monitor.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(SPI_PROBE): spi_probe.c rpi_spi.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(PD_TEST): pd_test.c rpi_gpio.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -rf $(BUILD_DIR)
