QT6_DIR := ../../QT/qt6-qnx-libs/output_dir
QNX800_DIR := ../../../qnx800

.PHONY: all qnx clean

all: qnx

qnx:
	@[ -n "$(QNX_HOST)" ] || [ -f "$(QNX800_DIR)/qnxsdp-env.sh" ] || { echo "QNX SDP not found at $(QNX800_DIR)/qnxsdp-env.sh"; exit 1; }
	@bash -c 'set -e; \
		[ -n "$$QNX_HOST" ] || . "$(QNX800_DIR)/qnxsdp-env.sh"; \
		cmake -S . -B build_qnx \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_TOOLCHAIN_FILE=$(QT6_DIR)/lib/cmake/Qt6/qt.toolchain.cmake \
			-DQNX_LIB_DIR=$(QNX800_DIR)/target/qnx/aarch64le; \
		cmake --build build_qnx'

clean:
	rm -rf build_qnx
