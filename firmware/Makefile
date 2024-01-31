.PHONY: all build cmake clean format

BUILD_DIR := build
BUILD_TYPE ?= Debug

BIN_FILE := ${BUILD_DIR}/embedded-controls.bin

all: build

${BUILD_DIR}/Makefile:
	cmake \
		-B${BUILD_DIR} \
		-DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
		-DCMAKE_TOOLCHAIN_FILE=gcc-arm-none-eabi.cmake \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake: ${BUILD_DIR}/Makefile

build: cmake
	$(MAKE) -C ${BUILD_DIR} --no-print-directory

SRCS := $(shell find . -name '*.[ch]' -or -name '*.[ch]pp')
format: $(addsuffix .format,${SRCS})

%.format: %
	clang-format -i $<

${BIN_FILE}:
	@echo "Error: Binary file not found: ${BIN_FILE}"
	@exit 1

stflash: ${BIN_FILE}
	@echo "Flashing Binary File: ${BIN_FILE}"
	st-flash --reset write $< 0x08000000

clean:
	rm -rf $(BUILD_DIR)
