# Host-native tests for the serial-update protocol core (no hardware, no ARM
# toolchain). Builds su_sim from the same su_core.c the flasher runs, then
# drives it through tools/test_serial_update.py.

CC     ?= cc
PYTHON ?= python3
ROOT   := $(abspath ../../..)

.PHONY: all clean

all: su_sim
	$(PYTHON) $(ROOT)/tools/test_serial_update.py --sim ./su_sim

su_sim: su_sim.c ../su_core.c ../su_core.h ../su_protocol.h
	$(CC) -std=c11 -D_POSIX_C_SOURCE=200809L -Wall -Wextra -Werror -O1 -g -I.. su_sim.c ../su_core.c -o $@

clean:
	rm -f su_sim
