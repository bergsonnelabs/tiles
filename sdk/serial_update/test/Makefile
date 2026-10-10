# Host-native tests for the serial-update protocol core (no hardware, no ARM
# toolchain). Builds su_sim from the same su_core.c the flashers run, once with
# the L4's protocol settings and once with the H5 flasher's (-D flags as in
# ../Makefile), then drives both through tools/test_serial_update.py.
# ota_sim: the Core.ST.W5 BLE update (su_core.c in staging mode, su_cmac.c,
# su_ota.c) on a simulated WBA55 flash with power cuts; see ota_sim.c.

CC     ?= cc
PYTHON ?= python3
ROOT   := $(abspath ../../..)
SIMFLAGS = -std=c11 -D_POSIX_C_SOURCE=200809L -Wall -Wextra -Werror -O1 -g -I..
H5FLAGS  = -DSIM_H5 -DSU_MAX_PAYLOAD=8192u -DSU_PROG_UNIT=16u -DSU_PROTOCOL_VERSION=2u

.PHONY: all clean

all: su_sim su_sim_h5 ota_sim
	$(PYTHON) $(ROOT)/tools/test_serial_update.py --sim ./su_sim
	$(PYTHON) $(ROOT)/tools/test_serial_update.py --sim ./su_sim_h5 --chip h5
	./ota_sim

su_sim: su_sim.c ../su_core.c ../su_core.h ../su_protocol.h
	$(CC) $(SIMFLAGS) su_sim.c ../su_core.c -o $@

su_sim_h5: su_sim.c ../su_core.c ../su_core.h ../su_protocol.h
	$(CC) $(SIMFLAGS) $(H5FLAGS) su_sim.c ../su_core.c -o $@

ota_sim: ota_sim.c ../su_core.c ../su_core.h ../su_protocol.h ../su_cmac.c ../su_cmac.h ../su_ota.c ../su_ota.h
	$(CC) $(SIMFLAGS) ota_sim.c -o $@

clean:
	rm -rf su_sim su_sim_h5 su_sim.dSYM su_sim_h5.dSYM ota_sim ota_sim.dSYM
