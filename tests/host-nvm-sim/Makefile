# Host-side power-cut simulation of core_nvm's flash emulation (no hardware,
# no ARM toolchain). Builds sdk/core/core_nvm.c for the L4, the W5 and the H5
# (its 16-bit data area) against a simulated flash; see nvm_sim.c.
#
#   make          # build and run all three

CC     ?= cc
ROOT   := $(abspath ../..)
CFLAGS := -std=c11 -O2 -g -Wall -Wextra -Werror -Wno-unused-function \
          -Istub -I$(ROOT)/sdk/core

.PHONY: all clean
all: nvm_sim_l4 nvm_sim_w5 nvm_sim_h5
	./nvm_sim_l4
	./nvm_sim_w5
	./nvm_sim_h5

nvm_sim_l4: nvm_sim.c $(ROOT)/sdk/core/core_nvm.c $(ROOT)/sdk/core/core_nvm.h $(wildcard stub/*.h)
	$(CC) $(CFLAGS) -DSTM32L422xx nvm_sim.c -o $@

nvm_sim_w5: nvm_sim.c $(ROOT)/sdk/core/core_nvm.c $(ROOT)/sdk/core/core_nvm.h $(wildcard stub/*.h)
	$(CC) $(CFLAGS) -DSTM32WBA55xx nvm_sim.c -o $@

nvm_sim_h5: nvm_sim.c $(ROOT)/sdk/core/core_nvm.c $(ROOT)/sdk/core/core_nvm.h $(wildcard stub/*.h)
	$(CC) $(CFLAGS) -DSTM32H523xx nvm_sim.c -o $@

clean:
	rm -rf nvm_sim_l4 nvm_sim_w5 nvm_sim_h5 *.dSYM
