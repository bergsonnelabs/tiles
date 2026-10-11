# Host-side simulation of the timer server (sdk/wpan/stm32_timer.c) on its
# LPTIM2 + SysTick interface (sdk/wpan/stm32_timer_if.c): no hardware, no ARM
# toolchain. One build per Core.ST.W5 clock level; see timer_sim.c.
#
#   make          # build and run all four

CC     ?= cc
ROOT   := $(abspath ../..)
CFLAGS := -std=c11 -O2 -g -Wall -Wextra -Werror -Wno-unused-function \
          -Istub -I$(ROOT)/sdk/wpan -I$(ROOT)/sdk/wpan/include
SRCS   := timer_sim.c $(ROOT)/sdk/wpan/stm32_timer.c $(ROOT)/sdk/wpan/stm32_timer_if.c \
          $(ROOT)/sdk/wpan/include/stm32_timer.h $(wildcard stub/*.h)
MHZ    := 16 32 64 100

.PHONY: all clean
all: $(addprefix timer_sim_,$(MHZ))
	@for m in $(MHZ); do ./timer_sim_$$m || exit 1; done

timer_sim_%: $(SRCS)
	$(CC) $(CFLAGS) -DSIM_MHZ=$*UL timer_sim.c -o $@

clean:
	rm -rf $(addprefix timer_sim_,$(MHZ)) *.dSYM
