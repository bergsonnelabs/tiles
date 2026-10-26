# Host-side tests of the Thread provisioning wire format and boot precedence
# (sdk/thread/thread_prov_codec.c): no hardware, no ARM toolchain. Also pins
# the worked example in docs/thread-provisioning-protocol.md.
#
#   make          # build and run

CC     ?= cc
PYTHON ?= python3
ROOT   := $(abspath ../..)
# core_thread.h includes core_caps.h, which coregen writes from the Core's
# status file in a firmware build; here the same generator writes the W5's.
STATUS := $(ROOT)/sdk/status/core-w.json
CFLAGS := -std=c11 -O2 -g -Wall -Wextra -Werror -Wshadow -Wconversion \
          -DSTM32WBA55xx -Ibuild -I$(ROOT)/sdk/core -I$(ROOT)/sdk/port/stm32/core -I$(ROOT)/sdk/thread/include
SRCS   := prov_test.c $(ROOT)/sdk/thread/thread_prov_codec.c
DEPS   := $(SRCS) $(ROOT)/sdk/thread/include/thread_prov.h $(ROOT)/sdk/core/core_thread.h build/core_caps.h

.PHONY: all clean
all: prov_test
	./prov_test

build/core_caps.h: $(STATUS) $(ROOT)/sdk/status/features.json $(ROOT)/tools/coregen/core_caps.py
	$(PYTHON) $(ROOT)/tools/coregen/core_caps.py $(STATUS) -o $@

prov_test: $(DEPS)
	$(CC) $(CFLAGS) $(SRCS) -o $@

clean:
	rm -rf prov_test *.dSYM build
