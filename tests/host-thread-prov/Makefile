# Host-side tests of the Thread provisioning wire format and boot precedence
# (sdk/thread/thread_prov_codec.c): no hardware, no ARM toolchain. Also pins
# the worked example in docs/thread-provisioning-protocol.md.
#
#   make          # build and run

CC     ?= cc
ROOT   := $(abspath ../..)
CFLAGS := -std=c11 -O2 -g -Wall -Wextra -Werror -Wshadow -Wconversion \
          -DSTM32WBA55xx -I$(ROOT)/sdk/core -I$(ROOT)/sdk/thread/include
SRCS   := prov_test.c $(ROOT)/sdk/thread/thread_prov_codec.c
DEPS   := $(SRCS) $(ROOT)/sdk/thread/include/thread_prov.h $(ROOT)/sdk/core/core_thread.h

.PHONY: all clean
all: prov_test
	./prov_test

prov_test: $(DEPS)
	$(CC) $(CFLAGS) $(SRCS) -o $@

clean:
	rm -rf prov_test *.dSYM
