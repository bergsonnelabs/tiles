/**
 * su_sim.c — the serial-update protocol core on a simulated STM32L422 flash.
 *
 * Runs su_core.c natively: frames arrive on stdin (in 64-byte "USB packets"),
 * replies go to stdout, and the simulated flash is written to a file whenever
 * the run ends — at a reboot (exit 0) or when stdin closes, i.e. the host went
 * away or the power was cut (exit 3). tools/test_serial_update.py drives it.
 *
 *   su_sim <flash-in> <flash-out> [--fail-program-at N] [--idle-ms MS]
 *
 * <flash-in> is the 128 KB flash before the update (the "old app").
 * --fail-program-at N  the Nth program() call reports an error
 * --idle-ms MS         at EOF, advance the idle timer by MS before stopping
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "su_core.h"

#define SIM_FLASH_BASE  0x08000000u
#define SIM_FLASH_SIZE  (128u * 1024u)
#define SIM_PAGE        2048u

static uint8_t     flash[SIM_FLASH_SIZE];
static const char *out_path;
static long        fail_program_at = -1;
static long        program_calls;

static void dump_and_exit(int code)
{
    FILE *f = fopen(out_path, "wb");
    if (!f || fwrite(flash, 1, sizeof(flash), f) != sizeof(flash)) {
        perror(out_path);
        exit(2);
    }
    fclose(f);
    fflush(stdout);
    exit(code);
}

static int sim_erase(uint32_t page)
{
    if (page >= SIM_FLASH_SIZE / SIM_PAGE) return -1;
    memset(flash + page * SIM_PAGE, 0xFF, SIM_PAGE);
    return 0;
}

static int sim_program(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (++program_calls == fail_program_at) return -1;
    if (addr < SIM_FLASH_BASE || addr + len > SIM_FLASH_BASE + SIM_FLASH_SIZE) return -1;
    if ((addr & 7u) || (len & 7u)) return -1;              /* L4 double-word unit */
    uint32_t off = addr - SIM_FLASH_BASE;
    for (uint32_t i = 0; i < len; i += 8) {
        /* Like the L4: programming a double-word that isn't erased is PROGERR. */
        for (uint32_t k = 0; k < 8; k++)
            if (flash[off + i + k] != 0xFF) return -1;
        memcpy(flash + off + i, buf + i, 8);
    }
    return 0;
}

static void sim_send(const char *line, uint32_t len)
{
    fwrite(line, 1, len, stdout);
    fflush(stdout);
}

static void sim_reboot(void)
{
    dump_and_exit(0);
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: su_sim <flash-in> <flash-out> [--fail-program-at N] [--idle-ms MS]\n");
        return 2;
    }
    long idle_ms = 0;
    out_path = argv[2];
    for (int i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "--fail-program-at") && i + 1 < argc) fail_program_at = atol(argv[++i]);
        else if (!strcmp(argv[i], "--idle-ms") && i + 1 < argc) idle_ms = atol(argv[++i]);
    }

    FILE *f = fopen(argv[1], "rb");
    if (!f || fread(flash, 1, sizeof(flash), f) != sizeof(flash)) {
        perror(argv[1]);
        return 2;
    }
    fclose(f);

    static const su_ops_t ops = {
        .flash_base = SIM_FLASH_BASE,
        .flash_size = SIM_FLASH_SIZE,
        .page_size  = SIM_PAGE,
        .dev_id     = 0x464,
        .sram_start = 0x20000000u,
        .sram_end   = 0x2000A000u,
        .flash_read = flash,
        .erase_page = sim_erase,
        .program    = sim_program,
        .send       = sim_send,
        .reboot     = sim_reboot,
    };
    su_core_init(&ops);

    uint8_t pkt[64];
    for (;;) {
        ssize_t n = read(0, pkt, sizeof(pkt));
        if (n <= 0) break;
        /* The flasher only takes a packet when it has room for all of it. */
        if (su_core_rx_space() < (uint32_t)n) {
            fprintf(stderr, "su_sim: receive buffer overrun\n");
            return 4;
        }
        su_core_rx(pkt, (uint32_t)n);
    }

    su_core_tick((uint32_t)idle_ms);
    dump_and_exit(3);
}
