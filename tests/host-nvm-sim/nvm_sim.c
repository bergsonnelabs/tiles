/**
 * nvm_sim.c — core_nvm's flash emulation on the host, with power cuts
 *
 * Builds sdk/core/core_nvm.c itself (L4 or W5 geometry) against a simulated
 * two-page flash, runs random workloads of writes and erase_all calls, and
 * checks, after every call, that core_nvm_read returns what was written.
 *
 * Then, for every flash operation in the workload (each unit program and each
 * page erase), it runs the workload again and cuts the power there: the
 * operation is left torn (a program clears only some of its bits, an erase
 * sets only some), and the "Core" reboots (core_nvm's RAM state is dropped).
 * After the reboot the bytes the interrupted call covered must read all-old
 * or all-new, everything else unchanged, and the rest of the workload must
 * then run correctly. Half the runs cut a second time, later on, which lands
 * some cuts inside the recovery (the compaction a torn append forces).
 *
 * Not modelled: ECC. On the chip a torn unit can also read with an
 * uncorrectable ECC error; core_nvm's NMI handler turns that into "invalid",
 * the same outcome as the corrupt CRC this simulation produces.
 *
 *   make            # both geometries
 */

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t sim_flash[2 * 8192];
#define NVM_MEM(addr)     ((uintptr_t)(sim_flash + ((addr) - CORE_NVM_FLASH_ADDR)))
#define NVM_LAYOUT_OK()   1
#include "core_nvm.c"

/* ---- Simulated flash ---- */

static long     sim_ops, sim_cut_at = -1, sim_cut2_at = -1;
static jmp_buf  sim_cut;
static uint64_t rng = 88172645463325252ULL;

static uint32_t rnd(void)
{
    rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
    return (uint32_t)rng;
}

static int cut_now(void)
{
    long n = sim_ops++;
    return n == sim_cut_at || n == sim_cut2_at;
}

int ll_flash_erase_page(uint32_t page)
{
    if (page < CORE_NVM_FIRST_PAGE || page > CORE_NVM_FIRST_PAGE + 1u) {
        fprintf(stderr, "erase of page %u, outside the NVM pages\n", page);
        abort();
    }
    uint8_t *p = sim_flash + (page - CORE_NVM_FIRST_PAGE) * CORE_NVM_PAGE_SIZE;
    if (cut_now()) {
        for (uint32_t k = 0; k < CORE_NVM_PAGE_SIZE; k++) p[k] |= (uint8_t)rnd();
        longjmp(sim_cut, 1);
    }
    memset(p, 0xFF, CORE_NVM_PAGE_SIZE);
    return 0;
}

static int program(uint32_t addr, const uint8_t *d, uint32_t n)
{
    if (addr < CORE_NVM_FLASH_ADDR || addr + n > CORE_NVM_FLASH_ADDR + CORE_NVM_FLASH_SIZE || addr % n) {
        fprintf(stderr, "program at 0x%08x (%u B), outside the NVM pages or misaligned\n", addr, n);
        abort();
    }
    uint8_t *p = sim_flash + (addr - CORE_NVM_FLASH_ADDR);
    for (uint32_t k = 0; k < n; k++)
        if (p[k] != 0xFF) return -1;            /* PROGERR: not erased */
    if (cut_now()) {
        for (uint32_t k = 0; k < n; k++) p[k] &= (uint8_t)(d[k] | rnd());
        longjmp(sim_cut, 1);
    }
    memcpy(p, d, n);
    return 0;
}

int ll_flash_program_dword(uint32_t addr, uint32_t w0, uint32_t w1)
{
    uint32_t w[2] = { w0, w1 };
    return program(addr, (const uint8_t *)w, 8);
}

int ll_flash_program_qword(uint32_t addr, const uint32_t w[4])
{
    return program(addr, (const uint8_t *)w, 16);
}

/* A reset: core_nvm keeps nothing but the flash. */
static void reboot(void)
{
    memset(&s, 0, sizeof(s));
    s_busy = 0;
    s_ecc_hit = 0;
}

/* ---- Workload ---- */

typedef struct { uint16_t off, len; uint32_t seed; uint8_t erase_all; } op_t;
#define NOPS 90
static op_t    ops[NOPS];
static uint8_t model[CORE_NVM_SIZE], before[CORE_NVM_SIZE];
static long    failures, checks;

static uint8_t data_at(const op_t *o, uint32_t i) { return (uint8_t)((o->seed >> (i % 24)) + i * 29u); }

static void make_workload(uint32_t seed)
{
    rng = 0x9E3779B97F4A7C15ULL ^ seed;
    for (int j = 0; j < NOPS; j++) {
        op_t *o = &ops[j];
        uint32_t r = rnd() % 100u;
        o->erase_all = (r == 0);
        o->len = (uint16_t)(r < 60 ? 1u + rnd() % 16u : r < 90 ? 17u + rnd() % 200u : 217u + rnd() % 808u);
        o->off = (uint16_t)(rnd() % (CORE_NVM_SIZE - o->len + 1u));
        o->seed = rnd();
        if (r >= 95) o->off = 0, o->len = CORE_NVM_SIZE;   /* the whole store */
    }
}

static void apply(const op_t *o, uint8_t *img)
{
    if (o->erase_all) { memset(img, 0xFF, CORE_NVM_SIZE); return; }
    for (uint32_t i = 0; i < o->len; i++) img[o->off + i] = data_at(o, i);
}

static int run_op(const op_t *o)
{
    if (o->erase_all) return core_nvm_erase_all();
    uint8_t buf[CORE_NVM_SIZE];
    for (uint32_t i = 0; i < o->len; i++) buf[i] = data_at(o, i);
    return core_nvm_write(o->off, buf, o->len);
}

static int read_all(uint8_t *img)
{
    return core_nvm_read(0, img, CORE_NVM_SIZE);
}

static void fail(const char *what, uint32_t seed, long cut, int j)
{
    if (failures++ < 10)
        printf("  FAIL  %s (workload %u, cut at op %ld, call %d)\n", what, seed, cut, j);
}

/* Run ops[from..] with cuts armed; after a cut, check atomicity, reboot and
 * carry on. Returns the flash operation count. */
static long run(uint32_t seed, long cut1, long cut2)
{
    static volatile int j;          /* survives longjmp */
    uint8_t img[CORE_NVM_SIZE];
    sim_ops = 0;
    sim_cut_at = cut1;
    sim_cut2_at = cut2;
    memset(model, 0xFF, sizeof(model));
    memset(sim_flash, 0xFF, sizeof(sim_flash));
    reboot();

    for (j = 0; j < NOPS; j++) {
        memcpy(before, model, sizeof(model));
        apply(&ops[j], model);
        if (setjmp(sim_cut)) {
            reboot();
            checks++;
            if (read_all(img) != 0) { fail("read after the cut", seed, cut1, j); return sim_ops; }
            if (memcmp(img, model, sizeof(img)) == 0) continue;            /* new */
            if (memcmp(img, before, sizeof(img)) == 0) {                   /* old */
                memcpy(model, before, sizeof(model));
                continue;
            }
            fail("neither old nor new after the cut", seed, cut1, j);
            return sim_ops;
        }
        int rc = run_op(&ops[j]);
        if (rc != 0) { fail("call failed", seed, cut1, j); return sim_ops; }
        checks++;
        if (read_all(img) != 0 || memcmp(img, model, sizeof(img)) != 0) {
            fail("read-back differs", seed, cut1, j);
            return sim_ops;
        }
    }
    /* And a fresh boot sees the same. */
    reboot();
    checks++;
    if (read_all(img) != 0 || memcmp(img, model, sizeof(img)) != 0) fail("after a clean reboot", seed, cut1, NOPS);
    return sim_ops;
}

int main(void)
{
    long runs = 0;
    uint32_t workloads = 12;
    printf("core_nvm on a simulated %s: %u B pages, %u B units\n",
           CORE_NVM_PAGE_SIZE == 2048u ? "Core.ST.L4 (STM32L422)" : "Core.ST.W5 (STM32WBA55)",
           CORE_NVM_PAGE_SIZE, CORE_NVM_PROG_UNIT);
    for (uint32_t w = 1; w <= workloads; w++) {
        make_workload(w);
        long total = run(w, -1, -1);
        runs++;
        for (long c = 0; c < total; c++) {
            make_workload(w);                   /* resets the RNG too */
            rng ^= (uint64_t)c * 0x2545F4914F6CDD1DULL;
            long c2 = (rnd() & 1u) ? c + 1 + (long)(rnd() % 400u) : -1;
            run(w, c, c2);
            runs++;
        }
        printf("  workload %2u: %4ld flash operations, a cut at each\n", w, total);
    }
    printf("%ld runs, %ld checks: %s\n", runs, checks, failures ? "FAIL" : "all passed");
    return failures ? 1 : 0;
}
