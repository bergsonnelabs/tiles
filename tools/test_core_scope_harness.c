/* Host harness for tools/test_core_scope.py — compiles sdk/core/core_scope.c
 * with the host cc against a scripted link and a fake clock, and writes
 * everything the "device" sent to stdout as records the test can check:
 *
 *   'P' u16 len bytes   one link write (a USB queue entry / a BLE packet)
 *   'T' u16 len bytes   print text that shared the port
 *   'S' u32 dropped     core_scope_dropped() at the end
 */
#include <stdio.h>
#include <string.h>

#include "core_scope.h"

static uint32_t g_ms;
uint32_t core_millis(void) { return g_ms; }

static int      g_ready = 1;
static int      g_refuse;      /* refuse this many writes */
static uint16_t g_take_max;    /* 0 = take everything offered */

static void rec(char tag, const void *p, uint16_t n)
{
    fputc(tag, stdout);
    fwrite(&n, 2, 1, stdout);
    fwrite(p, 1, n, stdout);
}

static int link_ready(void) { return g_ready; }
static int link_write(const uint8_t *buf, uint16_t len)
{
    if (g_refuse) { g_refuse--; return 0; }
    if (g_take_max && len > g_take_max) len = g_take_max;
    rec('P', buf, len);
    return len;
}
static void text(const char *s) { rec('T', s, (uint16_t)strlen(s)); }

static core_scope_link_t g_link = { 0, link_ready, link_write, 512, 0 };

static int32_t  distance_mm;
static float    temp_c;
static bool     pressed;
static uint32_t big;
static int16_t  accel[3];
static uint8_t  level;
static volatile int16_t isr_value;

static void finish(void)
{
    uint32_t d = core_scope_dropped();
    fputc('S', stdout);
    fwrite(&d, 4, 1, stdout);
}

/* The three types the generated Studio scope had, one sample at a known
 * time: the test compares these bytes with frames it builds itself. */
static void sc_legacy(void)
{
    core_scope_set_link(&g_link);
    core_scope_watch(distance_mm);
    core_scope_watch(temp_c);
    core_scope_watch(pressed);
    g_ms = 1000;
    core_scope_update();            /* host noticed; schema goes out */
    distance_mm = -1234; temp_c = 21.5f; pressed = true;
    g_ms = 1005;
    core_scope_update();
}

static void sc_basic(void)
{
    core_scope_set_link(&g_link);
    core_scope_watch(distance_mm);
    core_scope_watch(temp_c);
    core_scope_watch(pressed);
    core_scope_watch(big);
    core_scope_watch_array(accel, 3);
    core_scope_watch_as("lvl", level);
    core_scope_watch(isr_value);
    core_scope_update();
    for (int i = 0; i < 50; i++) {
        g_ms += 2;
        distance_mm = i - 25; temp_c = (float)i * 0.5f; pressed = (i & 1) != 0;
        big = 4000000000u + (uint32_t)i;
        accel[0] = (int16_t)(-i); accel[1] = (int16_t)(i * 100); accel[2] = -32768;
        level = (uint8_t)(200 + i); isr_value = (int16_t)(i * -3);
        core_scope_update();
        if (i % 10 == 0) text("hello \xA5 world\n"); /* a stray magic byte in text */
    }
}

static void sc_interval(void)
{
    core_scope_set_link(&g_link);
    core_scope_watch(distance_mm);
    core_scope_set_interval_ms(10);
    core_scope_update();
    for (int i = 0; i < 100; i++) { g_ms += 1; distance_mm = i; core_scope_update(); }
}

/* Declared-rate sampling from an "ISR", pumped less often than sampled, with
 * a stretch where the main loop stalls and the ring overflows. */
static void sc_rows(void)
{
    core_scope_set_link(&g_link);
    core_scope_watch(isr_value);
    core_scope_watch_array(accel, 3);
    core_scope_declare_rate_hz(1000);
    core_scope_pump();
    for (int i = 0; i < 400; i++) {
        g_ms += 1;
        isr_value = (int16_t)i; accel[0] = (int16_t)(i * 2); accel[1] = (int16_t)(-i); accel[2] = 7;
        core_scope_sample();
        int stalled = (i >= 100 && i < 250);
        if (!stalled && i % 5 == 0) core_scope_pump();
    }
    core_scope_pump();
    core_scope_pump();
}

/* A BLE-shaped link: 180-byte packets, batching, a radio that refuses. The
 * long names push the schema past one packet, the only frame ever split. */
static int32_t a_rather_long_channel_name_number_one, a_rather_long_channel_name_number_two,
    a_rather_long_channel_name_number_three, a_rather_long_channel_name_number_four,
    a_rather_long_channel_name_number_five;
static void sc_ble(void)
{
    g_link.packet_bytes = 180;
    g_link.flush_ms = 30;
    core_scope_set_link(&g_link);
    core_scope_watch(a_rather_long_channel_name_number_one);
    core_scope_watch(a_rather_long_channel_name_number_two);
    core_scope_watch(a_rather_long_channel_name_number_three);
    core_scope_watch(a_rather_long_channel_name_number_four);
    core_scope_watch(a_rather_long_channel_name_number_five);
    core_scope_set_interval_ms(10);
    core_scope_update();
    for (int i = 0; i < 600; i++) {
        g_ms += 1;
        a_rather_long_channel_name_number_one = i;
        a_rather_long_channel_name_number_five = -i;
        if (i == 200) g_refuse = 350;  /* the stack's queue stays full long enough to overflow the ring */
        core_scope_update();
    }
    g_ms += 40;
    core_scope_update();
}

/* Host goes away and comes back: nothing is sent while it is gone, and the
 * channel list leads when it returns. Also: pause / resume. */
static void sc_reconnect(void)
{
    core_scope_set_link(&g_link);
    core_scope_watch(distance_mm);
    core_scope_update();
    for (int i = 0; i < 5; i++) { g_ms += 1; distance_mm = i; core_scope_update(); }
    g_ready = 0;
    for (int i = 0; i < 50; i++) { g_ms += 1; distance_mm = 1000 + i; core_scope_update(); }
    text("|gone|");
    g_ready = 1;
    core_scope_update();
    for (int i = 0; i < 5; i++) { g_ms += 1; distance_mm = 2000 + i; core_scope_update(); }
    core_scope_enable(0);
    for (int i = 0; i < 20; i++) { g_ms += 1; distance_mm = 3000 + i; core_scope_update(); }
    text("|paused|");
    core_scope_enable(1);
    core_scope_update();
    for (int i = 0; i < 5; i++) { g_ms += 1; distance_mm = 4000 + i; core_scope_update(); }
}

/* A link that takes 7 bytes at a time (a slow UART): frames still arrive whole. */
static void sc_trickle(void)
{
    g_take_max = 7;
    core_scope_set_link(&g_link);
    core_scope_watch(distance_mm);
    core_scope_watch(temp_c);
    core_scope_update();
    for (int i = 0; i < 40; i++) { g_ms += 1; distance_mm = i; temp_c = (float)i; core_scope_update(); }
    for (int i = 0; i < 40; i++) core_scope_pump();
}

/* Registration limits. Prints nothing to the link; exit code is the verdict. */
static int sc_limits(void)
{
    static int32_t v[40];
    core_scope_set_link(&g_link);
    if (core_scope_watch_array(v, 33) == 0) return 1;      /* > 32 elements        */
    if (core_scope_watch_array(v, 30) != 0) return 2;
    if (core_scope_watch_array(v, 3) == 0) return 3;       /* would make 33: none  */
    if (core_scope_watch(distance_mm) != 0) return 4;      /* 31 */
    if (core_scope_watch(temp_c) != 0) return 5;           /* 32 */
    if (core_scope_watch(level) == 0) return 6;            /* full                 */
    return 0;
}

int main(int argc, char **argv)
{
    const char *s = argc > 1 ? argv[1] : "";
    if (!strcmp(s, "limits")) return sc_limits();
    if (!strcmp(s, "legacy")) sc_legacy();
    else if (!strcmp(s, "basic")) sc_basic();
    else if (!strcmp(s, "interval")) sc_interval();
    else if (!strcmp(s, "rows")) sc_rows();
    else if (!strcmp(s, "ble")) sc_ble();
    else if (!strcmp(s, "reconnect")) sc_reconnect();
    else if (!strcmp(s, "trickle")) sc_trickle();
    else return 64;
    finish();
    return 0;
}
