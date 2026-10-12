/**
 * prov_test.c — host tests for sdk/thread/thread_prov_codec.c
 *
 * The dataset blob (encode / decode / every validation error), the status
 * layout, and the boot precedence rule, the last both as a truth table and as
 * a sequence of boots, provisions, firmware updates and factory resets run
 * against a model of the settings flash. The two hex strings below are the
 * worked example in docs/thread-provisioning-protocol.md: change one and the
 * other in the same commit.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "thread_prov.h"

static int s_fail, s_pass;

#define CHECK(cond) do { \
    if (cond) s_pass++; \
    else { s_fail++; fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

/* docs/thread-provisioning-protocol.md §6: the dataset written to 0x5D02. */
static const char DOC_DATASET_HEX[] =
    "01001407efbe1122334455667788fd112233445566770f1e2d3c4b5a69788796"
    "a5b4c3d2e1f0000000006c61622d6e6574000000000000000000";

/* §6: the status the node then reports on 0x5D01 (a router, provisioned). */
static const char DOC_STATUS_HEX[] =
    "01030d14efbe004478563412027f1122334455667788076c61622d6e6574000000000000000000";

static size_t unhex(const char *s, uint8_t *out, size_t max)
{
    size_t n = 0;
    while (*s && n < max) {
        if (*s == ' ') { s++; continue; }
        unsigned v;
        if (sscanf(s, "%2x", &v) != 1)
            break;
        out[n++] = (uint8_t)v;
        s += 2;
    }
    return n;
}

static void hex(const uint8_t *p, size_t n, char *out)
{
    for (size_t i = 0; i < n; i++)
        sprintf(out + 2 * i, "%02x", p[i]);
}

static core_thread_dataset_t example(void)
{
    core_thread_dataset_t ds;
    static const uint8_t xpan[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
    static const uint8_t mlp[8]  = { 0xfd, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77 };
    static const uint8_t key[16] = { 0x0f, 0x1e, 0x2d, 0x3c, 0x4b, 0x5a, 0x69, 0x78,
                                     0x87, 0x96, 0xa5, 0xb4, 0xc3, 0xd2, 0xe1, 0xf0 };
    memset(&ds, 0, sizeof ds);
    ds.channel = 20;
    ds.panid = 0xBEEF;
    memcpy(ds.ext_panid, xpan, 8);
    memcpy(ds.mesh_local_prefix, mlp, 8);
    memcpy(ds.network_key, key, 16);
    strcpy(ds.network_name, "lab-net");
    ds.active_timestamp = 0;
    return ds;
}

static int all_zero(const void *p, size_t n)
{
    const uint8_t *b = p;
    for (size_t i = 0; i < n; i++)
        if (b[i])
            return 0;
    return 1;
}

/* ============================================================
 * Dataset blob
 * ============================================================ */

static void test_dataset_roundtrip(void)
{
    core_thread_dataset_t ds = example(), back;
    uint8_t blob[THREAD_PROV_DATASET_LEN], doc[THREAD_PROV_DATASET_LEN + 4];
    char h[2 * THREAD_PROV_DATASET_LEN + 1];

    CHECK(thread_prov_encode_dataset(&ds, blob) == THREAD_PROV_DATASET_LEN);
    hex(blob, sizeof blob, h);
    CHECK(strcmp(h, DOC_DATASET_HEX) == 0);
    if (strcmp(h, DOC_DATASET_HEX) != 0)
        fprintf(stderr, "  encoded: %s\n", h);
    CHECK(unhex(DOC_DATASET_HEX, doc, sizeof doc) == THREAD_PROV_DATASET_LEN);

    CHECK(thread_prov_decode_dataset(doc, THREAD_PROV_DATASET_LEN, &back) == THREAD_PROV_OK);
    CHECK(memcmp(&back, &ds, sizeof ds) == 0);          /* both zero-filled, padding too */
    CHECK(thread_prov_timestamp(&back) == 1u);              /* 0 stands for 1 */
    back.active_timestamp = 1700000000u;
    CHECK(thread_prov_timestamp(&back) == 1700000000u);

    /* A 16-byte name fills its field with no terminator in the blob. */
    strcpy(ds.network_name, "0123456789abcdef");
    ds.active_timestamp = 0x01020304u;
    thread_prov_encode_dataset(&ds, blob);
    CHECK(blob[3] == 16 && memcmp(&blob[42], "0123456789abcdef", 16) == 0);
    CHECK(blob[38] == 0x04 && blob[41] == 0x01);           /* little-endian */
    CHECK(thread_prov_decode_dataset(blob, sizeof blob, &back) == THREAD_PROV_OK);
    CHECK(strcmp(back.network_name, "0123456789abcdef") == 0 && back.active_timestamp == 0x01020304u);
}

static int decode_with(uint8_t *blob, uint16_t len)
{
    core_thread_dataset_t out;
    memset(&out, 0xA5, sizeof out);
    int rc = thread_prov_decode_dataset(blob, len, &out);
    if (rc != THREAD_PROV_OK)
        CHECK(all_zero(&out, sizeof out));                 /* nothing (no key) left behind */
    return rc;
}

static void test_dataset_errors(void)
{
    core_thread_dataset_t ds = example();
    uint8_t good[THREAD_PROV_DATASET_LEN + 1], b[THREAD_PROV_DATASET_LEN + 1];
    thread_prov_encode_dataset(&ds, good);
    good[THREAD_PROV_DATASET_LEN] = 0;

#define FRESH() memcpy(b, good, sizeof b)
    FRESH(); CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_OK);
    FRESH(); CHECK(decode_with(b, THREAD_PROV_DATASET_LEN - 1) == THREAD_PROV_ERR_LENGTH);
    FRESH(); CHECK(decode_with(b, THREAD_PROV_DATASET_LEN + 1) == THREAD_PROV_ERR_LENGTH);
    FRESH(); CHECK(decode_with(b, 0) == THREAD_PROV_ERR_LENGTH);
    FRESH(); CHECK(decode_with(b, 20) == THREAD_PROV_ERR_LENGTH);       /* an MTU-23 fragment */
    {
        core_thread_dataset_t out;
        CHECK(thread_prov_decode_dataset(NULL, THREAD_PROV_DATASET_LEN, &out) == THREAD_PROV_ERR_LENGTH);
    }
    FRESH(); b[0] = 0; CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_ERR_VERSION);
    FRESH(); b[0] = 2; CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_ERR_VERSION);
    FRESH(); b[1] = 1; CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_ERR_RESERVED);
    FRESH(); b[1] = 0x80; CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_ERR_RESERVED);

    static const uint8_t channels_ok[] = { 11, 15, 26 }, channels_bad[] = { 0, 10, 27, 255 };
    for (unsigned i = 0; i < sizeof channels_ok; i++) {
        FRESH(); b[2] = channels_ok[i]; CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_OK);
    }
    for (unsigned i = 0; i < sizeof channels_bad; i++) {
        FRESH(); b[2] = channels_bad[i]; CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_ERR_CHANNEL);
    }

    FRESH(); b[4] = 0xFF; b[5] = 0xFF; CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_ERR_PANID);
    FRESH(); b[4] = 0xFE; b[5] = 0xFF; CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_OK);
    FRESH(); b[4] = 0x00; b[5] = 0x00; CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_OK);

    FRESH(); b[3] = 0;  CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_ERR_NAME);
    FRESH(); b[3] = 17; CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_ERR_NAME);
    FRESH(); b[3] = 8;  CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_ERR_NAME);   /* takes in the NUL after "lab-net" */
    FRESH(); b[42 + 3] = 0; CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_ERR_NAME);
    FRESH(); b[3] = 3;  CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_OK);          /* "lab" */
    /* Bytes past the name length are padding and ignored. */
    FRESH(); b[42 + 10] = 'x'; CHECK(decode_with(b, THREAD_PROV_DATASET_LEN) == THREAD_PROV_OK);
#undef FRESH
}

static void test_check(void)
{
    core_thread_dataset_t ds = example();
    CHECK(thread_prov_check(&ds) == THREAD_PROV_OK);
    ds.channel = 10; CHECK(thread_prov_check(&ds) == THREAD_PROV_ERR_CHANNEL);
    ds = example(); ds.panid = 0xFFFF; CHECK(thread_prov_check(&ds) == THREAD_PROV_ERR_PANID);
    ds = example(); ds.network_name[0] = 0; CHECK(thread_prov_check(&ds) == THREAD_PROV_ERR_NAME);
    ds = example(); memset(ds.network_name, 'a', sizeof ds.network_name);     /* no terminator */
    CHECK(thread_prov_check(&ds) == THREAD_PROV_ERR_NAME);
    ds.network_name[16] = 0; CHECK(thread_prov_check(&ds) == THREAD_PROV_OK);
}

/* ============================================================
 * Status
 * ============================================================ */

static void test_status(void)
{
    thread_prov_status_t st;
    uint8_t out[THREAD_PROV_STATUS_LEN], doc[THREAD_PROV_STATUS_LEN + 4];
    char h[2 * THREAD_PROV_STATUS_LEN + 1];

    memset(&st, 0, sizeof st);
    st.role = CORE_THREAD_ROUTER;
    st.flags = THREAD_PROV_FLAG_PROVISIONED | THREAD_PROV_FLAG_ENABLED | THREAD_PROV_FLAG_WRITABLE;
    st.rloc16 = 0x4400;
    st.partition_id = 0x12345678u;
    st.neighbors = 2;
    st.parent_rssi = 127;
    st.net = example();                       /* the key must not appear */
    CHECK(thread_prov_encode_status(&st, out) == THREAD_PROV_STATUS_LEN);
    hex(out, sizeof out, h);
    CHECK(unhex(DOC_STATUS_HEX, doc, sizeof doc) == THREAD_PROV_STATUS_LEN);
    CHECK(memcmp(out, doc, THREAD_PROV_STATUS_LEN) == 0);
    if (memcmp(out, doc, THREAD_PROV_STATUS_LEN) != 0)
        fprintf(stderr, "  status: %s\n", h);
    CHECK(strstr(h, "0f1e2d3c") == NULL);

    /* Negative RSSI, and the empty status before core_thread_start(). */
    st.role = CORE_THREAD_CHILD;
    st.parent_rssi = -67;
    thread_prov_encode_status(&st, out);
    CHECK(out[1] == 2 && (int8_t)out[13] == -67);
    memset(&st, 0, sizeof st);
    st.rloc16 = 0xFFFE;
    st.parent_rssi = 127;
    thread_prov_encode_status(&st, out);
    CHECK(out[0] == 1 && out[1] == 0 && out[6] == 0xFE && out[7] == 0xFF && out[22] == 0);
}

/* ============================================================
 * Boot precedence
 * ============================================================ */

static void test_boot_table(void)
{
    /* marked, stored_ok, stored_is_config */
    CHECK(thread_prov_boot_action(1, 1, 0) == THREAD_PROV_BOOT_KEEP);   /* provisioned network wins */
    CHECK(thread_prov_boot_action(1, 1, 1) == THREAD_PROV_BOOT_KEEP);   /* provisioned = config's */
    CHECK(thread_prov_boot_action(1, 0, 0) == THREAD_PROV_BOOT_WRITE_CONFIG_CLEAR_MARK);  /* lost */
    CHECK(thread_prov_boot_action(0, 1, 1) == THREAD_PROV_BOOT_KEEP);   /* unchanged: no flash write */
    CHECK(thread_prov_boot_action(0, 1, 0) == THREAD_PROV_BOOT_WRITE_CONFIG);  /* config.json changed */
    CHECK(thread_prov_boot_action(0, 0, 0) == THREAD_PROV_BOOT_WRITE_CONFIG);  /* first boot */
}

/* The settings flash, reduced to what precedence depends on. Networks are
 * small integers; 0 = no (or a corrupt) active dataset. */
typedef struct {
    int marked;
    int stored;
    int writes;       /* dataset writes, to see that a quiet boot writes nothing */
} store_t;

static int boot(store_t *s, int config)
{
    int a = thread_prov_boot_action(s->marked, s->stored != 0, s->stored == config);
    if (a == THREAD_PROV_BOOT_WRITE_CONFIG_CLEAR_MARK)
        s->marked = 0;
    if (a != THREAD_PROV_BOOT_KEEP) {
        s->stored = config;
        s->writes++;
    }
    return s->stored;                 /* the network the node runs */
}

static void provision(store_t *s, int net) { s->stored = net; s->marked = 1; s->writes++; }
static void factory_reset(store_t *s, int config) { s->stored = config; s->marked = 0; s->writes++; }

static void test_boot_sequence(void)
{
    store_t s = { 0, 0, 0 };
    int config = 100;

    CHECK(boot(&s, config) == 100 && s.writes == 1);        /* first boot: config.json's */
    CHECK(boot(&s, config) == 100 && s.writes == 1);        /* reboot: nothing written */
    config = 101;                                           /* reflash with a new config.json */
    CHECK(boot(&s, config) == 101 && s.writes == 2);        /* not provisioned: config wins */

    provision(&s, 7);                                       /* phone sets network 7 */
    CHECK(boot(&s, config) == 7);                           /* reboot: stored wins */
    int w = s.writes;
    CHECK(boot(&s, config) == 7 && s.writes == w);          /* and writes nothing */
    config = 102;                                           /* BLE update with another config.json */
    CHECK(boot(&s, config) == 7);                           /* still the provisioned network */

    provision(&s, 102);                                     /* provisioned onto config's own values */
    config = 103;
    CHECK(boot(&s, config) == 102);                         /* stays: it was provisioned */

    factory_reset(&s, config);                              /* back to config.json's */
    CHECK(s.stored == 103 && !s.marked);
    config = 104;
    CHECK(boot(&s, config) == 104);                         /* config follows the image again */

    provision(&s, 9);
    s.stored = 0;                                           /* dataset lost (corrupt settings) */
    CHECK(boot(&s, config) == 104 && !s.marked);            /* falls back, forgets the mark */
    config = 105;
    CHECK(boot(&s, config) == 105);
}

int main(void)
{
    test_dataset_roundtrip();
    test_dataset_errors();
    test_check();
    test_status();
    test_boot_table();
    test_boot_sequence();

    printf("host-thread-prov: %d passed, %d failed\n", s_pass, s_fail);
    return s_fail ? EXIT_FAILURE : EXIT_SUCCESS;
}
