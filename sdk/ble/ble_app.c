/**
 * ble_app.c — BLE application init, event queue, and advertising
 *
 * Replaces the ble-wip's hal_ble.c + app_entry.c with a single file
 * that uses our LL/HAL infrastructure. No ST HAL dependency.
 */

#include <stdint.h>
#include <string.h>
#include "app_common.h"         /* Must come first — defines MAX, MIN, etc. */
#include "core_config.h"

/* The 2.4 GHz radio needs voltage range 1 (RM0493 §12.4.6); the "low" clock
 * level (HSI16, 16 MHz) runs in range 2. coregen refuses the combination when
 * config.json enables BLE; this catches a BLE_ENABLED=1 build that bypasses it. */
#if defined(SYSCLK_HZ) && (SYSCLK_HZ < 32000000UL)
#error "BLE needs clock medium or higher (the radio needs voltage range 1; clock low is HSI16 in range 2)"
#endif
#include "ll_common.h"
#include "ll_rcc.h"
#include "ll_pwr.h"
#include "ll_rtc.h"
#include "blestack.h"
#include "auto/ble_gap_aci.h"
#include "auto/ble_gatt_aci.h"
#include "auto/ble_hal_aci.h"
#include "ble_defs.h"
#include "ble_bufsize.h"
#include "bleplat.h"
#include "app_conf.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "stm32_mm.h"
#include "advanced_memory_manager.h"
#include "stm_list.h"
#include "svc_ctl.h"
#include "ll_sys.h"
#include "ll_sys_if.h"
#include "bpka.h"
#include "hw.h"

/* Forward declarations for binary lib functions */
extern void BleStackCB_Process(void);
extern void ll_sys_ble_cntrl_init(void *hostCallback);

/* ============================================================
 * BLE stack memory config
 * ============================================================ */

#define CFG_BLE_NUM_LINK             2
#define CFG_BLE_NUM_GATT_SERVICES    12
/* Each service reserves up to 25 attribute records (the 8-char-per-service cap
 * in ble_svc). Sized for ~8 user services + the built-in GAP/GATT — bump alongside
 * NUM_GATT_SERVICES if a multi-service app overflows (services silently fail to
 * register past the limit). 224 = 8*25 + headroom; the Ring uses 7 (DIS, Battery,
 * System, Motion, Haptics, Audio, Info). */
#define CFG_BLE_NUM_GATT_ATTRIBUTES  224
#define CFG_BLE_ATT_VALUE_ARRAY_SIZE 1344
#define CFG_BLE_ATT_MTU_MAX          251
#define CFG_BLE_MBLOCK_COUNT_MARGIN  0x15
#define CFG_BLE_COC_NBR_MAX          0
#define CFG_BLE_COC_MPS_MAX          0
#define CFG_BLE_COC_INITIATOR_NBR_MAX 0
#define PREP_WRITE_LIST_SIZE         0
#define CFG_BLE_OPTIONS              0

#ifndef DIVC
#define DIVC(x, y)  (((x) + (y) - 1) / (y))
#endif

/* Stack buffers */
#define BLE_GATT_BUF_SIZE \
    BLE_TOTAL_BUFFER_SIZE_GATT(CFG_BLE_NUM_GATT_ATTRIBUTES, \
                               CFG_BLE_NUM_GATT_SERVICES, \
                               CFG_BLE_ATT_VALUE_ARRAY_SIZE)

#define MBLOCK_COUNT (BLE_MBLOCKS_CALC(PREP_WRITE_LIST_SIZE, \
                                        CFG_BLE_ATT_MTU_MAX, \
                                        CFG_BLE_NUM_LINK) \
                      + CFG_BLE_MBLOCK_COUNT_MARGIN)

#define BLE_DYN_ALLOC_SIZE BLE_TOTAL_BUFFER_SIZE(CFG_BLE_NUM_LINK, MBLOCK_COUNT)

static uint32_t ble_buffer[DIVC(BLE_DYN_ALLOC_SIZE, 4)] __attribute__((aligned(4)));
static uint32_t gatt_buffer[DIVC(BLE_GATT_BUF_SIZE, 4)] __attribute__((aligned(4)));

/* GAP handles */
static uint16_t gap_service_handle;
static uint16_t gap_dev_name_handle;
static uint16_t gap_appearance_handle;

/* ============================================================
 * Async Event Queue — buffers HCI events from BLE stack
 * ============================================================ */

#define EVT_POOL_SIZE        8

typedef struct {
    tListNode node;
    struct {
        uint8_t type;
        struct {
            uint8_t evtcode;
            uint8_t plen;
            uint8_t payload[256];
        } evt;
    } evtserial;
} BleEvtPacket_t;

static BleEvtPacket_t evt_pool[EVT_POOL_SIZE];
static uint8_t evt_pool_used[EVT_POOL_SIZE];
static tListNode BleAsynchEventQueue;
static uint8_t ble_evt_queue_ready;

static BleEvtPacket_t *evt_pool_alloc(void)
{
    for (int i = 0; i < EVT_POOL_SIZE; i++) {
        if (!evt_pool_used[i]) {
            evt_pool_used[i] = 1;
            return &evt_pool[i];
        }
    }
    return (BleEvtPacket_t *)0;
}

static void evt_pool_free(BleEvtPacket_t *pkt)
{
    for (int i = 0; i < EVT_POOL_SIZE; i++) {
        if (&evt_pool[i] == pkt) {
            evt_pool_used[i] = 0;
            return;
        }
    }
}

/* Drain task — routes events through SVCCTL */
static void Ble_UserEvtRx(void)
{
    BleEvtPacket_t *pkt = (BleEvtPacket_t *)0;
    LST_remove_head(&BleAsynchEventQueue, (tListNode **)&pkt);
    if (!pkt) return;

    SVCCTL_UserEvtFlowStatus_t status;
    status = SVCCTL_UserEvtRx((void *)&(pkt->evtserial));

    if (status != SVCCTL_UserEvtFlowDisable) {
        evt_pool_free(pkt);
    } else {
        LST_insert_head(&BleAsynchEventQueue, (tListNode *)pkt);
    }

    if (LST_is_empty(&BleAsynchEventQueue) == FALSE) {
        UTIL_SEQ_SetTask(1U << CFG_TASK_HCI_ASYNCH_EVT_ID, 0);
    }

    UTIL_SEQ_SetTask(1U << CFG_TASK_BLE_HOST, 0);
}

/* BLECB_Indication — called by BleStack_Process for each HCI event */
uint8_t BLECB_Indication(const uint8_t *data, uint16_t length,
                         const uint8_t *ext_data, uint16_t ext_length)
{
    (void)length;
    (void)ext_data;
    (void)ext_length;

    if (!ble_evt_queue_ready) return 1;
    if (data[0] != HCI_EVENT_PKT_TYPE) return 1;

    BleEvtPacket_t *pkt = evt_pool_alloc();
    if (!pkt) return 1;

    pkt->evtserial.type = HCI_EVENT_PKT_TYPE;
    pkt->evtserial.evt.evtcode = data[1];
    pkt->evtserial.evt.plen = data[2];
    uint16_t copy_len = data[2];
    if (copy_len > sizeof(pkt->evtserial.evt.payload))
        copy_len = sizeof(pkt->evtserial.evt.payload);
    memcpy(pkt->evtserial.evt.payload, &data[3], copy_len);

    LST_insert_tail(&BleAsynchEventQueue, (tListNode *)pkt);
    UTIL_SEQ_SetTask(1U << CFG_TASK_HCI_ASYNCH_EVT_ID, 0);

    return 0;
}

/* BLE host background task */
static void ble_host_task(void)
{
    if (BleStack_Process() == 0x0) {
        BleStackCB_Process();
    }
}

/* ============================================================
 * HSE tuning from OTP
 * ============================================================ */

/* OTP on the WBA55: 512 B at 0x0BF90000 (FLASH_OTP_BASE / FLASH_OTP_SIZE in
 * the CubeWBA CMSIS header; core_otp.h uses the same). This pointed at
 * 0x0BFA0000, which isn't the OTP — hence the old "not accessible" TODO. */
#define OTP_AREA_BASE   0x0BF90000UL
#define OTP_AREA_SIZE   512u

/* RCC_ECSCR1 at offset 0x210: HSETRIM bits [21:16] (RM0493 §12.8.53) */
#define RCC_ECSCR1      REG32(RCC_BASE + 0x210UL)

/* Default HSE load-capacitor trim when the OTP holds none. Kept at the value
 * the stack has shipped with; changing it needs a frequency measurement. */
#define HSE_TRIM_DEFAULT  0x0Cu

/* ST's OTP record (CubeWBA otp.h, OTP_Data_s): 16-byte quad-word slots,
 *   [0..7] additional data, [8..13] BD address, [14] hsetune, [15] index.
 * OTP_Read(0) takes the LAST slot whose index byte is 0 (later writes
 * supersede earlier ones). A blank slot reads 0xFF throughout. */
static int otp_hsetune(uint8_t *out)
{
    for (uint32_t off = OTP_AREA_SIZE; off >= 16u; off -= 16u) {
        const volatile uint8_t *slot = (const volatile uint8_t *)(OTP_AREA_BASE + off - 16u);
        if (slot[15] == 0x00u && slot[14] <= 0x3Fu) {
            *out = slot[14];
            return 1;
        }
    }
    return 0;
}

static void config_hse_tuning(void)
{
    uint8_t hsetune = HSE_TRIM_DEFAULT;
    (void)otp_hsetune(&hsetune);          /* factory/board trim if programmed */

    /* Write HSE trim: RCC_ECSCR1 register, HSETRIM bits [21:16] */
    MOD_BITS(RCC_ECSCR1, 0x3FUL << 16, ((uint32_t)(hsetune & 0x3F)) << 16);
}

/* ============================================================
 * ble_app_init — THE main init function
 * ============================================================ */

static uint8_t ble_init_done;

/* Optional callback to register GATT services after SVCCTL_Init */
static void (*_register_services_cb)(void);
void ble_app_set_services_cb(void (*cb)(void)) { _register_services_cb = cb; }

/* Configurable parameters — set before calling ble_app_init/advertise */
uint8_t  ble_app_tx_power_code = 0x19;        /* default ~0 dBm */
uint16_t ble_app_adv_interval_min = 0x00A0;   /* default 100ms (units of 0.625ms) */
uint16_t ble_app_adv_interval_max = 0x00F0;   /* default 150ms */
uint8_t  ble_app_pairing_enabled = 0;         /* default: pairing disabled */

void ble_app_init(void)
{
    /* 0. The radio needs range 1 with hclk5 undivided by HDIV5 (RM0493
     *    §12.4.6). core_clock_init() already does this for every level BLE
     *    builds at (coregen refuses BLE with clock "low"). This used to write
     *    RCC_CFGR4 = 0, which also zeroed HPRE5: on the PLL levels that put
     *    hclk5 at 64 / 100 MHz (limit 32) and changed HPRE5 while SYSCLK ran
     *    from the PLL, which RM0493 §12.8.51 forbids. Only HDIV5 is touched. */
    if (ll_pwr_get_vos() == 1u)
        ll_rcc_set_hdiv5(0);

    /* 0b. Enable instruction cache (1-way mode) */
    {
        #define ICACHE_NS_BASE  (PERIPH_BASE + 0x00030400UL)  /* 0x40030400 */
        #define ICACHE_CR_REG   REG32(ICACHE_NS_BASE + 0x00UL)
        ICACHE_CR_REG = (1UL << 2);  /* WAYSEL = 1-way */
        ICACHE_CR_REG = (1UL << 2) | (1UL << 0);  /* WAYSEL + EN */
    }

    /* 0. Run AES test vector to validate byte ordering */
    extern void baes_run_test(void);
    baes_run_test();

    /* 1. HSE tuning from OTP (HSE must already be running — BLE requires it) */
    config_hse_tuning();

    /* 1b. Match working project's peripheral clock config:
     *     - RNG clock source = HSI (CCIPR2 RNGSEL bits [13:12] = 0b10)
     *     - Enable HASH clock (AHB2ENR bit 16) — used as BLE SW low ISR
     *     - Enable RNG clock (AHB2ENR bit 18) */
    MOD_BITS(REG32(RCC_BASE + 0xE4UL), 0x3UL << 12, 0x2UL << 12);
    SET_BITS(REG32(RCC_BASE + 0x8CUL), (1UL << 17) | (1UL << 18));  /* HASHEN(bit17) + RNGEN(bit18) */

    /* Configure RNG: enable conditioning, set NIST compliance */
    {
        #define RNG_BASE_NS  (PERIPH_BASE + 0x020C0800UL)  /* 0x420C0800 */
        #define RNG_CR_REG   REG32(RNG_BASE_NS + 0x00UL)
        /* Disable RNG before config */
        RNG_CR_REG = 0;
        /* RNGEN=1, CONDRST=1 (start conditioning reset) */
        RNG_CR_REG = (1UL << 2) | (1UL << 6);
        /* Clear CONDRST to complete conditioning config */
        RNG_CR_REG = (1UL << 2);
    }

    /* 2a. The RTC is the SDK's (core_rtc / core_stop_for): a 1 Hz calendar on
     *     LSI, left as it is if it already runs. The BLE timer server runs on
     *     LPTIM2 + SysTick (stm32_timer_if.c), not the RTC. This used to put
     *     the RTC in "binary mode" by setting RTC_CR bits 9:8 — which are
     *     ALRBE:ALRAE, so it enabled both alarms (BIN is RTC_ICSR[9:8],
     *     RM0493 §36.6.4) — and set PRER to PREDIV_A 31 / PREDIV_S 0, a 1 kHz
     *     calendar "second" that broke core_rtc time and the RTC-measured
     *     core_stop_for until the next ll_rtc_init. It runs before the radio
     *     sleep clock below: if RTCSEL holds another source, ll_rtc_init()
     *     resets the backup domain, which also clears RADIOSTSEL and LSI1. */
    ll_rtc_ensure();

    /* 2. Radio sleep clock setup — HSE/1024.
     *    Register dump of working CubeWBA project confirms RADIOSTSEL=0b10
     *    (HSE/1024) despite its code saying RCC_RADIOSTCLKSOURCE_LSI —
     *    the HAL value maps differently than the LL value. */
    ll_pwr_enable_backup_access();
    ll_rcc_lsi1_enable_wait();  /* LSI1 still needed by link layer */
    ll_rcc_set_radio_sleep_clk(LL_RCC_RADIOSLEEPSOURCE_LSI);


    /* 3. Initialize sequencer */
    UTIL_SEQ_Init();

    /* 4. Initialize timer server */
    UTIL_TIMER_Init();

    /* 5. Initialize link layer controller
     *    ll_sys_ble_cntrl_init calls ll_sys_bg_process_init and
     *    ll_sys_config_params internally. Pass HostStack_Process as the
     *    host callback. */
    extern void HostStack_Process(void);
    ll_sys_ble_cntrl_init((void *)HostStack_Process);

    /* 6. Initialize BLEPLAT (crypto, timer, RNG dispatch) */
    BLEPLAT_Init();

    /* 7. Initialize AMM */
    static AMM_InitParameters_t amm_init;
    static AMM_VirtualMemoryConfig_t amm_vms[CFG_AMM_VIRTUAL_MEMORY_NUMBER];
    amm_vms[0].Id = CFG_AMM_VIRTUAL_STACK_BLE;
    amm_vms[0].BufferSize = CFG_AMM_VIRTUAL_STACK_BLE_BUFFER_SIZE;
    amm_vms[1].Id = CFG_AMM_VIRTUAL_APP_BLE;
    amm_vms[1].BufferSize = CFG_AMM_VIRTUAL_APP_BLE_BUFFER_SIZE;
    amm_init.p_PoolAddr = NULL;  /* Set by AMM_RegisterBasicMemoryManager callback */
    amm_init.PoolSize = CFG_AMM_POOL_SIZE;
    amm_init.VirtualMemoryNumber = CFG_AMM_VIRTUAL_MEMORY_NUMBER;
    amm_init.p_VirtualMemoryConfigList = amm_vms;
    AMM_Init(&amm_init);

    /* Register AMM + BPKA background tasks */
    UTIL_SEQ_RegTask(1U << CFG_TASK_AMM, 0, AMM_BackgroundProcess);
    UTIL_SEQ_RegTask(1U << CFG_TASK_BPKA, 0, BPKA_BG_Process);

    /* 8. Event queue + host task */
    LST_init_head(&BleAsynchEventQueue);
    memset(evt_pool_used, 0, sizeof(evt_pool_used));
    ble_evt_queue_ready = 1;
    UTIL_SEQ_RegTask(1U << CFG_TASK_HCI_ASYNCH_EVT_ID, 0, Ble_UserEvtRx);
    UTIL_SEQ_RegTask(1U << CFG_TASK_BLE_HOST, 0, ble_host_task);

    /* 9. Initialize NVM (RAM-backed security database) */
    extern void NVM_Init(void *buffer, uint32_t offset, uint32_t size);
    NVM_Init(NULL, 0, 0);

    /* 10. Initialize BLE stack */
    BleStack_init_t params;
    params.numAttrRecord           = CFG_BLE_NUM_GATT_ATTRIBUTES;
    params.numAttrServ             = CFG_BLE_NUM_GATT_SERVICES;
    params.attrValueArrSize        = CFG_BLE_ATT_VALUE_ARRAY_SIZE;
    params.prWriteListSize         = PREP_WRITE_LIST_SIZE;
    params.attMtu                  = CFG_BLE_ATT_MTU_MAX;
    params.max_coc_nbr             = CFG_BLE_COC_NBR_MAX;
    params.max_coc_mps             = CFG_BLE_COC_MPS_MAX;
    params.max_coc_initiator_nbr   = CFG_BLE_COC_INITIATOR_NBR_MAX;
    params.numOfLinks              = CFG_BLE_NUM_LINK;
    params.mblockCount             = MBLOCK_COUNT;
    params.bleStartRamAddress      = (uint8_t *)ble_buffer;
    params.total_buffer_size       = BLE_DYN_ALLOC_SIZE;
    params.bleStartRamAddress_GATT = (uint8_t *)gatt_buffer;
    params.total_buffer_size_GATT  = BLE_GATT_BUF_SIZE;
    params.options                 = CFG_BLE_OPTIONS;
    params.debug                   = 0U;

    BleStack_Init(&params);

    /* Public BD address from the IEEE 64-bit unique ID (DESIG_UID64R1/R2 at
     * 0x0BF90A00, RM0493 §44.1.11-12), the way CubeWBA builds it:
     *   [47:24] STID, ST's IEEE company ID (OUI 00:80:E1)
     *   [23:16] DEVID, [15:0] low 16 bits of the device number (DEVNUM)
     * The old address took bytes of the 96-bit UID (wafer / lot number), so
     * its top 24 bits were not anyone's OUI: an unregistered "public" address
     * that could collide with a real vendor's. This one is in ST's registered
     * block, keeps the public address type the GAP/advertising/pairing calls
     * below already use, and is stable per chip. ST's note: a shipping
     * product should use its own OUI (or a static random address). */
    {
        const volatile uint32_t *uid64 = (const volatile uint32_t *)0x0BF90A00UL;
        uint32_t devnum = uid64[0];            /* UID64R1: DEVNUM[31:0] */
        uint32_t r2     = uid64[1];            /* UID64R2: STID[23:0] << 8 | DEVID */
        uint8_t bd_addr[6];
        if (devnum != 0xFFFFFFFFUL) {
            bd_addr[0] = (uint8_t)(devnum);
            bd_addr[1] = (uint8_t)(devnum >> 8);
            bd_addr[2] = (uint8_t)(r2);            /* DEVID */
            bd_addr[3] = (uint8_t)(r2 >> 8);       /* STID, LSB first */
            bd_addr[4] = (uint8_t)(r2 >> 16);
            bd_addr[5] = (uint8_t)(r2 >> 24);
        } else {
            /* Unprogrammed UID64 (not expected on production parts): keep the
             * previous UID96-derived bytes so the board still has an address. */
            const volatile uint32_t *uid = (const volatile uint32_t *)0x0BF90700UL;
            bd_addr[0] = (uint8_t)(uid[0]);
            bd_addr[1] = (uint8_t)(uid[0] >> 8);
            bd_addr[2] = (uint8_t)(uid[0] >> 16);
            bd_addr[3] = (uint8_t)(uid[1]);
            bd_addr[4] = (uint8_t)(uid[1] >> 8);
            bd_addr[5] = (uint8_t)(uid[1] >> 16);
        }
        aci_hal_write_config_data(0x00 /* CONFIG_DATA_PUBADDR_OFFSET */, 6, bd_addr);
    }

    /* Set Identity Root (IR) and Encryption Root (ER) keys.
     * Required for connection security — even without pairing. */
    {
        uint8_t ir[16] = {0x12,0x34,0x56,0x78,0x9A,0xBC,0xDE,0xF0,
                          0x12,0x34,0x56,0x78,0x9A,0xBC,0xDE,0xF0};
        uint8_t er[16] = {0xFE,0xDC,0xBA,0x09,0x87,0x65,0x43,0x21,
                          0xFE,0xDC,0xBA,0x09,0x87,0x65,0x43,0x21};
        aci_hal_write_config_data(0x18 /* CONFIG_DATA_IR_OFFSET */, 16, ir);
        aci_hal_write_config_data(0x08 /* CONFIG_DATA_ER_OFFSET */, 16, er);
    }

    /* Set TX power — configurable via ble_app_tx_power_code */
    aci_hal_set_tx_power_level(1, ble_app_tx_power_code);

    /* Init GATT + GAP */
    aci_gatt_init();
    aci_gap_init(0x01 /* GAP_PERIPHERAL_ROLE */, 0x00 /* public addr */, 20,
                 &gap_service_handle,
                 &gap_dev_name_handle,
                 &gap_appearance_handle);

    /* Set up security — configurable via ble_app_pairing_enabled */
    if (ble_app_pairing_enabled) {
        aci_gap_set_io_capability(IO_CAP_NO_INPUT_NO_OUTPUT);
        aci_gap_set_authentication_requirement(
            1,                          /* bonding mode: enabled */
            0,                          /* MITM: not required */
            0x00,                       /* SC: NOT supported (legacy only) */
            KEYPRESS_NOT_SUPPORTED,
            8, 16,                      /* encryption key size min/max */
            0, 0,                       /* no fixed pin */
            GAP_PUBLIC_ADDR
        );
    }

    /* Init service controller */
    SVCCTL_Init();

    /* Init application GATT services (registered via ble_app_register_services_cb) */
    if (_register_services_cb) _register_services_cb();

    ble_init_done = 1;
}

/* ============================================================
 * ble_app_advertise — start BLE advertising
 * ============================================================ */

int ble_app_advertise(const char *name)
{
    tBleStatus ret;

    /* Set the GAP Device Name characteristic to match the advertised name —
     * the stack defaults it to "STM32WB!", which is what phones display. */
    if (name) {
        uint8_t n = 0;
        while (name[n] && n < 248) n++;
        aci_gatt_update_char_value(gap_service_handle, gap_dev_name_handle, 0,
                                   n, (const uint8_t *)name);
    }

    /* Match the working CubeWBA project exactly:
     * 1. aci_gap_set_discoverable with NO name (all zeros)
     * 2. aci_gap_delete_ad_type to remove TX power level
     * 3. aci_gap_update_adv_data with explicit advertising data */

    ret = aci_gap_set_discoverable(
        0x00,                       /* ADV_IND */
        ble_app_adv_interval_min,   /* min interval */
        ble_app_adv_interval_max,   /* max interval */
        0x00,                       /* public address */
        0x00,                       /* no filter */
        0, NULL,        /* no local name in this call */
        0, NULL,        /* no service UUIDs */
        0x0006, 0x0010  /* conn interval min/max */
    );

    if (ret != 0) return -1;

    /* Remove TX power level from advertising data */
    #ifndef AD_TYPE_TX_POWER_LEVEL
    #define AD_TYPE_TX_POWER_LEVEL  0x0A
    #endif
    #ifndef AD_TYPE_COMPLETE_NAME
    #define AD_TYPE_COMPLETE_NAME   0x09
    #endif
    aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL);

    /* Build advertising data */
    uint8_t name_len = 0;
    while (name[name_len] && name_len < 20) name_len++;

    uint8_t adv_data[31];
    uint8_t pos = 0;

    /* AD element: Complete Local Name */
    adv_data[pos++] = name_len + 1;
    adv_data[pos++] = AD_TYPE_COMPLETE_NAME;
    for (uint8_t i = 0; i < name_len; i++)
        adv_data[pos++] = (uint8_t)name[i];


    ret = aci_gap_update_adv_data(pos, adv_data);

    return (ret == 0) ? 0 : -1;
}

/* ============================================================
 * ble_app_stop_advertise — stop BLE advertising
 * ============================================================ */

int ble_app_stop_advertise(void)
{
    tBleStatus ret = aci_gap_set_non_discoverable();
    return (ret == 0) ? 0 : -1;
}
