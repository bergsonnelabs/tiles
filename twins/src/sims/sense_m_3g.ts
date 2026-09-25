// Digital twin for Sense.M.3G — Bosch Sensortec BMM350 3-axis magnetometer
// (driver tile_sense_m_3g.{h,c}; datasheet BST-BMM350-DS001-27).
//
// The world (the magnetic field at the tile, in µT, and the die temperature)
// is what a person moves. The chip converts it into its data registers in
// normal mode at the configured ODR, or once per forced-mode trigger; the
// driver's read() copies the registers into its cache, and the get_* calls
// return that cache. So, as on hardware, nothing changes until the program
// puts the part in normal mode (init leaves it in suspend) and reads.
//
// Values are the driver's COMPENSATED output (nT, m°C). The twin returns the
// ideal field: no noise (datasheet ~190 nT rms), no offset or gain error, no
// OTP trim. get_raw back-computes counts from the ideal value with the
// driver's own scaling constants, without the trim.
//
// Pads (schematic): GND 1, SCL 4, SDA 5, INT 9 (push-pull at VDDIO), V+ 10.
// VDDIO is V+; VDD (1.72-1.98 V) comes from an on-tile 1.8 V LDO off V+, so
// the supply current on pad 10 is the BMM350's plus the LDO's own. ADSEL is
// tied to GND: address 0x14.
import type { PowerCtx, TileSim } from '../tileSim';

interface State {
  // ── the world (stimuli) ──
  field_x_ut: number;
  field_y_ut: number;
  field_z_ut: number;
  die_temp_c: number;

  // ── chip data registers (the last conversion) ──
  conv_x_nt: number;
  conv_y_nt: number;
  conv_z_nt: number;
  conv_temp_mc: number;
  /** INT_STATUS.drdy_data_reg: set by a conversion, cleared by reading it. */
  drdy: number;
  /** `t` of the last conversion, and of the last tick. */
  last_conv_t: number;
  last_tick_t: number;
  /** Internal sensor-time counter (40 µs ticks) and its value latched at the
   * last conversion (SENSORTIME registers). */
  sensor_clock: number;
  sensortime: number;

  // ── driver cache (the last read()) ──
  mag_x_nt: number;
  mag_y_nt: number;
  mag_z_nt: number;
  temp_mc: number;

  // ── mode / configuration ──
  /** Device power mode: 0 suspend, 1 normal (forced returns to suspend). */
  mode: number;
  /** tile->state: 1 = READY (process() runs), 0 = SLEEPING (after init / sleep). */
  tile_ready: number;
  /** sense_m_3g_odr_t code (0x2..0xA). */
  odr: number;
  /** sense_m_3g_avg_t code (0..3). */
  avg: number;
  /** Axis-enable mask, BMM350_EN_* (bit 0 X, 1 Y, 2 Z). */
  axis_en: number;
  /** INT_CTRL fields as configure_interrupt() set them. */
  int_output_en: number;
  int_active_high: number;
  int_push_pull: number;
  int_latched: number;
  pad_drive: number;
  i2c_wdt: number; // I2C_WDT_SET: bit 0 enable, bit 1 long
  sensortime_aon: number;
  /** Last PMU command issued (PMU_CMD_STATUS_0[7:5] per the SensorAPI). */
  last_pmu_cmd: number;
  /** A forced conversion has run since bring-up: the driver's first trigger
   * uses FM, later ones FM_FAST at 25 Hz or faster. */
  fm_primed: number;
}

// PMU_CMD codes
const CMD_SUS = 0x00;
const CMD_NM = 0x01;
const CMD_UPD_OAE = 0x02;
const CMD_FM = 0x03;
const CMD_FM_FAST = 0x04;
const CMD_FGR = 0x05;

/** ODR code → Hz (datasheet §8.4). */
export const ODR_HZ: Readonly<Record<number, number>> = {
  2: 400,
  3: 200,
  4: 100,
  5: 50,
  6: 25,
  7: 12.5,
  8: 6.25,
  9: 3.125,
  10: 1.5625,
};
const ODR_25HZ = 6;
const AVG_4 = 2;
const EN_XYZ = 0x07;
const FULL_SCALE_NT = 2_000_000; // ±2000 µT per axis (datasheet Table 2)
const SENSORTIME_TICKS_PER_MS = 25; // 40 µs per tick
/** In pulsed mode INT_STATUS.drdy clears itself after 1.25 ms (datasheet §5.5). */
const PULSE_MS = 1.25;

// The driver's raw-count scaling (tile_sense_m_3g.c, [API]), for get_raw.
const NT_PER_LSB_XY = 463338 / 65536;
const NT_PER_LSB_Z = 470218 / 65536;
const MC_PER_LSB_T = 64309 / 65536;
const TEMP_RAW_OFFSET_MC = 25490;

/** The driver's clamp of an illegal ODR / averaging pair (datasheet Table 5). */
export function clampAvg(odr: number, avg: number): number {
  if (odr === 2) return Math.min(avg, 0);
  if (odr === 3) return Math.min(avg, 1);
  if (odr === 4) return Math.min(avg, 2);
  return avg & 0x03;
}

const sat = (nt: number) => Math.max(-FULL_SCALE_NT, Math.min(FULL_SCALE_NT, Math.round(nt)));

/** One conversion of the world into the data registers. Disabled axes read 0
 * after compensation (the driver zeroes them). */
function convert(s: State): Partial<State> {
  return {
    conv_x_nt: s.axis_en & 1 ? sat(s.field_x_ut * 1000) : 0,
    conv_y_nt: s.axis_en & 2 ? sat(s.field_y_ut * 1000) : 0,
    conv_z_nt: s.axis_en & 4 ? sat(s.field_z_ut * 1000) : 0,
    conv_temp_mc: Math.round(s.die_temp_c * 1000),
    drdy: 1,
    sensortime: s.sensor_clock & 0xffffff,
  };
}

/** read(): the burst copy into the driver's cache. */
const readIntoCache = (s: State): Partial<State> => ({
  mag_x_nt: s.conv_x_nt,
  mag_y_nt: s.conv_y_nt,
  mag_z_nt: s.conv_z_nt,
  temp_mc: s.conv_temp_mc,
});

/** set_mode(), including the driver's park-in-suspend before forced mode and
 * the device's return to suspend after one forced conversion. */
function enterMode(s: State, mode: number): Partial<State> {
  const tile_ready = mode === CMD_SUS ? 0 : 1;
  if (mode === CMD_NM) return { mode: 1, tile_ready, last_pmu_cmd: CMD_NM };
  if (mode === CMD_FM || mode === CMD_FM_FAST)
    return { ...convert(s), mode: 0, tile_ready, last_pmu_cmd: mode };
  return { mode: 0, tile_ready, last_pmu_cmd: CMD_SUS };
}

const arg = (args: number[], i: number, fallback: number) =>
  args.length > i && Number.isFinite(args[i]) ? args[i] : fallback;

// ── Supply current (datasheet Table 5, µA, "approximate") ──
// Rows are averaging 1/2/4/8; columns 400 … 1.5625 Hz. The datasheet elides
// 6.25 and 3.125 Hz ("…"); those are geometric means of their neighbours.
// Illegal pairs cannot occur (the driver clamps). Suspend 1.8 µA (Table 1).
const TABLE5: Readonly<Record<number, Readonly<Record<number, number>>>> = {
  0: { 2: 455, 3: 235, 4: 122, 5: 70, 6: 40, 7: 23, 10: 12 },
  1: { 3: 370, 4: 190, 5: 100, 6: 55, 7: 33, 10: 15 },
  2: { 4: 335, 5: 175, 6: 96, 7: 57, 10: 25 },
  3: { 5: 325, 6: 180, 7: 108, 10: 50 },
};
const I_SUSPEND_UA = 1.8;
/** The 1.8 V LDO's ground current. */
const LDO_IQ_UA = 6.5;
export function normalCurrentUa(odr: number, avg: number): number {
  const r = TABLE5[clampAvg(odr, avg & 3)];
  if (r[odr] != null) return r[odr];
  // 6.25 Hz (8) and 3.125 Hz (9): log-interpolate between 12.5 Hz and 1.5625 Hz.
  const hi = r[7];
  const lo = r[10];
  const f = odr === 8 ? 1 / 3 : 2 / 3;
  return Math.round(hi * Math.pow(lo / hi, f));
}

const sim: TileSim<State> = {
  tile: 'Sense.M.3G',

  // After init with cfg = NULL: 25 Hz, 4x averaging, XYZ, data-ready mapped
  // into INT_STATUS with the pin off (active low, push-pull, PULSED), device
  // in suspend, tile SLEEPING, driver cache zeroed. World: the field and die
  // temperature measured on the bring-up bench (driver commit d7f1cc0).
  defaultState: {
    field_x_ut: 42.4,
    field_y_ut: 32.5,
    field_z_ut: 55.8,
    die_temp_c: 29.6,

    conv_x_nt: 0,
    conv_y_nt: 0,
    conv_z_nt: 0,
    conv_temp_mc: 0,
    drdy: 0,
    last_conv_t: 0,
    last_tick_t: 0,
    sensor_clock: 0,
    sensortime: 0,

    mag_x_nt: 0,
    mag_y_nt: 0,
    mag_z_nt: 0,
    temp_mc: 0,

    mode: 0,
    tile_ready: 0,
    odr: ODR_25HZ,
    avg: AVG_4,
    axis_en: EN_XYZ,
    int_output_en: 0,
    int_active_high: 0,
    int_push_pull: 1,
    int_latched: 0,
    pad_drive: 7,
    i2c_wdt: 0,
    sensortime_aon: 0,
    last_pmu_cmd: CMD_UPD_OAE,
    fm_primed: 0,
  },

  controls: [
    {
      type: 'slider',
      field: 'die_temp_c',
      label: 'Die temperature',
      min: -40,
      max: 85,
      step: 0.5,
      unit: '°C',
    },
  ],

  // Where the field points. Earth's field is 25-65 µT; a magnet nearby goes
  // far beyond it, up to the ±2000 µT range where the output saturates.
  stimuli: [
    {
      id: 'field',
      label: 'Magnetic field',
      controls: (['x', 'y', 'z'] as const).map((a) => ({
        kind: 'slider' as const,
        id: `field_${a}`,
        label: `B${a}`,
        field: `field_${a}_ut`,
        min: -150,
        max: 150,
        step: 0.5,
        unit: 'µT',
      })),
    },
    {
      id: 'magnet',
      label: 'Magnet nearby',
      controls: (['x', 'y', 'z'] as const).map((a) => ({
        kind: 'slider' as const,
        id: `magnet_${a}`,
        label: `B${a}`,
        field: `field_${a}_ut`,
        min: -2500,
        max: 2500,
        step: 10,
        unit: 'µT',
      })),
    },
  ],

  hostCalls: {
    tile_sense_m_3g_find: () => ({ scalar: 1 }),
    tile_sense_m_3g_init: () => ({ scalar: 0 }),

    // ── data ──
    // Works in READY or SLEEPING; copies whatever the data registers hold.
    tile_sense_m_3g_read: ({ state }) => ({
      scalar: 1,
      nextState: readIntoCache(state),
    }),
    // Only in READY. Polled (Studio's init has no INT pin): reads INT_STATUS,
    // then read() on a new sample. The callback is not modeled.
    tile_sense_m_3g_process: ({ state }) => {
      if (state.tile_ready !== 1 || !state.drdy) return;
      return { nextState: { drdy: 0, ...readIntoCache(state) } };
    },
    tile_sense_m_3g_on_data: () => undefined,
    tile_sense_m_3g_get_x_nt: ({ state }) => ({ scalar: state.mag_x_nt }),
    tile_sense_m_3g_get_y_nt: ({ state }) => ({ scalar: state.mag_y_nt }),
    tile_sense_m_3g_get_z_nt: ({ state }) => ({ scalar: state.mag_z_nt }),
    tile_sense_m_3g_get_temperature_mc: ({ state }) => ({
      scalar: state.temp_mc,
    }),
    tile_sense_m_3g_get_magnitude_nt: ({ state }) => ({
      scalar: Math.floor(
        Math.sqrt(state.mag_x_nt ** 2 + state.mag_y_nt ** 2 + state.mag_z_nt ** 2),
      ),
    }),
    // Reading INT_STATUS clears it.
    tile_sense_m_3g_data_ready: ({ state }) => ({
      scalar: state.drdy ? 1 : 0,
      nextState: { drdy: 0 },
    }),
    tile_sense_m_3g_get_sensortime: ({ state }) => ({
      scalar: state.sensortime,
    }),

    // ── lifecycle ──
    tile_sense_m_3g_set_mode: ({ state, args }) => ({
      nextState: enterMode(state, arg(args, 0, CMD_SUS)),
    }),
    // FM for the first trigger after bring-up and below 25 Hz, FM_FAST
    // otherwise (§5.1.4); one conversion, then the device is back in suspend
    // (a normal-mode stream ends).
    tile_sense_m_3g_trigger_measurement: ({ state }) => ({
      scalar: 1,
      nextState: {
        ...enterMode(state, state.fm_primed && state.odr <= ODR_25HZ ? CMD_FM_FAST : CMD_FM),
        fm_primed: 1,
      },
    }),
    tile_sense_m_3g_sleep: ({ state }) => ({
      nextState: enterMode(state, CMD_SUS),
    }),
    tile_sense_m_3g_wake: ({ state }) => ({
      nextState: enterMode(state, CMD_NM),
    }),
    // Soft reset + bring_up: registers to POR (100 Hz / 2x, INT_CTRL 0), the
    // driver's odr/avg/axis cache kept, cached sample zeroed, suspend.
    tile_sense_m_3g_reset: () => ({
      nextState: {
        mode: 0,
        tile_ready: 0,
        mag_x_nt: 0,
        mag_y_nt: 0,
        mag_z_nt: 0,
        temp_mc: 0,
        conv_x_nt: 0,
        conv_y_nt: 0,
        conv_z_nt: 0,
        conv_temp_mc: 0,
        drdy: 0,
        sensor_clock: 0,
        sensortime: 0,
        int_output_en: 0,
        int_active_high: 0,
        int_push_pull: 0,
        int_latched: 0,
        pad_drive: 7,
        i2c_wdt: 0,
        sensortime_aon: 0,
        last_pmu_cmd: CMD_FGR,
        fm_primed: 0,
      },
    }),
    // Bit reset then flip-gain reset from suspend; normal mode is restored.
    tile_sense_m_3g_magnetic_reset: () => ({
      scalar: 1,
      nextState: { last_pmu_cmd: CMD_FGR },
    }),

    // ── configuration ──
    tile_sense_m_3g_set_odr_averaging: ({ state, args }) => {
      const odr = arg(args, 0, state.odr) & 0x0f;
      return {
        nextState: {
          odr,
          avg: clampAvg(odr, arg(args, 1, state.avg)),
          last_pmu_cmd: CMD_UPD_OAE,
        },
      };
    },
    tile_sense_m_3g_set_axes: ({ state, args }) => ({
      nextState: { axis_en: arg(args, 0, state.axis_en) & EN_XYZ },
    }),
    tile_sense_m_3g_configure_interrupt: ({ args }) => ({
      nextState: {
        int_output_en: arg(args, 0, 0) ? 1 : 0,
        int_active_high: arg(args, 1, 0) ? 1 : 0,
        int_push_pull: arg(args, 2, 0) ? 1 : 0,
        int_latched: arg(args, 3, 0) ? 1 : 0,
      },
    }),
    tile_sense_m_3g_set_pad_drive: ({ state, args }) => ({
      nextState: { pad_drive: arg(args, 0, state.pad_drive) & 0x07 },
    }),
    tile_sense_m_3g_set_i2c_watchdog: ({ args }) => ({
      nextState: {
        i2c_wdt: (arg(args, 0, 0) ? 1 : 0) | (arg(args, 1, 0) ? 2 : 0),
      },
    }),
    tile_sense_m_3g_set_sensortime_always_on: ({ args }) => ({
      nextState: { sensortime_aon: arg(args, 0, 0) ? 1 : 0 },
    }),

    // ── diagnostics ──
    tile_sense_m_3g_get_error: () => ({ scalar: 0 }),
    // bit 3 normal mode; [7:5] last command (SensorAPI layout).
    tile_sense_m_3g_get_pmu_status: ({ state }) => ({
      scalar: ((state.last_pmu_cmd & 0x07) << 5) | (state.mode === 1 ? 0x08 : 0),
    }),
    // The twin is an ideal part: its compensated output IS the field, which is
    // what all-zero trim words produce (zero offset, sensitivity, TCO, TCS,
    // cross-axis; t0 = 23 °C). Words outside the trim map are per-part IDs.
    tile_sense_m_3g_get_otp_word: () => ({ scalar: 0 }),
    tile_sense_m_3g_get_raw: ({ state, args }) => {
      const axis = arg(args, 0, 0);
      if (axis === 0) return { scalar: Math.round(state.mag_x_nt / NT_PER_LSB_XY) };
      if (axis === 1) return { scalar: Math.round(state.mag_y_nt / NT_PER_LSB_XY) };
      if (axis === 2) return { scalar: Math.round(state.mag_z_nt / NT_PER_LSB_Z) };
      if (axis === 3)
        return {
          scalar: Math.round((state.temp_mc + TEMP_RAW_OFFSET_MC) / MC_PER_LSB_T),
        };
      return { scalar: 0 };
    },
    tile_sense_m_3g_read_reg: ({ state, args }) => {
      switch (arg(args, 0, 0) & 0x7f) {
        case 0x00:
          return { scalar: 0x33 }; // CHIP_ID
        case 0x03:
          return { scalar: state.pad_drive };
        case 0x04:
          return { scalar: ((state.avg & 3) << 4) | (state.odr & 0x0f) };
        case 0x05:
          return { scalar: state.axis_en };
        case 0x07:
          return {
            scalar: ((state.last_pmu_cmd & 0x07) << 5) | (state.mode === 1 ? 0x08 : 0),
          };
        case 0x0a:
          return { scalar: state.i2c_wdt };
        case 0x2e:
          return {
            scalar:
              0x80 |
              (state.int_output_en << 3) |
              (state.int_push_pull << 2) |
              (state.int_active_high << 1) |
              state.int_latched,
          };
        case 0x30:
          return { scalar: state.drdy ? 0x04 : 0, nextState: { drdy: 0 } };
        case 0x61:
          return { scalar: state.sensortime_aon };
        default:
          return { scalar: 0 };
      }
    },
    // Only the registers the twin tracks take effect; others are accepted and ignored.
    tile_sense_m_3g_write_reg: ({ state, args }) => {
      const v = arg(args, 1, 0) & 0xff;
      switch (arg(args, 0, 0) & 0x7f) {
        case 0x03:
          return { nextState: { pad_drive: v & 0x07 } };
        case 0x05:
          return { nextState: { axis_en: v & EN_XYZ } };
        case 0x06:
          return { nextState: enterMode(state, v & 0x0f) };
        case 0x0a:
          return { nextState: { i2c_wdt: v & 0x03 } };
        case 0x2e:
          return {
            nextState: {
              int_output_en: (v >> 3) & 1,
              int_push_pull: (v >> 2) & 1,
              int_active_high: (v >> 1) & 1,
              int_latched: v & 1,
            },
          };
        case 0x61:
          return { nextState: { sensortime_aon: v & 1 } };
        default:
          return;
      }
    },
  },

  provenance: {
    tile_sense_m_3g_find: 'canonical', // CHIP_ID 0x33 at 0x14 (Table 8)
    tile_sense_m_3g_read: 'inferred', // ideal compensated field, no noise / trim
    tile_sense_m_3g_process: 'inferred',
    tile_sense_m_3g_on_data: 'inferred',
    tile_sense_m_3g_get_x_nt: 'inferred',
    tile_sense_m_3g_get_y_nt: 'inferred',
    tile_sense_m_3g_get_z_nt: 'inferred',
    tile_sense_m_3g_get_temperature_mc: 'inferred',
    tile_sense_m_3g_get_magnitude_nt: 'canonical', // integer sqrt of the cached axes
    tile_sense_m_3g_data_ready: 'canonical', // INT_STATUS bit 2, clear on read, pulsed 1.25 ms (§5.5, §8.13)
    tile_sense_m_3g_get_sensortime: 'canonical', // 40 µs ticks, latched per conversion (§5.2.2)
    tile_sense_m_3g_set_mode: 'canonical', // forced only from suspend, returns to suspend (§5.1.4)
    tile_sense_m_3g_trigger_measurement: 'canonical', // first FM, then FM_FAST at ≥ 25 Hz (§5.1.4)
    tile_sense_m_3g_sleep: 'inferred',
    tile_sense_m_3g_wake: 'inferred',
    tile_sense_m_3g_reset: 'inferred',
    tile_sense_m_3g_magnetic_reset: 'inferred', // always acknowledged
    tile_sense_m_3g_set_odr_averaging: 'canonical', // Table 5 clamp
    tile_sense_m_3g_set_axes: 'canonical',
    tile_sense_m_3g_configure_interrupt: 'canonical', // INT_CTRL bits (§8.11)
    tile_sense_m_3g_set_pad_drive: 'canonical',
    tile_sense_m_3g_set_i2c_watchdog: 'canonical',
    tile_sense_m_3g_set_sensortime_always_on: 'canonical',
    tile_sense_m_3g_get_error: 'inferred', // never reports an error
    tile_sense_m_3g_get_pmu_status: 'inferred', // [7:5] per SensorAPI, reserved in the datasheet
    tile_sense_m_3g_get_otp_word: 'inferred', // an ideal part's trim: zeros
    tile_sense_m_3g_get_raw: 'inferred', // inverse of the driver scaling, no trim
    tile_sense_m_3g_read_reg: 'inferred', // a handful of registers modeled
    tile_sense_m_3g_write_reg: 'inferred',
    power: 'inferred', // Table 5 ("approximate") / Table 1 currents, 6.25 and 3.125 Hz interpolated, plus the LDO's quiescent current
  },

  // Normal mode converts at the ODR. The sensor-time counter runs in normal
  // mode (or always, with sensortime_aon). In pulsed mode an unread drdy
  // expires 1.25 ms after its conversion, so it is gone by the next tick.
  deriveState(state, { t }) {
    const out: Partial<State> = { last_tick_t: t };
    const dt = Math.max(0, t - state.last_tick_t);
    if (state.mode === 1 || state.sensortime_aon === 1)
      out.sensor_clock = (state.sensor_clock + Math.round(dt * SENSORTIME_TICKS_PER_MS)) >>> 0;
    if (!state.int_latched && state.drdy && t - state.last_conv_t > PULSE_MS) out.drdy = 0;
    if (state.mode === 1) {
      const hz = ODR_HZ[state.odr] ?? 25;
      if (t - state.last_conv_t >= 1000 / hz) {
        Object.assign(
          out,
          convert({
            ...state,
            sensor_clock: out.sensor_clock ?? state.sensor_clock,
          }),
        );
        out.last_conv_t = t;
      }
    }
    return out;
  },

  // INT (pad 9), shown 1 = asserted: the data-ready pulse, when the pin is
  // enabled. Pulsed mode clears it 1.25 ms after the conversion (deriveState).
  padOutputs(state) {
    return { '9': state.int_output_en && state.drdy ? 1 : 0 };
  },

  // V+ (pad 10) feeds VDDIO directly and VDD through the 1.8 V LDO, so the
  // pad carries the BMM350's current plus the LDO's quiescent current.
  power(state, ctx?: PowerCtx) {
    const normal = state.mode === 1;
    const chip = normal ? normalCurrentUa(state.odr, state.avg) : I_SUSPEND_UA;
    const ua = chip + LDO_IQ_UA;
    return {
      draw_ua: ua,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: ctx?.padVoltage?.['10'] ?? 3300,
          i_ua: ua,
          pads: ['10'],
          note: normal
            ? `normal mode, ${ODR_HZ[state.odr] ?? '?'} Hz, ${1 << state.avg}x averaging (Table 5), + LDO`
            : 'suspend (1.8 µA) + LDO',
        },
      ],
    };
  },
};

export default sim;
