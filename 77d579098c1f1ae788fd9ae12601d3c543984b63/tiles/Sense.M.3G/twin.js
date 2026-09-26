// src/sims/sense_m_3g.ts
var CMD_SUS = 0;
var CMD_NM = 1;
var CMD_UPD_OAE = 2;
var CMD_FM = 3;
var CMD_FM_FAST = 4;
var CMD_FGR = 5;
var ODR_HZ = {
  2: 400,
  3: 200,
  4: 100,
  5: 50,
  6: 25,
  7: 12.5,
  8: 6.25,
  9: 3.125,
  10: 1.5625
};
var ODR_25HZ = 6;
var AVG_4 = 2;
var EN_XYZ = 7;
var FULL_SCALE_NT = 2e6;
var SENSORTIME_TICKS_PER_MS = 25;
var PULSE_MS = 1.25;
var NT_PER_LSB_XY = 463338 / 65536;
var NT_PER_LSB_Z = 470218 / 65536;
var MC_PER_LSB_T = 64309 / 65536;
var TEMP_RAW_OFFSET_MC = 25490;
function clampAvg(odr, avg) {
  if (odr === 2) return Math.min(avg, 0);
  if (odr === 3) return Math.min(avg, 1);
  if (odr === 4) return Math.min(avg, 2);
  return avg & 3;
}
var sat = (nt) => Math.max(-FULL_SCALE_NT, Math.min(FULL_SCALE_NT, Math.round(nt)));
function convert(s) {
  return {
    conv_x_nt: s.axis_en & 1 ? sat(s.field_x_ut * 1e3) : 0,
    conv_y_nt: s.axis_en & 2 ? sat(s.field_y_ut * 1e3) : 0,
    conv_z_nt: s.axis_en & 4 ? sat(s.field_z_ut * 1e3) : 0,
    conv_temp_mc: Math.round(s.die_temp_c * 1e3),
    drdy: 1,
    sensortime: s.sensor_clock & 16777215
  };
}
var readIntoCache = (s) => ({
  mag_x_nt: s.conv_x_nt,
  mag_y_nt: s.conv_y_nt,
  mag_z_nt: s.conv_z_nt,
  temp_mc: s.conv_temp_mc
});
function enterMode(s, mode) {
  const tile_ready = mode === CMD_SUS ? 0 : 1;
  if (mode === CMD_NM) return { mode: 1, tile_ready, last_pmu_cmd: CMD_NM };
  if (mode === CMD_FM || mode === CMD_FM_FAST)
    return { ...convert(s), mode: 0, tile_ready, last_pmu_cmd: mode };
  return { mode: 0, tile_ready, last_pmu_cmd: CMD_SUS };
}
var arg = (args, i, fallback) => args.length > i && Number.isFinite(args[i]) ? args[i] : fallback;
var TABLE5 = {
  0: { 2: 455, 3: 235, 4: 122, 5: 70, 6: 40, 7: 23, 10: 12 },
  1: { 3: 370, 4: 190, 5: 100, 6: 55, 7: 33, 10: 15 },
  2: { 4: 335, 5: 175, 6: 96, 7: 57, 10: 25 },
  3: { 5: 325, 6: 180, 7: 108, 10: 50 }
};
var I_SUSPEND_UA = 1.8;
var LDO_IQ_UA = 6.5;
function normalCurrentUa(odr, avg) {
  const r = TABLE5[clampAvg(odr, avg & 3)];
  if (r[odr] != null) return r[odr];
  const hi = r[7];
  const lo = r[10];
  const f = odr === 8 ? 1 / 3 : 2 / 3;
  return Math.round(hi * Math.pow(lo / hi, f));
}
var sim = {
  tile: "Sense.M.3G",
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
    fm_primed: 0
  },
  controls: [
    {
      type: "slider",
      field: "die_temp_c",
      label: "Die temperature",
      min: -40,
      max: 85,
      step: 0.5,
      unit: "\xB0C"
    }
  ],
  // Where the field points. Earth's field is 25-65 µT; a magnet nearby goes
  // far beyond it, up to the ±2000 µT range where the output saturates.
  stimuli: [
    {
      id: "field",
      label: "Magnetic field",
      controls: ["x", "y", "z"].map((a) => ({
        kind: "slider",
        id: `field_${a}`,
        label: `B${a}`,
        field: `field_${a}_ut`,
        min: -150,
        max: 150,
        step: 0.5,
        unit: "\xB5T"
      }))
    },
    {
      id: "magnet",
      label: "Magnet nearby",
      controls: ["x", "y", "z"].map((a) => ({
        kind: "slider",
        id: `magnet_${a}`,
        label: `B${a}`,
        field: `field_${a}_ut`,
        min: -2500,
        max: 2500,
        step: 10,
        unit: "\xB5T"
      }))
    }
  ],
  hostCalls: {
    tile_sense_m_3g_find: () => ({ scalar: 1 }),
    tile_sense_m_3g_init: () => ({ scalar: 0 }),
    // ── data ──
    // Works in READY or SLEEPING; copies whatever the data registers hold.
    tile_sense_m_3g_read: ({ state }) => ({
      scalar: 1,
      nextState: readIntoCache(state)
    }),
    // Only in READY. Polled (Studio's init has no INT pin): reads INT_STATUS,
    // then read() on a new sample. The callback is not modeled.
    tile_sense_m_3g_process: ({ state }) => {
      if (state.tile_ready !== 1 || !state.drdy) return;
      return { nextState: { drdy: 0, ...readIntoCache(state) } };
    },
    tile_sense_m_3g_on_data: () => void 0,
    tile_sense_m_3g_get_x_nt: ({ state }) => ({ scalar: state.mag_x_nt }),
    tile_sense_m_3g_get_y_nt: ({ state }) => ({ scalar: state.mag_y_nt }),
    tile_sense_m_3g_get_z_nt: ({ state }) => ({ scalar: state.mag_z_nt }),
    tile_sense_m_3g_get_temperature_mc: ({ state }) => ({
      scalar: state.temp_mc
    }),
    tile_sense_m_3g_get_magnitude_nt: ({ state }) => ({
      scalar: Math.floor(
        Math.sqrt(state.mag_x_nt ** 2 + state.mag_y_nt ** 2 + state.mag_z_nt ** 2)
      )
    }),
    // Reading INT_STATUS clears it.
    tile_sense_m_3g_data_ready: ({ state }) => ({
      scalar: state.drdy ? 1 : 0,
      nextState: { drdy: 0 }
    }),
    tile_sense_m_3g_get_sensortime: ({ state }) => ({
      scalar: state.sensortime
    }),
    // ── lifecycle ──
    tile_sense_m_3g_set_mode: ({ state, args }) => ({
      nextState: enterMode(state, arg(args, 0, CMD_SUS))
    }),
    // FM for the first trigger after bring-up and below 25 Hz, FM_FAST
    // otherwise (§5.1.4); one conversion, then the device is back in suspend
    // (a normal-mode stream ends).
    tile_sense_m_3g_trigger_measurement: ({ state }) => ({
      scalar: 1,
      nextState: {
        ...enterMode(state, state.fm_primed && state.odr <= ODR_25HZ ? CMD_FM_FAST : CMD_FM),
        fm_primed: 1
      }
    }),
    tile_sense_m_3g_sleep: ({ state }) => ({
      nextState: enterMode(state, CMD_SUS)
    }),
    tile_sense_m_3g_wake: ({ state }) => ({
      nextState: enterMode(state, CMD_NM)
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
        fm_primed: 0
      }
    }),
    // Bit reset then flip-gain reset from suspend; normal mode is restored.
    tile_sense_m_3g_magnetic_reset: () => ({
      scalar: 1,
      nextState: { last_pmu_cmd: CMD_FGR }
    }),
    // ── configuration ──
    tile_sense_m_3g_set_odr_averaging: ({ state, args }) => {
      const odr = arg(args, 0, state.odr) & 15;
      return {
        nextState: {
          odr,
          avg: clampAvg(odr, arg(args, 1, state.avg)),
          last_pmu_cmd: CMD_UPD_OAE
        }
      };
    },
    tile_sense_m_3g_set_axes: ({ state, args }) => ({
      nextState: { axis_en: arg(args, 0, state.axis_en) & EN_XYZ }
    }),
    tile_sense_m_3g_configure_interrupt: ({ args }) => ({
      nextState: {
        int_output_en: arg(args, 0, 0) ? 1 : 0,
        int_active_high: arg(args, 1, 0) ? 1 : 0,
        int_push_pull: arg(args, 2, 0) ? 1 : 0,
        int_latched: arg(args, 3, 0) ? 1 : 0
      }
    }),
    tile_sense_m_3g_set_pad_drive: ({ state, args }) => ({
      nextState: { pad_drive: arg(args, 0, state.pad_drive) & 7 }
    }),
    tile_sense_m_3g_set_i2c_watchdog: ({ args }) => ({
      nextState: {
        i2c_wdt: (arg(args, 0, 0) ? 1 : 0) | (arg(args, 1, 0) ? 2 : 0)
      }
    }),
    tile_sense_m_3g_set_sensortime_always_on: ({ args }) => ({
      nextState: { sensortime_aon: arg(args, 0, 0) ? 1 : 0 }
    }),
    // ── diagnostics ──
    tile_sense_m_3g_get_error: () => ({ scalar: 0 }),
    // bit 3 normal mode; [7:5] last command (SensorAPI layout).
    tile_sense_m_3g_get_pmu_status: ({ state }) => ({
      scalar: (state.last_pmu_cmd & 7) << 5 | (state.mode === 1 ? 8 : 0)
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
          scalar: Math.round((state.temp_mc + TEMP_RAW_OFFSET_MC) / MC_PER_LSB_T)
        };
      return { scalar: 0 };
    },
    tile_sense_m_3g_read_reg: ({ state, args }) => {
      switch (arg(args, 0, 0) & 127) {
        case 0:
          return { scalar: 51 };
        // CHIP_ID
        case 3:
          return { scalar: state.pad_drive };
        case 4:
          return { scalar: (state.avg & 3) << 4 | state.odr & 15 };
        case 5:
          return { scalar: state.axis_en };
        case 7:
          return {
            scalar: (state.last_pmu_cmd & 7) << 5 | (state.mode === 1 ? 8 : 0)
          };
        case 10:
          return { scalar: state.i2c_wdt };
        case 46:
          return {
            scalar: 128 | state.int_output_en << 3 | state.int_push_pull << 2 | state.int_active_high << 1 | state.int_latched
          };
        case 48:
          return { scalar: state.drdy ? 4 : 0, nextState: { drdy: 0 } };
        case 97:
          return { scalar: state.sensortime_aon };
        default:
          return { scalar: 0 };
      }
    },
    // Only the registers the twin tracks take effect; others are accepted and ignored.
    tile_sense_m_3g_write_reg: ({ state, args }) => {
      const v = arg(args, 1, 0) & 255;
      switch (arg(args, 0, 0) & 127) {
        case 3:
          return { nextState: { pad_drive: v & 7 } };
        case 5:
          return { nextState: { axis_en: v & EN_XYZ } };
        case 6:
          return { nextState: enterMode(state, v & 15) };
        case 10:
          return { nextState: { i2c_wdt: v & 3 } };
        case 46:
          return {
            nextState: {
              int_output_en: v >> 3 & 1,
              int_push_pull: v >> 2 & 1,
              int_active_high: v >> 1 & 1,
              int_latched: v & 1
            }
          };
        case 97:
          return { nextState: { sensortime_aon: v & 1 } };
        default:
          return;
      }
    }
  },
  provenance: {
    tile_sense_m_3g_find: "canonical",
    // CHIP_ID 0x33 at 0x14 (Table 8)
    tile_sense_m_3g_read: "inferred",
    // ideal compensated field, no noise / trim
    tile_sense_m_3g_process: "inferred",
    tile_sense_m_3g_on_data: "inferred",
    tile_sense_m_3g_get_x_nt: "inferred",
    tile_sense_m_3g_get_y_nt: "inferred",
    tile_sense_m_3g_get_z_nt: "inferred",
    tile_sense_m_3g_get_temperature_mc: "inferred",
    tile_sense_m_3g_get_magnitude_nt: "canonical",
    // integer sqrt of the cached axes
    tile_sense_m_3g_data_ready: "canonical",
    // INT_STATUS bit 2, clear on read, pulsed 1.25 ms (§5.5, §8.13)
    tile_sense_m_3g_get_sensortime: "canonical",
    // 40 µs ticks, latched per conversion (§5.2.2)
    tile_sense_m_3g_set_mode: "canonical",
    // forced only from suspend, returns to suspend (§5.1.4)
    tile_sense_m_3g_trigger_measurement: "canonical",
    // first FM, then FM_FAST at ≥ 25 Hz (§5.1.4)
    tile_sense_m_3g_sleep: "inferred",
    tile_sense_m_3g_wake: "inferred",
    tile_sense_m_3g_reset: "inferred",
    tile_sense_m_3g_magnetic_reset: "inferred",
    // always acknowledged
    tile_sense_m_3g_set_odr_averaging: "canonical",
    // Table 5 clamp
    tile_sense_m_3g_set_axes: "canonical",
    tile_sense_m_3g_configure_interrupt: "canonical",
    // INT_CTRL bits (§8.11)
    tile_sense_m_3g_set_pad_drive: "canonical",
    tile_sense_m_3g_set_i2c_watchdog: "canonical",
    tile_sense_m_3g_set_sensortime_always_on: "canonical",
    tile_sense_m_3g_get_error: "inferred",
    // never reports an error
    tile_sense_m_3g_get_pmu_status: "inferred",
    // [7:5] per SensorAPI, reserved in the datasheet
    tile_sense_m_3g_get_otp_word: "inferred",
    // an ideal part's trim: zeros
    tile_sense_m_3g_get_raw: "inferred",
    // inverse of the driver scaling, no trim
    tile_sense_m_3g_read_reg: "inferred",
    // a handful of registers modeled
    tile_sense_m_3g_write_reg: "inferred",
    power: "inferred"
    // Table 5 ("approximate") / Table 1 currents, 6.25 and 3.125 Hz interpolated, plus the LDO's quiescent current
  },
  // Normal mode converts at the ODR. The sensor-time counter runs in normal
  // mode (or always, with sensortime_aon). In pulsed mode an unread drdy
  // expires 1.25 ms after its conversion, so it is gone by the next tick.
  deriveState(state, { t }) {
    const out = { last_tick_t: t };
    const dt = Math.max(0, t - state.last_tick_t);
    if (state.mode === 1 || state.sensortime_aon === 1)
      out.sensor_clock = state.sensor_clock + Math.round(dt * SENSORTIME_TICKS_PER_MS) >>> 0;
    if (!state.int_latched && state.drdy && t - state.last_conv_t > PULSE_MS) out.drdy = 0;
    if (state.mode === 1) {
      const hz = ODR_HZ[state.odr] ?? 25;
      if (t - state.last_conv_t >= 1e3 / hz) {
        Object.assign(
          out,
          convert({
            ...state,
            sensor_clock: out.sensor_clock ?? state.sensor_clock
          })
        );
        out.last_conv_t = t;
      }
    }
    return out;
  },
  // INT (pad 9), shown 1 = asserted: the data-ready pulse, when the pin is
  // enabled. Pulsed mode clears it 1.25 ms after the conversion (deriveState).
  padOutputs(state) {
    return { "9": state.int_output_en && state.drdy ? 1 : 0 };
  },
  // V+ (pad 10) feeds VDDIO directly and VDD through the 1.8 V LDO, so the
  // pad carries the BMM350's current plus the LDO's quiescent current.
  power(state, ctx) {
    const normal = state.mode === 1;
    const chip = normal ? normalCurrentUa(state.odr, state.avg) : I_SUSPEND_UA;
    const ua = chip + LDO_IQ_UA;
    return {
      draw_ua: ua,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: ctx?.padVoltage?.["10"] ?? 3300,
          i_ua: ua,
          pads: ["10"],
          note: normal ? `normal mode, ${ODR_HZ[state.odr] ?? "?"} Hz, ${1 << state.avg}x averaging (Table 5), + LDO` : "suspend (1.8 \xB5A) + LDO"
        }
      ]
    };
  }
};
var sense_m_3g_default = sim;
export {
  ODR_HZ,
  clampAvg,
  sense_m_3g_default as default,
  normalCurrentUa
};
