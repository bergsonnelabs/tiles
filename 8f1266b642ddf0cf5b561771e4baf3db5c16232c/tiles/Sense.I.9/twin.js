// src/sims/sense_i_9.ts
var INT_WOM = 1 << 3;
var RAW_DATA_RDY = 1 << 0;
var FIFO0 = 1 << 0;
var BYPASS_EN = 2;
var FIFO_BYTES = 512;
var FIFO_SNAPSHOT = 31;
var ACCEL_LSB_PER_G = {
  0: 16384,
  2: 8192,
  4: 4096,
  6: 2048
};
var GYRO_LSB_PER_DPS = {
  0: 131,
  2: 65.5,
  4: 32.8,
  6: 16.4
};
var MAG_UT_PER_LSB = 0.15;
var MAG_MAX = 32752;
var MAG_HZ = {
  2: 10,
  4: 20,
  6: 50,
  8: 100
};
var LSB_PER_G_2G = 16384;
var FACE_Z_MIN = 13926;
var FACE_XY_MAX = 8192;
function atan2Centi(y, x) {
  if (x === 0 && y === 0) return 0;
  const ay = Math.abs(y);
  const ax = Math.abs(x);
  const core = (t) => {
    const corr = Math.trunc((t - 1e3) * (14e3 + 4 * t) / 1e3);
    const a = Math.trunc((t * 4500 - Math.trunc(t * corr / 10)) / 1e3);
    return Math.max(0, Math.min(4500, a));
  };
  let angle = ax >= ay ? core(Math.trunc(ay * 1e3 / ax)) : 9e3 - core(Math.trunc(ax * 1e3 / ay));
  if (x < 0) angle = 18e3 - angle;
  if (y < 0) angle = -angle;
  return angle;
}
var clampI16 = (v) => {
  const r = Math.round(v);
  return r > 32767 ? 32767 : r < -32768 ? -32768 : r;
};
var accelLsb = (s) => ACCEL_LSB_PER_G[s.accel_range & 6] ?? 16384;
var gyroLsb = (s) => GYRO_LSB_PER_DPS[s.gyro_range & 6] ?? 131;
var accelRaw = (s) => {
  const k = accelLsb(s) / 1e3;
  return [clampI16(s.accel_x_mg * k), clampI16(s.accel_y_mg * k), clampI16(s.accel_z_mg * k)];
};
var gyroRaw = (s) => {
  const k = gyroLsb(s);
  return [clampI16(s.gyro_x_dps * k), clampI16(s.gyro_y_dps * k), clampI16(s.gyro_z_dps * k)];
};
var magCount = (ut) => {
  const r = Math.round(ut / MAG_UT_PER_LSB);
  return r > MAG_MAX ? MAG_MAX : r < -MAG_MAX ? -MAG_MAX : r;
};
var magReachable = (s) => (s.int_pin_cfg & BYPASS_EN) !== 0;
var magMeasuring = (s) => s.mag_mode !== 0 && !s.sleeping;
var magOverflow = (s) => Math.abs(s.mag_x_ut) + Math.abs(s.mag_y_ut) + Math.abs(s.mag_z_ut) > 4912;
var sampleHz = (s) => s.sleeping ? 0 : Math.max(1125 / (1 + s.accel_divider), 1125 / (1 + s.gyro_divider));
var fifoSampleBytes = (s) => (s.fifo_accel_en ? 6 : 0) + (s.fifo_gyro_en ? 6 : 0) + (s.fifo_temp_en ? 2 : 0);
var magUa = (s) => (MAG_HZ[s.mag_mode] ?? 0) * 90 / 8;
var sim = {
  tile: "Sense.I.9",
  defaultState: {
    accel_x_mg: 0,
    accel_y_mg: 0,
    accel_z_mg: 1e3,
    // resting flat, face up
    gyro_x_dps: 0,
    gyro_y_dps: 0,
    gyro_z_dps: 0,
    mag_x_ut: 25,
    // ~Earth's field, horizontal
    mag_y_ut: 0,
    mag_z_ut: -45,
    // ~Earth's field, vertical
    temperature_c: 22,
    fault_inject: 0,
    accel_range: 0,
    // ±2 g (ACCEL_CONFIG reset 0x01; init doesn't change it)
    gyro_range: 0,
    // ±250 dps (GYRO_CONFIG_1 reset 0x01)
    mag_mode: 8,
    // continuous 100 Hz (init)
    accel_divider: 0,
    gyro_divider: 0,
    sleeping: 0,
    int_pin_cfg: 2,
    // init: BYPASS_EN, active-high, push-pull, pulsed
    int_dry_en: 0,
    int_wom_en: 0,
    int_fifo_ovf_en: 0,
    int_fifo_wm_en: 0,
    int_status: 0,
    int_status_1: 0,
    int_status_fifo_ovf: 0,
    int_status_fifo_wm: 0,
    last_drdy_ms: 0,
    wom_global_en: 0,
    wom_threshold_mg: 0,
    // ACCEL_WOM_THR reset 0x00
    wom_mode: 0,
    wom_ref_x_mg: 0,
    wom_ref_y_mg: 0,
    wom_ref_z_mg: 1e3,
    prev_wom_x: 0,
    prev_wom_y: 0,
    prev_wom_z: 0,
    fifo_mode: 0,
    fifo_accel_en: 0,
    fifo_gyro_en: 0,
    fifo_temp_en: 0,
    fifo_bytes: 0,
    last_fifo_ms: -1
  },
  controls: [
    {
      type: "slider",
      field: "accel_x_mg",
      label: "Accel X",
      min: -16e3,
      max: 16e3,
      step: 50,
      unit: "mg"
    },
    {
      type: "slider",
      field: "accel_y_mg",
      label: "Accel Y",
      min: -16e3,
      max: 16e3,
      step: 50,
      unit: "mg"
    },
    {
      type: "slider",
      field: "accel_z_mg",
      label: "Accel Z",
      min: -16e3,
      max: 16e3,
      step: 50,
      unit: "mg"
    },
    {
      type: "slider",
      field: "gyro_x_dps",
      label: "Gyro X",
      min: -2e3,
      max: 2e3,
      step: 5,
      unit: "dps"
    },
    {
      type: "slider",
      field: "gyro_y_dps",
      label: "Gyro Y",
      min: -2e3,
      max: 2e3,
      step: 5,
      unit: "dps"
    },
    {
      type: "slider",
      field: "gyro_z_dps",
      label: "Gyro Z",
      min: -2e3,
      max: 2e3,
      step: 5,
      unit: "dps"
    },
    {
      type: "slider",
      field: "mag_x_ut",
      label: "Mag X",
      min: -4900,
      max: 4900,
      step: 5,
      unit: "\xB5T"
    },
    {
      type: "slider",
      field: "mag_y_ut",
      label: "Mag Y",
      min: -4900,
      max: 4900,
      step: 5,
      unit: "\xB5T"
    },
    {
      type: "slider",
      field: "mag_z_ut",
      label: "Mag Z",
      min: -4900,
      max: 4900,
      step: 5,
      unit: "\xB5T"
    },
    {
      type: "slider",
      field: "temperature_c",
      label: "Temperature",
      min: -40,
      max: 85,
      step: 0.5,
      unit: "\xB0C"
    },
    {
      type: "toggle",
      field: "fault_inject",
      label: "Fault",
      description: "A faulty part: self-tests fail and mag_overflowed returns 1."
    }
  ],
  // What a person does to a 9-axis IMU: tilt it, shake it, turn it, bring a
  // magnet near it, warm it.
  stimuli: [
    {
      id: "orientation",
      label: "Orientation",
      controls: [
        {
          kind: "attitude",
          id: "attitude",
          label: "orientation",
          accel: { x: "accel_x_mg", y: "accel_y_mg", z: "accel_z_mg" }
        }
      ]
    },
    {
      id: "accel",
      label: "Accelerations",
      controls: ["x", "y", "z"].map((a) => ({
        kind: "slider",
        id: `accel_${a}_mg`,
        label: `accel ${a}`,
        field: `accel_${a}_mg`,
        min: -16e3,
        max: 16e3,
        step: 50,
        unit: "mg"
      }))
    },
    {
      id: "shake",
      label: "Shake intensity",
      controls: [
        {
          kind: "shake",
          id: "shake",
          label: "shake",
          max: 2e3,
          unit: "mg",
          accel: { x: "accel_x_mg", y: "accel_y_mg", z: "accel_z_mg" }
        }
      ]
    },
    {
      id: "gyro",
      label: "Gyro rates",
      controls: ["x", "y", "z"].map((a) => ({
        kind: "slider",
        id: `gyro_${a}_dps`,
        label: `gyro ${a}`,
        field: `gyro_${a}_dps`,
        min: -2e3,
        max: 2e3,
        step: 5,
        unit: "\xB0/s"
      }))
    },
    {
      id: "magnetic",
      label: "Magnetic field",
      controls: ["x", "y", "z"].map((a) => ({
        kind: "slider",
        id: `mag_${a}_ut`,
        label: `field ${a}`,
        field: `mag_${a}_ut`,
        min: -4900,
        max: 4900,
        step: 5,
        unit: "\xB5T"
      }))
    },
    {
      id: "temperature",
      label: "Temperature",
      controls: [
        {
          kind: "slider",
          id: "temperature_c",
          label: "temperature",
          field: "temperature_c",
          min: -40,
          max: 85,
          step: 0.5,
          unit: "\xB0C"
        }
      ]
    }
  ],
  hostCalls: {
    // ── lifecycle ──
    tile_sense_i_9_find: () => ({ scalar: 1 }),
    // 1 when the part ACKs (driver .c:141)
    tile_sense_i_9_init: () => ({
      nextState: { sleeping: 0, int_pin_cfg: 2, mag_mode: 8 }
    }),
    tile_sense_i_9_sleep: () => ({ nextState: { sleeping: 1 } }),
    tile_sense_i_9_wake: () => ({
      nextState: { sleeping: 0, last_drdy_ms: 0 }
    }),
    // DEVICE_RESET: ICM registers to reset values (PWR_MGMT_1 = 0x41 → asleep,
    // INT_PIN_CFG = 0x00 → no bypass). The AK09916 is a separate die: its mode
    // is untouched, but it can't be reached until init().
    tile_sense_i_9_reset: () => ({
      nextState: {
        accel_range: 0,
        gyro_range: 0,
        accel_divider: 0,
        gyro_divider: 0,
        sleeping: 1,
        int_pin_cfg: 0,
        int_dry_en: 0,
        int_wom_en: 0,
        int_fifo_ovf_en: 0,
        int_fifo_wm_en: 0,
        int_status: 0,
        int_status_1: 0,
        int_status_fifo_ovf: 0,
        int_status_fifo_wm: 0,
        wom_global_en: 0,
        wom_threshold_mg: 0,
        wom_mode: 0,
        prev_wom_x: 0,
        prev_wom_y: 0,
        prev_wom_z: 0,
        fifo_mode: 0,
        fifo_accel_en: 0,
        fifo_gyro_en: 0,
        fifo_temp_en: 0,
        fifo_bytes: 0,
        last_fifo_ms: -1
      }
    }),
    // ── configuration ──
    // Read-modify-write of FS_SEL [2:1] only; DLPF / FCHOICE are kept.
    tile_sense_i_9_set_accel_range: ({ args }) => ({
      nextState: { accel_range: (args[0] ?? 0) & 6 }
    }),
    tile_sense_i_9_set_gyro_range: ({ args }) => ({
      nextState: { gyro_range: (args[0] ?? 0) & 6 }
    }),
    tile_sense_i_9_set_mag_mode: ({ args }) => ({
      nextState: { mag_mode: (args[0] ?? 0) & 31 }
    }),
    tile_sense_i_9_set_accel_odr: ({ args }) => ({
      nextState: { accel_divider: (args[0] ?? 0) & 4095 }
    }),
    tile_sense_i_9_set_gyro_odr: ({ args }) => ({
      nextState: { gyro_divider: (args[0] ?? 0) & 255 }
    }),
    // ── data reads ──
    // INT_STATUS_1 read (clears it). While awake a fresh sample is always
    // waiting: the slowest ODR (~0.27 Hz) aside, every rate outpaces the 10 Hz tick.
    tile_sense_i_9_data_ready: ({ state }) => ({
      scalar: state.sleeping ? 0 : 1,
      nextState: { int_status_1: 0 }
    }),
    tile_sense_i_9_get_raw_accels: ({ state }) => ({ array: accelRaw(state) }),
    tile_sense_i_9_get_raw_gyros: ({ state }) => ({ array: gyroRaw(state) }),
    tile_sense_i_9_get_raw_6dof: ({ state }) => ({
      array: [...accelRaw(state), ...gyroRaw(state)]
    }),
    // AK09916 HX/HY/HZ. Power-down reads 0; without BYPASS_EN the part doesn't
    // answer and the caller's array is left as it was.
    tile_sense_i_9_get_raw_mags: ({ state }) => {
      if (!magReachable(state)) return void 0;
      if (!magMeasuring(state)) return { array: [0, 0, 0] };
      return {
        array: [magCount(state.mag_x_ut), magCount(state.mag_y_ut), magCount(state.mag_z_ut)]
      };
    },
    tile_sense_i_9_mag_overflowed: ({ state }) => ({
      scalar: magReachable(state) && (state.fault_inject || magMeasuring(state) && magOverflow(state)) ? 1 : 0
    }),
    // TEMP_degC = TEMP_OUT / 333.87 + 21 (RoomTemp_Offset 0).
    tile_sense_i_9_get_temperature: ({ state }) => ({
      scalar: clampI16((state.temperature_c - 21) * 333.87)
    }),
    // ── interrupts ──
    // Flags supply bits 7..4; BYPASS_EN (bit 1) is preserved.
    tile_sense_i_9_int_config: ({ state, args }) => ({
      nextState: {
        int_pin_cfg: (args[0] ?? 0) & 240 | state.int_pin_cfg & BYPASS_EN
      }
    }),
    tile_sense_i_9_int_data_ready: ({ args }) => ({
      nextState: { int_dry_en: args[0] ? 1 : 0 }
    }),
    tile_sense_i_9_int_wom: ({ args }) => ({
      nextState: { int_wom_en: args[0] ? 1 : 0 }
    }),
    tile_sense_i_9_int_fifo_overflow: ({ args }) => ({
      nextState: { int_fifo_ovf_en: args[0] ? 1 : 0 }
    }),
    tile_sense_i_9_int_fifo_watermark: ({ args }) => ({
      nextState: { int_fifo_wm_en: args[0] ? 1 : 0 }
    }),
    tile_sense_i_9_get_int_status: ({ state }) => ({
      scalar: state.int_status,
      nextState: { int_status: 0 }
    }),
    tile_sense_i_9_get_int_status_fifo_overflow: ({ state }) => ({
      scalar: state.int_status_fifo_ovf & 31,
      nextState: { int_status_fifo_ovf: 0 }
    }),
    tile_sense_i_9_get_int_status_fifo_watermark: ({ state }) => ({
      scalar: state.int_status_fifo_wm & 31,
      nextState: { int_status_fifo_wm: 0 }
    }),
    // ── wake-on-motion ──
    // ACCEL_WOM_THR = round(mg / 4), clamped to 0xFF (1020 mg).
    tile_sense_i_9_wom_config: ({ args }) => ({
      nextState: {
        wom_threshold_mg: Math.min(255, Math.trunc(((args[0] ?? 0) + 2) / 4)) * 4,
        wom_mode: args[1] === 1 ? 1 : 0
      }
    }),
    tile_sense_i_9_wom_enable: ({ state }) => ({
      nextState: {
        wom_global_en: 1,
        wom_ref_x_mg: state.accel_x_mg,
        wom_ref_y_mg: state.accel_y_mg,
        wom_ref_z_mg: state.accel_z_mg,
        prev_wom_x: 0,
        prev_wom_y: 0,
        prev_wom_z: 0
      }
    }),
    tile_sense_i_9_wom_disable: () => ({ nextState: { wom_global_en: 0 } }),
    // Polls INT_STATUS (clear-on-read) for WOM. Time can't pass inside one
    // host call, so this is the first poll's answer.
    tile_sense_i_9_wait_for_motion: ({ state }) => ({
      scalar: state.int_status & INT_WOM ? 1 : 0,
      nextState: { int_status: 0 }
    }),
    // ── FIFO ──
    // FIFO_RST asserted + released: the FIFO empties.
    tile_sense_i_9_fifo_config: ({ args }) => ({
      nextState: {
        fifo_mode: (args[0] ?? 0) === FIFO_SNAPSHOT ? FIFO_SNAPSHOT : 0,
        fifo_accel_en: args[1] ? 1 : 0,
        fifo_gyro_en: args[2] ? 1 : 0,
        fifo_temp_en: args[3] ? 1 : 0,
        fifo_bytes: 0,
        last_fifo_ms: -1
      }
    }),
    tile_sense_i_9_fifo_flush: () => ({
      nextState: { fifo_bytes: 0, last_fifo_ms: -1 }
    }),
    tile_sense_i_9_fifo_count: ({ state }) => ({
      scalar: state.fifo_bytes & 8191
    }),
    // Needs ≥ 12 bytes; reads them as [ax ay az gx gy gz] (the accel + gyro
    // layout). Otherwise the caller's array is left as it was.
    tile_sense_i_9_fifo_read_packet_flat: ({ state }) => state.fifo_bytes >= 12 ? {
      array: [...accelRaw(state), ...gyroRaw(state)],
      nextState: { fifo_bytes: state.fifo_bytes - 12 }
    } : void 0,
    // ── self-test ──
    // Per-axis pass bits (bit0 X … bit2 Y/Z). The driver leaves the part at
    // ±2 g / ±250 dps, dividers 0 afterwards (driver .c:551-590).
    tile_sense_i_9_self_test: ({ state }) => {
      const pass = state.fault_inject ? 0 : 7;
      return {
        scalar: state.fault_inject ? 0 : 1,
        outScalars: { accel_pass: pass, gyro_pass: pass },
        nextState: {
          accel_range: 0,
          gyro_range: 0,
          accel_divider: 0,
          gyro_divider: 0
        }
      };
    },
    // Leaves the AK09916 powered down (self-test mode is single-shot).
    tile_sense_i_9_mag_self_test: ({ state }) => magReachable(state) ? { scalar: state.fault_inject ? 0 : 1, nextState: { mag_mode: 0 } } : { scalar: 0 },
    // ── tier-2 helpers: the driver's integer math on raw counts ──
    tile_sense_i_9_is_face_up: ({ state }) => {
      const [x, y, z] = accelRaw(state);
      if (Math.abs(x) > FACE_XY_MAX || Math.abs(y) > FACE_XY_MAX) return { scalar: 0 };
      return { scalar: z > FACE_Z_MIN ? 1 : 0 };
    },
    tile_sense_i_9_is_face_down: ({ state }) => {
      const [x, y, z] = accelRaw(state);
      if (Math.abs(x) > FACE_XY_MAX || Math.abs(y) > FACE_XY_MAX) return { scalar: 0 };
      return { scalar: z < -FACE_Z_MIN ? 1 : 0 };
    },
    tile_sense_i_9_is_moving: ({ state, args }) => {
      const [x, y, z] = accelRaw(state);
      const delta2 = Math.abs(x * x + y * y + z * z - LSB_PER_G_2G * LSB_PER_G_2G);
      const thrLsb = Math.trunc((args[0] ?? 0) * LSB_PER_G_2G / 1e3);
      return { scalar: delta2 > thrLsb * 2 * LSB_PER_G_2G ? 1 : 0 };
    },
    // Angle between the axis and "up" (0..18000), the driver's integer math on
    // raw counts: atan2(isqrt(other² + other²), axis).
    tile_sense_i_9_read_tilt_centi_degrees_flat: ({ state, args }) => {
      const a = accelRaw(state);
      const axis = args[0] ?? 0;
      const target = axis >= 0 && axis < 3 ? a[axis] : 0;
      const oa = axis === 0 ? a[1] : a[0];
      const ob = axis === 2 ? a[1] : a[2];
      const perp = Math.floor(Math.sqrt(oa * oa + ob * ob));
      const c = Math.max(-18e3, Math.min(18e3, atan2Centi(perp, target)));
      return { outScalars: { out_centi_deg: c } };
    },
    // Heading of +X from magnetic north: atan2(−HY, HX) on AK09916 counts,
    // normalised to 0..35999. No mag-axis remap (as the driver). Without
    // BYPASS_EN the mag doesn't answer and the local is left untouched.
    tile_sense_i_9_read_heading_centi_degrees_flat: ({ state }) => {
      if (!magReachable(state)) return void 0;
      const on = magMeasuring(state);
      const mx = on ? magCount(state.mag_x_ut) : 0;
      const my = on ? magCount(state.mag_y_ut) : 0;
      let c = atan2Centi(-my, mx);
      if (c < 0) c += 36e3;
      if (c >= 36e3) c -= 36e3;
      return { outScalars: { out_centi_deg: c } };
    },
    // ── DMP ──
    // dmp_start_quat9 needs dmp_load() first (driver .c:1333), which no program
    // can call (it isn't in the manifest) — so, as on the hardware, the DMP
    // never starts and its reads report nothing.
    tile_sense_i_9_dmp_start_quat9: () => ({ scalar: 0 }),
    tile_sense_i_9_dmp_stop: () => void 0,
    tile_sense_i_9_dmp_data_ready: () => ({ scalar: 0 }),
    tile_sense_i_9_dmp_read_quat9: () => ({ scalar: 0 })
  },
  provenance: {
    tile_sense_i_9_find: "canonical",
    tile_sense_i_9_init: "canonical",
    tile_sense_i_9_reset: "canonical",
    // PWR_MGMT_1 / INT_PIN_CFG reset values
    tile_sense_i_9_sleep: "canonical",
    tile_sense_i_9_wake: "canonical",
    tile_sense_i_9_set_accel_range: "canonical",
    tile_sense_i_9_set_gyro_range: "canonical",
    tile_sense_i_9_get_raw_accels: "canonical",
    // LSB/g table
    tile_sense_i_9_get_raw_gyros: "canonical",
    // LSB/dps table
    tile_sense_i_9_get_raw_6dof: "canonical",
    tile_sense_i_9_get_raw_mags: "canonical",
    // 0.15 µT/LSB
    tile_sense_i_9_mag_overflowed: "canonical",
    // HOFL at 4912 µT
    tile_sense_i_9_get_temperature: "canonical",
    // /333.87 + 21
    tile_sense_i_9_int_config: "canonical",
    tile_sense_i_9_get_int_status: "canonical",
    tile_sense_i_9_wom_config: "canonical",
    // 4 mg/LSB
    tile_sense_i_9_is_face_up: "canonical",
    // driver constants
    tile_sense_i_9_is_face_down: "canonical",
    tile_sense_i_9_is_moving: "canonical",
    tile_sense_i_9_fifo_count: "canonical",
    // bytes
    tile_sense_i_9_data_ready: "inferred",
    // "always fresh" at the 10 Hz tick
    tile_sense_i_9_self_test: "inferred",
    // pass assumed; side effects canonical
    tile_sense_i_9_mag_self_test: "inferred",
    tile_sense_i_9_fifo_read_packet_flat: "inferred",
    tile_sense_i_9_get_int_status_fifo_watermark: "inferred",
    // WM level not modeled
    tile_sense_i_9_read_tilt_centi_degrees_flat: "canonical",
    // driver integer math
    tile_sense_i_9_read_heading_centi_degrees_flat: "inferred",
    // mag axis remap unverified
    tile_sense_i_9_dmp_start_quat9: "inferred",
    // needs dmp_load (not exposed)
    tile_sense_i_9_dmp_read_quat9: "inferred",
    tile_sense_i_9_dmp_data_ready: "inferred",
    power: "inferred"
    // 9-axis LN 3.11 mA / sleep 8 µA / mag 90 µA@8 Hz canonical; split scaled
  },
  deriveState(state, { t }) {
    const u = {};
    const hz = sampleHz(state);
    if (hz > 0 && t - state.last_drdy_ms >= 1e3 / hz) {
      if (!(state.int_status_1 & RAW_DATA_RDY)) u.int_status_1 = state.int_status_1 | RAW_DATA_RDY;
      u.last_drdy_ms = t;
    }
    const per = fifoSampleBytes(state);
    if (per > 0 && hz > 0) {
      if (state.last_fifo_ms < 0) {
        u.last_fifo_ms = t;
      } else {
        const n = Math.floor((t - state.last_fifo_ms) * hz / 1e3);
        if (n > 0) {
          const total = state.fifo_bytes + n * per;
          if (total > FIFO_BYTES) {
            u.fifo_bytes = state.fifo_mode === FIFO_SNAPSHOT ? state.fifo_bytes + Math.floor((FIFO_BYTES - state.fifo_bytes) / per) * per : Math.floor(FIFO_BYTES / per) * per;
            u.int_status_fifo_ovf = state.int_status_fifo_ovf | FIFO0;
          } else {
            u.fifo_bytes = total;
          }
          u.last_fifo_ms = state.last_fifo_ms + n * 1e3 / hz;
        }
      }
    }
    if (state.wom_global_en && !state.sleeping) {
      const thr = state.wom_threshold_mg;
      const ax = thr > 0 && Math.abs(state.accel_x_mg - state.wom_ref_x_mg) > thr ? 1 : 0;
      const ay = thr > 0 && Math.abs(state.accel_y_mg - state.wom_ref_y_mg) > thr ? 1 : 0;
      const az = thr > 0 && Math.abs(state.accel_z_mg - state.wom_ref_z_mg) > thr ? 1 : 0;
      const fresh = ax && !state.prev_wom_x || ay && !state.prev_wom_y || az && !state.prev_wom_z;
      if (fresh && !(state.int_status & INT_WOM)) u.int_status = state.int_status | INT_WOM;
      if (ax !== state.prev_wom_x) u.prev_wom_x = ax;
      if (ay !== state.prev_wom_y) u.prev_wom_y = ay;
      if (az !== state.prev_wom_z) u.prev_wom_z = az;
      if (state.wom_mode === 1) {
        if (state.wom_ref_x_mg !== state.accel_x_mg) u.wom_ref_x_mg = state.accel_x_mg;
        if (state.wom_ref_y_mg !== state.accel_y_mg) u.wom_ref_y_mg = state.accel_y_mg;
        if (state.wom_ref_z_mg !== state.accel_z_mg) u.wom_ref_z_mg = state.accel_z_mg;
      }
    } else {
      if (state.prev_wom_x) u.prev_wom_x = 0;
      if (state.prev_wom_y) u.prev_wom_y = 0;
      if (state.prev_wom_z) u.prev_wom_z = 0;
    }
    return u;
  },
  // INT (pad 9) follows its routed, latched sources at the polarity INT_PIN_CFG
  // sets (bit7: 1 = active-low; init leaves it active-high). The level is held
  // while the status is latched — the 50 µs pulse is below the canvas's resolution.
  padOutputs(state) {
    const asserted = state.int_dry_en === 1 && (state.int_status_1 & RAW_DATA_RDY) !== 0 || state.int_wom_en === 1 && (state.int_status & INT_WOM) !== 0 || state.int_fifo_ovf_en === 1 && (state.int_status_fifo_ovf & 31) !== 0 || state.int_fifo_wm_en === 1 && (state.int_status_fifo_wm & 31) !== 0;
    const activeLow = (state.int_pin_cfg & 128) !== 0;
    return { "9": (activeLow ? !asserted : asserted) ? 1 : 0 };
  },
  // V+ (pad 10), DS-000189 §3.4 typicals: 9-axis low-noise with the compass
  // continuous = 3.11 mA; magnetometer alone 90 µA at 8 Hz (scaled here by rate,
  // ≈1.1 mA at 100 Hz); full-chip sleep 8 µA. Accel + gyro awake = 3.11 mA less
  // the 100 Hz compass share. The driver never enables duty-cycled (LP) mode.
  // Sleep does not stop the AK09916: its draw stays until set_mag_mode(0).
  power(state, ctx) {
    const mag = magUa(state);
    const imu = state.sleeping ? 8 : 3110 - 100 * 90 / 8;
    const ua = imu + mag;
    return {
      draw_ua: ua,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: ctx?.padVoltage["10"] ?? 1800,
          i_ua: ua,
          pads: ["10"]
        }
      ]
    };
  }
};
var sense_i_9_default = sim;
export {
  sense_i_9_default as default
};
