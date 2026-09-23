// src/sims/sense_i_6p6.ts
var INT_DATA_RDY = 1 << 3;
var INT_FIFO_THS = 1 << 2;
var INT_FIFO_FULL = 1 << 1;
var WOM_X = 1 << 0;
var WOM_Y = 1 << 1;
var WOM_Z = 1 << 2;
var WOM_ANY = WOM_X | WOM_Y | WOM_Z;
var S3_TAP = 1 << 5;
var S3_TILT = 1 << 3;
var S3_STEP = 1 << 1;
var WHO_AM_I = 68;
var RAW_OFF = -32768;
var ACCEL_LSB_PER_G = [1024, 2048, 4096, 8192, 16384];
var GYRO_LSB_PER_DPS = [8.2, 16.4, 32.8, 65.5, 131, 262, 524.3, 1048.6];
var ODR_HZ = {
  1: 32e3,
  2: 16e3,
  3: 8e3,
  4: 4e3,
  5: 2e3,
  6: 1e3,
  7: 200,
  8: 100,
  9: 50,
  10: 25,
  11: 12.5,
  12: 6.25,
  13: 3.125,
  14: 1.5625,
  15: 500
};
var FIFO_BYTES = 2048;
var clampI16 = (v) => {
  const r = Math.round(v);
  return r > 32767 ? 32767 : r < -32768 ? -32768 : r;
};
var s12 = (v) => {
  const r = Math.trunc(v) & 4095;
  return r & 2048 ? r - 4096 : r;
};
var accelLsb = (s) => ACCEL_LSB_PER_G[s.accel_range] ?? 4096;
var gyroLsb = (s) => GYRO_LSB_PER_DPS[s.gyro_range] ?? 32.8;
var accelOn = (s) => s.power_accel !== 0;
var gyroOn = (s) => s.power_gyro === 3;
function accelRaw(s) {
  if (!accelOn(s)) return [RAW_OFF, RAW_OFF, RAW_OFF];
  const k = accelLsb(s) / 1e3;
  return [
    clampI16((s.accel_x_mg + s12(s.accel_off_x_mg)) * k),
    clampI16((s.accel_y_mg + s12(s.accel_off_y_mg)) * k),
    clampI16((s.accel_z_mg + s12(s.accel_off_z_mg)) * k)
  ];
}
function gyroRaw(s) {
  if (!gyroOn(s)) return [RAW_OFF, RAW_OFF, RAW_OFF];
  const k = gyroLsb(s);
  return [
    clampI16((s.gyro_x_dps + s12(s.gyro_off_x_dps16) / 16) * k),
    clampI16((s.gyro_y_dps + s12(s.gyro_off_y_dps16) / 16) * k),
    clampI16((s.gyro_z_dps + s12(s.gyro_off_z_dps16) / 16) * k)
  ];
}
var tempRaw = (s) => s.temp_enabled ? clampI16((s.temp_c - 25) * 132.48) : RAW_OFF;
var fifoTemp = (s) => Math.max(-128, Math.min(127, Math.round((s.temp_c - 25) * 2.07)));
function dataHz(s) {
  const a = accelOn(s) ? ODR_HZ[s.accel_odr] ?? 0 : 0;
  const g = gyroOn(s) ? ODR_HZ[s.gyro_odr] ?? 0 : 0;
  return Math.max(a, g);
}
var producing = (s) => dataHz(s) > 0;
var motionMg = (s) => Math.abs(Math.hypot(s.accel_x_mg, s.accel_y_mg, s.accel_z_mg) - 1e3);
var womThrMg = (mg) => (Math.trunc((Math.trunc(mg) * 256 + 500) / 1e3) & 255) * 1e3 / 256;
var fifoPacketBytes = (s) => s.fifo_hires ? 20 : s.fifo_accel_en && s.fifo_gyro_en ? 16 : 8;
var fifoOn = (s) => s.fifo_mode !== 0 && (s.fifo_accel_en || s.fifo_gyro_en) === 1 && producing(s);
var fifoPacket = (s) => [...accelRaw(s), ...gyroRaw(s), fifoTemp(s), 0];
function angleFrom(s, rx, ry, rz) {
  const a = Math.hypot(s.accel_x_mg, s.accel_y_mg, s.accel_z_mg);
  const r = Math.hypot(rx, ry, rz);
  if (a === 0 || r === 0) return 0;
  const c = (s.accel_x_mg * rx + s.accel_y_mg * ry + s.accel_z_mg * rz) / (a * r);
  return Math.acos(Math.max(-1, Math.min(1, c))) * 180 / Math.PI;
}
var pinLevel = (asserted, config) => (config & 1 ? asserted : !asserted) ? 1 : 0;
var sim = {
  tile: "Sense.I.6P6",
  defaultState: {
    accel_x_mg: 0,
    accel_y_mg: 0,
    accel_z_mg: 1e3,
    // resting flat, face up: 1 g on +Z
    gyro_x_dps: 0,
    gyro_y_dps: 0,
    gyro_z_dps: 0,
    temp_c: 25,
    tap_axis: 0,
    // tile_sense_i_6p6_init with a NULL cfg (driver .c:204-213)
    accel_range: 2,
    // ±8 g
    gyro_range: 2,
    // ±1000 dps
    accel_odr: 8,
    // 100 Hz
    gyro_odr: 8,
    power_accel: 3,
    // low-noise
    power_gyro: 3,
    sleeping: 0,
    // datasheet reset values: GYRO_ACCEL_CONFIG0 0x11, ACCEL_CONFIG1 0x0D, GYRO_CONFIG1 0x16
    filter_bw_a: 1,
    filter_bw_g: 1,
    filter_ord_a: 1,
    filter_ord_g: 1,
    temp_filter: 0,
    temp_enabled: 1,
    int1_config: 0,
    // INT_CONFIG reset 0x00: active-low, open-drain, pulsed
    int2_config: 0,
    int_pulse: 0,
    int1_dry_en: 0,
    int1_fifo_en: 0,
    int1_wom_en: 0,
    int2_dry_en: 0,
    int2_fifo_en: 0,
    int2_wom_en: 0,
    int_status: 0,
    int_status2: 0,
    int_status3: 0,
    last_drdy_ms: 0,
    wom_global_en: 0,
    // SMD_CONFIG reset 0x00
    wom_x_th_mg: 0,
    // ACCEL_WOM_*_THR reset 0x00
    wom_y_th_mg: 0,
    wom_z_th_mg: 0,
    wom_mode: 0,
    wom_ref_x_mg: 0,
    wom_ref_y_mg: 0,
    wom_ref_z_mg: 1e3,
    prev_wom_x: 0,
    prev_wom_y: 0,
    prev_wom_z: 0,
    smd_mode: 0,
    fifo_mode: 0,
    fifo_watermark: 0,
    fifo_accel_en: 0,
    fifo_gyro_en: 0,
    fifo_temp_en: 0,
    fifo_hires: 0,
    fifo_packets: 0,
    fifo_lost: 0,
    last_fifo_ms: -1,
    accel_off_x_mg: 0,
    accel_off_y_mg: 0,
    accel_off_z_mg: 0,
    gyro_off_x_dps16: 0,
    gyro_off_y_dps16: 0,
    gyro_off_z_dps16: 0,
    pedometer_enabled: 0,
    pedometer_dmp_odr: 2,
    // APEX_CONFIG0 reset 0x82: DMP_ODR 50 Hz
    step_count: 0,
    step_cadence: 0,
    activity: 0,
    last_step_ms: 0,
    tilt_enabled: 0,
    tilt_wait_seconds: 4,
    // APEX_CONFIG4 reset 0xA4: TILT_WAIT_TIME_SEL 2 → 4 s
    tilt_ref_x_mg: 0,
    tilt_ref_y_mg: 0,
    tilt_ref_z_mg: 1e3,
    tilt_since_ms: -1,
    prev_tilt: 0,
    tap_enabled: 0,
    tap_count: 0,
    tap_result_axis: 0,
    tap_direction: 0
  },
  controls: [
    {
      type: "slider",
      field: "accel_x_mg",
      label: "Accel X",
      min: -2e3,
      max: 2e3,
      step: 1,
      unit: "mg"
    },
    {
      type: "slider",
      field: "accel_y_mg",
      label: "Accel Y",
      min: -2e3,
      max: 2e3,
      step: 1,
      unit: "mg"
    },
    {
      type: "slider",
      field: "accel_z_mg",
      label: "Accel Z",
      min: -2e3,
      max: 2e3,
      step: 1,
      unit: "mg",
      description: "1000 mg \u2248 resting flat with gravity on Z."
    },
    {
      type: "slider",
      field: "gyro_x_dps",
      label: "Gyro X",
      min: -2e3,
      max: 2e3,
      step: 1,
      unit: "dps"
    },
    {
      type: "slider",
      field: "gyro_y_dps",
      label: "Gyro Y",
      min: -2e3,
      max: 2e3,
      step: 1,
      unit: "dps"
    },
    {
      type: "slider",
      field: "gyro_z_dps",
      label: "Gyro Z",
      min: -2e3,
      max: 2e3,
      step: 1,
      unit: "dps"
    },
    {
      type: "slider",
      field: "temp_c",
      label: "Temperature",
      min: -40,
      max: 85,
      step: 1,
      unit: "\xB0C"
    },
    {
      type: "slider",
      field: "tap_axis",
      label: "Tap (1 X \xB7 2 Y \xB7 3 Z)",
      min: 0,
      max: 3,
      step: 1,
      description: "Tap the part along an axis. Seen only while the program has tap detection on."
    }
  ],
  // Legacy SimulatorPane only: a slow tumble around X (deltas on the anchor).
  automatic: ({ t }) => {
    const tSec = t / 1e3;
    const omegaX = Math.PI / 4;
    return {
      accel_x_mg: Math.sin(tSec * (Math.PI / 5)) * 80,
      accel_y_mg: Math.sin(tSec * omegaX) * 1e3,
      accel_z_mg: Math.cos(tSec * omegaX) * 1e3,
      gyro_x_dps: omegaX * Math.cos(tSec * omegaX) * (180 / Math.PI),
      gyro_y_dps: Math.sin(tSec * (Math.PI / 7)) * 30,
      gyro_z_dps: Math.cos(tSec * (Math.PI / 11)) * 20
    };
  },
  // What a person does to an IMU: tilt it, push it, turn it, shake it, warm it.
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
        min: -2e3,
        max: 2e3,
        step: 10,
        unit: "mg"
      }))
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
      id: "temperature",
      label: "Temperature",
      controls: [
        {
          kind: "slider",
          id: "temp_c",
          label: "temperature",
          field: "temp_c",
          min: -40,
          max: 85,
          step: 1,
          unit: "\xB0C"
        }
      ]
    }
  ],
  hostCalls: {
    // ── lifecycle ──
    // find() returns 1 when the part ACKs (driver .c:120), not an address.
    tile_sense_i_6p6_find: () => ({ scalar: 1 }),
    tile_sense_i_6p6_init: () => ({
      nextState: {
        accel_range: 2,
        gyro_range: 2,
        accel_odr: 8,
        gyro_odr: 8,
        power_accel: 3,
        power_gyro: 3,
        sleeping: 0
      }
    }),
    // process() reads INT_STATUS (clearing it) and hands it to the C callback.
    tile_sense_i_6p6_process: () => ({ nextState: { int_status: 0 } }),
    tile_sense_i_6p6_on_event: () => void 0,
    tile_sense_i_6p6_sleep: () => ({
      nextState: { power_accel: 0, power_gyro: 0, sleeping: 1 }
    }),
    tile_sense_i_6p6_wake: () => ({
      nextState: { power_accel: 3, power_gyro: 3, sleeping: 0, last_drdy_ms: 0 }
    }),
    // DEVICE_CONFIG soft reset: every register back to its reset value —
    // sensors OFF, ±32 g / ±4000 dps at 1 kHz (ACCEL/GYRO_CONFIG0 = 0x06). The
    // driver says init() must run again.
    tile_sense_i_6p6_reset: () => ({
      nextState: {
        accel_range: 0,
        gyro_range: 0,
        accel_odr: 6,
        gyro_odr: 6,
        power_accel: 0,
        power_gyro: 0,
        sleeping: 0,
        filter_bw_a: 1,
        filter_bw_g: 1,
        filter_ord_a: 1,
        filter_ord_g: 1,
        temp_filter: 0,
        temp_enabled: 1,
        int1_config: 0,
        int2_config: 0,
        int_pulse: 0,
        int1_dry_en: 0,
        int1_fifo_en: 0,
        int1_wom_en: 0,
        int2_dry_en: 0,
        int2_fifo_en: 0,
        int2_wom_en: 0,
        int_status: 0,
        int_status2: 0,
        int_status3: 0,
        wom_global_en: 0,
        wom_x_th_mg: 0,
        wom_y_th_mg: 0,
        wom_z_th_mg: 0,
        wom_mode: 0,
        prev_wom_x: 0,
        prev_wom_y: 0,
        prev_wom_z: 0,
        smd_mode: 0,
        fifo_mode: 0,
        fifo_watermark: 0,
        fifo_accel_en: 0,
        fifo_gyro_en: 0,
        fifo_temp_en: 0,
        fifo_hires: 0,
        fifo_packets: 0,
        fifo_lost: 0,
        last_fifo_ms: -1,
        accel_off_x_mg: 0,
        accel_off_y_mg: 0,
        accel_off_z_mg: 0,
        gyro_off_x_dps16: 0,
        gyro_off_y_dps16: 0,
        gyro_off_z_dps16: 0,
        pedometer_enabled: 0,
        pedometer_dmp_odr: 2,
        step_count: 0,
        step_cadence: 0,
        activity: 0,
        tilt_enabled: 0,
        tilt_wait_seconds: 4,
        tilt_since_ms: -1,
        prev_tilt: 0,
        tap_enabled: 0,
        tap_count: 0,
        tap_result_axis: 0,
        tap_direction: 0
      }
    }),
    // APEX (0) resets the DMP memory → step/activity/APEX status cleared.
    // TEMP (1) resets the temperature path: nothing the twin holds.
    tile_sense_i_6p6_subsystem_reset: ({ args }) => (args[0] ?? 0) === 0 ? {
      nextState: {
        step_count: 0,
        step_cadence: 0,
        activity: 0,
        int_status3: 0,
        tap_count: 0,
        tap_result_axis: 0,
        tap_direction: 0
      }
    } : void 0,
    // ── configuration ──
    tile_sense_i_6p6_set_accel_range: ({ args }) => ({
      nextState: { accel_range: (args[0] ?? 0) & 7 }
    }),
    tile_sense_i_6p6_set_gyro_range: ({ args }) => ({
      nextState: { gyro_range: (args[0] ?? 0) & 7 }
    }),
    tile_sense_i_6p6_set_accel_odr: ({ args }) => ({
      nextState: { accel_odr: (args[0] ?? 0) & 15 }
    }),
    tile_sense_i_6p6_set_gyro_odr: ({ args }) => ({
      nextState: { gyro_odr: (args[0] ?? 0) & 15 }
    }),
    tile_sense_i_6p6_set_power_mode: ({ args }) => ({
      nextState: { power_accel: (args[0] ?? 0) & 3, power_gyro: (args[1] ?? 0) & 3 }
    }),
    tile_sense_i_6p6_set_filter_bw: ({ args }) => ({
      nextState: { filter_bw_a: (args[0] ?? 0) & 15, filter_bw_g: (args[1] ?? 0) & 15 }
    }),
    tile_sense_i_6p6_set_filter_order: ({ args }) => ({
      nextState: { filter_ord_a: (args[0] ?? 0) & 3, filter_ord_g: (args[1] ?? 0) & 3 }
    }),
    tile_sense_i_6p6_set_temp_filter: ({ args }) => ({
      nextState: { temp_filter: (args[0] ?? 0) & 7 }
    }),
    tile_sense_i_6p6_set_temp_enabled: ({ args }) => ({
      nextState: { temp_enabled: args[0] ? 1 : 0 }
    }),
    // ── data reads ──
    // Reads INT_STATUS (which clears it). A fresh sample is always waiting
    // while a sensor runs: every ODR (≥ 1.5 Hz) outpaces the canvas's 10 Hz tick.
    tile_sense_i_6p6_data_ready: ({ state }) => ({
      scalar: producing(state) ? 1 : 0,
      nextState: { int_status: 0 }
    }),
    tile_sense_i_6p6_get_raw_accels: ({ state }) => ({ array: accelRaw(state) }),
    tile_sense_i_6p6_get_raw_gyros: ({ state }) => ({ array: gyroRaw(state) }),
    tile_sense_i_6p6_get_raw_6dof: ({ state }) => ({
      array: [...accelRaw(state), ...gyroRaw(state)]
    }),
    tile_sense_i_6p6_get_temperature: ({ state }) => ({ scalar: tempRaw(state) }),
    // [Temp, AX, AY, AZ, GX, GY, GZ] (driver .c:402)
    tile_sense_i_6p6_get_raw_all: ({ state }) => ({
      array: [tempRaw(state), ...accelRaw(state), ...gyroRaw(state)]
    }),
    // ── orientation / motion helpers: the driver's integer math on raw counts ──
    tile_sense_i_6p6_is_face_up: ({ state }) => {
      const [x, y, z] = accelRaw(state);
      const lsb = accelLsb(state);
      const tz = Math.trunc(lsb * 200 / 1e3);
      const txy = Math.trunc(lsb * 300 / 1e3);
      const ok = z >= lsb - tz && z <= lsb + tz && Math.abs(x) <= txy && Math.abs(y) <= txy;
      return { scalar: ok ? 1 : 0 };
    },
    tile_sense_i_6p6_is_face_down: ({ state }) => {
      const [x, y, z] = accelRaw(state);
      const lsb = accelLsb(state);
      const tz = Math.trunc(lsb * 200 / 1e3);
      const txy = Math.trunc(lsb * 300 / 1e3);
      const ok = z >= -lsb - tz && z <= -lsb + tz && Math.abs(x) <= txy && Math.abs(y) <= txy;
      return { scalar: ok ? 1 : 0 };
    },
    tile_sense_i_6p6_is_moving: ({ state, args }) => {
      const [x, y, z] = accelRaw(state);
      const lsb = accelLsb(state);
      const mag2 = x * x + y * y + z * z;
      const thr = Math.trunc(lsb * (args[0] ?? 0) / 1e3);
      const hi = lsb + thr;
      const lo = Math.max(0, lsb - thr);
      return { scalar: mag2 > hi * hi || mag2 < lo * lo ? 1 : 0 };
    },
    // Returns 1 once the angle is written through `out_centi_deg`. The manifest
    // declares no out-scalar for that pointer, so the twin can't fill it.
    tile_sense_i_6p6_read_tilt_centi_degrees: ({ state, args }) => {
      const axis = args[0] ?? 0;
      if (axis > 2) return { scalar: 0 };
      const a = accelRaw(state);
      const all0 = a[(axis + 1) % 3] === 0 && a[(axis + 2) % 3] === 0 && a[axis] === 0;
      return { scalar: all0 ? 0 : 1 };
    },
    // Polls INT_STATUS3 (clear-on-read). Time can't pass inside one host call,
    // so this is the first poll's answer.
    tile_sense_i_6p6_wait_for_tap: ({ state }) => ({
      scalar: state.int_status3 & S3_TAP ? 1 : 0,
      nextState: { int_status3: 0 }
    }),
    tile_sense_i_6p6_wait_for_motion: ({ state }) => ({
      scalar: state.int_status2 & WOM_ANY ? 1 : 0,
      nextState: { int_status2: 0 }
    }),
    // ── FIFO ──
    tile_sense_i_6p6_fifo_config: ({ args }) => ({
      nextState: {
        fifo_mode: (args[0] ?? 0) & 3,
        fifo_accel_en: args[1] ? 1 : 0,
        fifo_gyro_en: args[2] ? 1 : 0,
        fifo_temp_en: args[3] ? 1 : 0,
        fifo_hires: args[4] ? 1 : 0,
        last_fifo_ms: -1
      }
    }),
    // 0 is invalid per datasheet → the driver writes 1. 12-bit register.
    tile_sense_i_6p6_fifo_set_watermark: ({ args }) => ({
      nextState: { fifo_watermark: Math.max(1, (args[0] ?? 0) & 65535) & 4095 }
    }),
    tile_sense_i_6p6_fifo_flush: () => ({
      nextState: { fifo_packets: 0, last_fifo_ms: -1 }
    }),
    // FIFO_COUNT in bytes (INTF_CONFIG0.FIFO_COUNT_REC = 0 at reset).
    tile_sense_i_6p6_fifo_count: ({ state }) => ({
      scalar: state.fifo_packets * fifoPacketBytes(state)
    }),
    tile_sense_i_6p6_fifo_lost_count: ({ state }) => ({ scalar: state.fifo_lost & 65535 }),
    // One packet [ax ay az gx gy gz temp8 timestamp]; an empty FIFO leaves the
    // caller's array untouched (driver .c:490).
    tile_sense_i_6p6_fifo_read_packet_flat: ({ state }) => state.fifo_packets > 0 ? { array: fifoPacket(state), nextState: { fifo_packets: state.fifo_packets - 1 } } : void 0,
    // Drains up to cap/8 packets, 8 ints each; returns the packet count.
    tile_sense_i_6p6_fifo_read_packets_flat: ({ state, caps }) => {
      const cap = caps?.out ?? 0;
      const n = Math.min(Math.floor(cap / 8), state.fifo_packets);
      if (n <= 0) return { scalar: 0 };
      const one = fifoPacket(state);
      const out = [];
      for (let i = 0; i < n; i++) out.push(...one);
      return { scalar: n, out: { out }, nextState: { fifo_packets: state.fifo_packets - n } };
    },
    // ── interrupts ──
    tile_sense_i_6p6_int1_config: ({ args }) => ({
      nextState: { int1_config: (args[0] ?? 0) & 7 }
    }),
    tile_sense_i_6p6_int2_config: ({ args }) => ({
      nextState: { int2_config: (args[0] ?? 0) & 7 }
    }),
    tile_sense_i_6p6_int1_data_ready: ({ args }) => ({
      nextState: { int1_dry_en: args[0] ? 1 : 0 }
    }),
    tile_sense_i_6p6_int1_fifo_ths: ({ args }) => ({
      nextState: { int1_fifo_en: args[0] ? 1 : 0 }
    }),
    tile_sense_i_6p6_int1_wom: ({ args }) => ({ nextState: { int1_wom_en: args[0] ? 1 : 0 } }),
    tile_sense_i_6p6_int2_data_ready: ({ args }) => ({
      nextState: { int2_dry_en: args[0] ? 1 : 0 }
    }),
    tile_sense_i_6p6_int2_fifo_ths: ({ args }) => ({
      nextState: { int2_fifo_en: args[0] ? 1 : 0 }
    }),
    tile_sense_i_6p6_int2_wom: ({ args }) => ({ nextState: { int2_wom_en: args[0] ? 1 : 0 } }),
    tile_sense_i_6p6_set_int_pulse_duration: ({ args }) => ({
      nextState: { int_pulse: args[0] === 1 ? 1 : 0 }
    }),
    tile_sense_i_6p6_get_int_status: ({ state }) => ({
      scalar: state.int_status,
      nextState: { int_status: 0 }
    }),
    tile_sense_i_6p6_get_int_status2: ({ state }) => ({
      scalar: state.int_status2,
      nextState: { int_status2: 0 }
    }),
    tile_sense_i_6p6_get_int_status3: ({ state }) => ({
      scalar: state.int_status3,
      nextState: { int_status3: 0 }
    }),
    // ── wake-on-motion ──
    tile_sense_i_6p6_wom_config: ({ args }) => ({
      nextState: {
        wom_x_th_mg: womThrMg(args[0] ?? 0),
        wom_y_th_mg: womThrMg(args[1] ?? 0),
        wom_z_th_mg: womThrMg(args[2] ?? 0),
        wom_mode: (args[3] ?? 0) & 1
      }
    }),
    // SMD_CONFIG = 0x05: WOM_MODE = previous-sample, SMD_MODE = WOM. It
    // overwrites the compare mode wom_config chose.
    tile_sense_i_6p6_wom_enable: ({ state }) => ({
      nextState: {
        wom_global_en: 1,
        smd_mode: 1,
        wom_mode: 1,
        wom_ref_x_mg: state.accel_x_mg,
        wom_ref_y_mg: state.accel_y_mg,
        wom_ref_z_mg: state.accel_z_mg,
        prev_wom_x: 0,
        prev_wom_y: 0,
        prev_wom_z: 0
      }
    }),
    // The driver only zeroes the thresholds; SMD_CONFIG keeps WOM running. A
    // zero threshold never fires here (see deriveState).
    tile_sense_i_6p6_wom_disable: () => ({
      nextState: { wom_x_th_mg: 0, wom_y_th_mg: 0, wom_z_th_mg: 0 }
    }),
    tile_sense_i_6p6_smd_config: ({ args }) => {
      const mode = (args[0] ?? 0) & 3;
      return { nextState: { smd_mode: mode, wom_global_en: mode !== 0 ? 1 : 0 } };
    },
    // ── APEX: pedometer ──
    tile_sense_i_6p6_pedometer_enable: ({ args }) => ({
      nextState: { pedometer_enabled: 1, pedometer_dmp_odr: (args[0] ?? 0) & 3 }
    }),
    tile_sense_i_6p6_pedometer_disable: () => ({ nextState: { pedometer_enabled: 0 } }),
    tile_sense_i_6p6_get_step_count: ({ state }) => ({ scalar: state.step_count & 65535 }),
    tile_sense_i_6p6_get_step_cadence: ({ state }) => ({ scalar: state.step_cadence & 255 }),
    tile_sense_i_6p6_get_activity: ({ state }) => ({ scalar: state.activity & 3 }),
    // ── APEX: tilt / tap ──
    // TILT_WAIT_TIME_SEL: ≤0 → 0 s, ≤2 → 2 s, ≤4 → 4 s, else 6 s.
    tile_sense_i_6p6_tilt_enable: ({ state, args }) => {
      const w = args[0] ?? 0;
      const sel = w <= 0 ? 0 : w <= 2 ? 1 : w <= 4 ? 2 : 3;
      return {
        nextState: {
          tilt_enabled: 1,
          tilt_wait_seconds: sel * 2,
          tilt_ref_x_mg: state.accel_x_mg,
          tilt_ref_y_mg: state.accel_y_mg,
          tilt_ref_z_mg: state.accel_z_mg,
          tilt_since_ms: -1,
          prev_tilt: 0
        }
      };
    },
    tile_sense_i_6p6_tilt_disable: () => ({ nextState: { tilt_enabled: 0, tilt_since_ms: -1 } }),
    tile_sense_i_6p6_tap_enable: () => ({ nextState: { tap_enabled: 1 } }),
    tile_sense_i_6p6_tap_disable: () => ({ nextState: { tap_enabled: 0 } }),
    // APEX_DATA4/5: TAP_NUM (1 single, 2 double), TAP_AXIS (0 X, 1 Y, 2 Z),
    // TAP_DIR, DOUBLE_TAP_TIMING (not modeled → 0).
    tile_sense_i_6p6_get_tap_result_flat: ({ state }) => ({
      outScalars: {
        count: state.tap_count,
        axis: state.tap_result_axis,
        direction: state.tap_direction,
        timing: 0
      }
    }),
    // ── advanced ──
    tile_sense_i_6p6_set_accel_offset: ({ args }) => ({
      nextState: {
        accel_off_x_mg: s12(args[0] ?? 0),
        accel_off_y_mg: s12(args[1] ?? 0),
        accel_off_z_mg: s12(args[2] ?? 0)
      }
    }),
    tile_sense_i_6p6_set_gyro_offset: ({ args }) => ({
      nextState: {
        gyro_off_x_dps16: s12(args[0] ?? 0),
        gyro_off_y_dps16: s12(args[1] ?? 0),
        gyro_off_z_dps16: s12(args[2] ?? 0)
      }
    }),
    // A healthy part passes. The driver reconfigures ±4 g / ±250 dps at 1 kHz
    // for the test and restores ONLY PWR_MGMT0 (driver .c:904-981), so the
    // ranges and ODRs stay changed afterwards.
    tile_sense_i_6p6_self_test: () => ({
      scalar: 1,
      outScalars: { accel_pass: 1, gyro_pass: 1 },
      nextState: { accel_range: 3, gyro_range: 4, accel_odr: 6, gyro_odr: 6 }
    }),
    // Only WHO_AM_I (bank 0, 0x75) is modeled.
    tile_sense_i_6p6_read_reg: ({ args }) => ({
      scalar: (args[0] ?? 0) === 0 && (args[1] ?? 0) === 117 ? WHO_AM_I : 0
    }),
    tile_sense_i_6p6_write_reg: () => void 0
  },
  provenance: {
    tile_sense_i_6p6_find: "canonical",
    tile_sense_i_6p6_init: "canonical",
    // driver .c:204-213
    tile_sense_i_6p6_reset: "canonical",
    // register reset values
    tile_sense_i_6p6_sleep: "canonical",
    tile_sense_i_6p6_wake: "canonical",
    tile_sense_i_6p6_get_raw_accels: "canonical",
    // LSB/g table, 0x8000 when off
    tile_sense_i_6p6_get_raw_gyros: "canonical",
    tile_sense_i_6p6_get_raw_6dof: "canonical",
    tile_sense_i_6p6_get_raw_all: "canonical",
    tile_sense_i_6p6_get_temperature: "canonical",
    // raw/132.48 + 25
    tile_sense_i_6p6_set_accel_range: "canonical",
    tile_sense_i_6p6_set_gyro_range: "canonical",
    tile_sense_i_6p6_set_power_mode: "canonical",
    tile_sense_i_6p6_is_face_up: "canonical",
    // driver integer math
    tile_sense_i_6p6_is_face_down: "canonical",
    tile_sense_i_6p6_is_moving: "canonical",
    tile_sense_i_6p6_get_int_status: "canonical",
    tile_sense_i_6p6_get_int_status2: "canonical",
    tile_sense_i_6p6_get_int_status3: "canonical",
    tile_sense_i_6p6_int1_data_ready: "canonical",
    tile_sense_i_6p6_int1_fifo_ths: "canonical",
    tile_sense_i_6p6_int1_wom: "canonical",
    tile_sense_i_6p6_int2_data_ready: "canonical",
    tile_sense_i_6p6_int2_fifo_ths: "canonical",
    tile_sense_i_6p6_int2_wom: "canonical",
    tile_sense_i_6p6_wom_config: "canonical",
    // thr = (mg·256 + 500)/1000
    tile_sense_i_6p6_wom_enable: "canonical",
    // SMD_CONFIG = 0x05
    tile_sense_i_6p6_set_accel_offset: "canonical",
    // 1 mg/LSB, 12-bit
    tile_sense_i_6p6_set_gyro_offset: "canonical",
    // 1/16 dps/LSB, 12-bit
    tile_sense_i_6p6_fifo_count: "canonical",
    // bytes
    tile_sense_i_6p6_self_test: "inferred",
    // pass is assumed; side effects canonical
    tile_sense_i_6p6_data_ready: "inferred",
    // "always fresh" at 10 Hz tick
    tile_sense_i_6p6_fifo_read_packet_flat: "inferred",
    // timestamp not modeled
    tile_sense_i_6p6_fifo_read_packets_flat: "inferred",
    tile_sense_i_6p6_read_tilt_centi_degrees: "inferred",
    // angle not deliverable
    tile_sense_i_6p6_get_step_count: "hallucinated",
    // step model is a placeholder
    tile_sense_i_6p6_get_step_cadence: "hallucinated",
    tile_sense_i_6p6_get_activity: "hallucinated",
    tile_sense_i_6p6_set_filter_bw: "inferred",
    // stored, no effect on data
    tile_sense_i_6p6_set_filter_order: "inferred",
    tile_sense_i_6p6_set_temp_filter: "inferred",
    tile_sense_i_6p6_smd_config: "inferred",
    // SMD event itself not modeled
    tile_sense_i_6p6_write_reg: "hallucinated",
    tile_sense_i_6p6_read_reg: "inferred",
    // WHO_AM_I only
    power: "inferred"
    // LN/sleep canonical (Table 3); LP and standby estimated
  },
  // The chip over time: DATA_RDY at the ODR, FIFO filling, WOM / tilt / tap /
  // step events latching into their status bytes.
  deriveState(state, { t }) {
    const u = {};
    const hz = dataHz(state);
    let s1 = state.int_status;
    if (hz > 0 && t - state.last_drdy_ms >= 1e3 / hz) {
      s1 |= INT_DATA_RDY;
      u.last_drdy_ms = t;
    }
    if (fifoOn(state)) {
      if (state.last_fifo_ms < 0) {
        u.last_fifo_ms = t;
      } else {
        const n = Math.floor((t - state.last_fifo_ms) * hz / 1e3);
        if (n > 0) {
          const bytes2 = fifoPacketBytes(state);
          const cap = Math.floor(FIFO_BYTES / bytes2);
          const total = state.fifo_packets + n;
          u.fifo_packets = Math.min(cap, total);
          if (total > cap) u.fifo_lost = state.fifo_lost + (total - cap);
          u.last_fifo_ms = state.last_fifo_ms + n * 1e3 / hz;
        }
      }
      const packets = u.fifo_packets ?? state.fifo_packets;
      const bytes = packets * fifoPacketBytes(state);
      if (state.fifo_watermark > 0 && bytes >= state.fifo_watermark) s1 |= INT_FIFO_THS;
      if (packets >= Math.floor(FIFO_BYTES / fifoPacketBytes(state))) s1 |= INT_FIFO_FULL;
    }
    if (s1 !== state.int_status) u.int_status = s1;
    if (state.wom_global_en && accelOn(state)) {
      const dx = Math.abs(state.accel_x_mg - state.wom_ref_x_mg);
      const dy = Math.abs(state.accel_y_mg - state.wom_ref_y_mg);
      const dz = Math.abs(state.accel_z_mg - state.wom_ref_z_mg);
      const ax = state.wom_x_th_mg > 0 && dx > state.wom_x_th_mg ? 1 : 0;
      const ay = state.wom_y_th_mg > 0 && dy > state.wom_y_th_mg ? 1 : 0;
      const az = state.wom_z_th_mg > 0 && dz > state.wom_z_th_mg ? 1 : 0;
      let s2 = state.int_status2;
      if (ax && !state.prev_wom_x) s2 |= WOM_X;
      if (ay && !state.prev_wom_y) s2 |= WOM_Y;
      if (az && !state.prev_wom_z) s2 |= WOM_Z;
      if (s2 !== state.int_status2) u.int_status2 = s2;
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
    let s3 = state.int_status3;
    if (state.tilt_enabled && accelOn(state)) {
      const tilted = angleFrom(state, state.tilt_ref_x_mg, state.tilt_ref_y_mg, state.tilt_ref_z_mg) > 35;
      if (!tilted) {
        if (state.tilt_since_ms !== -1) u.tilt_since_ms = -1;
        if (state.prev_tilt) u.prev_tilt = 0;
      } else if (state.tilt_since_ms === -1) {
        u.tilt_since_ms = t;
      } else if (!state.prev_tilt && t - state.tilt_since_ms >= state.tilt_wait_seconds * 1e3) {
        s3 |= S3_TILT;
        u.prev_tilt = 1;
      }
    }
    if (state.tap_axis > 0) {
      if (state.tap_enabled && accelOn(state)) {
        s3 |= S3_TAP;
        u.tap_count = 1;
        u.tap_result_axis = Math.min(2, state.tap_axis - 1);
        u.tap_direction = 0;
      }
      u.tap_axis = 0;
    }
    if (state.pedometer_enabled && accelOn(state)) {
      if (motionMg(state) > 50) {
        if (t - state.last_step_ms >= 500) {
          u.step_count = state.step_count + 1 & 65535;
          u.last_step_ms = t;
          s3 |= S3_STEP;
          const dmpHz = state.pedometer_dmp_odr === 0 ? 25 : 50;
          u.step_cadence = Math.round(dmpHz / 2 * 4);
          u.activity = 1;
        }
      } else if (state.step_cadence !== 0 || state.activity !== 0) {
        u.step_cadence = 0;
        u.activity = 0;
      }
    }
    if (s3 !== state.int_status3) u.int_status3 = s3;
    return u;
  },
  // INT1 = pad 9, INT2 = pad 8. Each pin follows its routed, latched sources
  // at the polarity INT_CONFIG sets (reset: active-low, so idle high). APEX
  // events route through INT_SOURCE6/7, which the driver never sets (reset 0).
  padOutputs(state) {
    const drdy = (state.int_status & INT_DATA_RDY) !== 0;
    const ths = (state.int_status & INT_FIFO_THS) !== 0;
    const wom = (state.int_status2 & WOM_ANY) !== 0;
    const int1 = state.int1_dry_en === 1 && drdy || state.int1_fifo_en === 1 && ths || state.int1_wom_en === 1 && wom;
    const int2 = state.int2_dry_en === 1 && drdy || state.int2_fifo_en === 1 && ths || state.int2_wom_en === 1 && wom;
    return { "9": pinLevel(int1, state.int1_config), "8": pinLevel(int2, state.int2_config) };
  },
  // V+ (pad 10) draw by PWR_MGMT0, DS-000639 Table 3 typicals: accel + gyro LN
  // 0.70 mA, accel LN 0.28 mA, gyro LN 0.58 mA, full-chip sleep 7.5 µA. Accel
  // LP and gyro standby aren't in the table — estimated.
  power(state, ctx) {
    const accelLN = state.power_accel === 3;
    const accelLP = state.power_accel === 2;
    const gyro = state.power_gyro === 3 || state.power_gyro === 1;
    let ua = 7.5;
    if (gyro && accelLN) ua = 700;
    else if (gyro && accelLP)
      ua = 600;
    else if (gyro) ua = 580;
    else if (accelLN) ua = 280;
    else if (accelLP) ua = 30;
    return {
      draw_ua: ua,
      rails: [
        { name: "V+", role: "supply", v_mv: ctx?.padVoltage["10"] ?? 1800, i_ua: ua, pads: ["10"] }
      ]
    };
  }
};
var sense_i_6p6_default = sim;
export {
  sense_i_6p6_default as default
};
