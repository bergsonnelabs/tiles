// Digital twin for Drive.DC.H — TI DRV8214 brushed-DC H-bridge motor driver.
//
// 11 V / 2 A H-bridge with current sense (IPROPI → NPROP), voltage / speed
// regulation, stall detection and sensorless ripple counting. I²C on pads 4/5;
// the bridge drives a motor across OUT1 / OUT2: chip OUT1 on pad 8, OUT2 on pad 7,
// as the tile definition lists them since the 2026-06 product-DB correction (the
// pre-correction JSON had them swapped). NPROP (pad 6) mirrors motor current.
// VM on pad 9, the logic supply V+ (the chip's VCC) on pad 10.
//
// ONE state, two readers: the firmware's calls (worker) and power / pads (main
// thread) read the same fields. The motor is external, so its electrical and
// mechanical response is MODELED from three physical stimuli — VM, winding
// resistance, mechanical load — through one function (`motorOp`) that every
// reader shares. The chip side follows tile_drive_dc_h.c + the DRV8214 datasheet
// (SLVSH04): init values, the VMTR / IMTR / RC_STATUS scalings, FAULT bits (NPOR
// latched high by init's CLR_FLT; CNT_DONE cleared only by CLR_CNT), EN_RC gating
// the ripple counter, sleep = EN_OUT only (outputs Hi-Z; nSLEEP isn't on a pad),
// and the "not ready" gate a sleeping tile puts on every bridge call.
import type { PowerCtx, TileSim } from '../tileSim';

// CS_GAIN_SEL → max current (mA), the driver's cs_max_ma table.
const CS_MAX_MA = [4000, 2000, 1000, 500, 250, 125, 250, 125];
// W_SCALE codes (REG_CTRL0[1:0]) → ripple rad/s per SPEED LSB (datasheet Table 8-24).
const W_SCALE = [16, 32, 64, 128];
// Ripple speed is in rad/s (datasheet Eq. 11): ripple Hz × 2π. The driver's
// integer conversions use 2π × 1000 = 6283.
const TWO_PI_X1000 = 6283;
// rpm → ripple rad/s and back, for `ripples_per_rev` ripples per shaft turn.
const rpmToRad = (rpm: number, rpr: number) => (rpm * rpr * 2 * Math.PI) / 60;
const radToRpm = (rad: number, rpr: number) => (rad * 60) / (2 * Math.PI * Math.max(1, rpr));

// bridge state (`direction`)
const COAST = 0;
const FORWARD = 1;
const REVERSE = 2;
const BRAKE = 3;

// drive_dc_h_control_mode_t / drive_dc_h_reg_mode_t / drive_dc_h_direction_t
const CTRL_I2C = 0;
const REG_SPEED = 2;
const REG_VOLTAGE = 3;
const DIR_REVERSE = 1;
// drive_dc_h_stall_recovery_t: 0 = latch outputs off on stall
const STALL_LATCH = 0;

// FAULT register bits
const FLT_FAULT = 0x80;
const FLT_STALL = 0x20;
const FLT_NPOR = 0x02;
const FLT_CNT_DONE = 0x01;

// datasheet §7.5
const IVM_ACTIVE_UA = 1300;
const IVCC_ACTIVE_UA = 1500;
const VCC_UVLO_MV = 1650; // VCC below this: device off (UVLO)
const VM_MIN_MV = 1650; // full-bridge supply minimum

// The external motor (modeled — no datasheet): default back-EMF constant (a
// placeholder for a small 3–6 V hobby motor, ≈ 3.3 rpm/mV; set_motor_params()'s
// kv argument, or the "Motor Kv" control, replaces it) and the share of stall
// current a spinning, unloaded rotor still draws.
const MOTOR_KV_UV_PER_RPM = 300;
const NO_LOAD_CURRENT_FRAC = 0.1;

interface State {
  // ── the world around the tile (stimuli) ──
  vm_mv: number; // motor supply on VM (pad 9)
  vplus_mv: number; // logic supply on V+ (pad 10)
  motor_mohm: number; // winding resistance of the attached motor
  motor_kv: number; // back-EMF constant of the attached motor, µV/RPM
  load: number; // mechanical load 0…100 % (100 = locked rotor)

  // ── bridge + regulation (driver setters) ──
  direction: number; // COAST / FORWARD / REVERSE / BRAKE (CONFIG4 IN1/IN2)
  target: number; // WSET_VSET, 0…255
  enabled: number; // EN_OUT (sleep clears it)
  control_mode: number; // drive_dc_h_control_mode_t
  regulation_mode: number; // drive_dc_h_reg_mode_t
  current_imode: number; // drive_dc_h_imode_t
  vm_gain: number; // VM_GAIN_SEL: 1 = 0–3.92 V, 0 = 0–15.7 V
  cs_gain: number; // CS_GAIN_SEL 0…5
  stall_enabled: number; // EN_STALL
  stall_recovery: number; // SMODE: 0 latch off, 1 report only
  inrush_time_ms: number; // TINRUSH
  ripple_threshold: number; // RC_THR × RC_THR_SCALE
  ripple_filter_gain: number; // FLT_GAIN_SEL
  rc_enabled: number; // EN_RC — ripple counter / speed estimator running
  w_scale: number; // W_SCALE (24/40/64/128)
  ripples_per_rev: number; // the driver's cached motor profile

  // ── chip status ──
  fault_byte: number; // FAULT register
  ripple_count: number; // RC_CNT (fractional internally; reads floor & 0xFFFF)

  // ── modeled readings, refreshed each tick ──
  voltage_mv: number;
  current_ma: number;
  speed_raw: number;

  // ── bookkeeping ──
  inrush_remaining_ms: number;
  move_target: number; // move_distance in flight: brake at this count
  last_tick_ms: number;
}

const num = (args: number[], i: number, dflt: number) =>
  args.length > i && Number.isFinite(args[i]) ? Math.trunc(args[i]!) : dflt;
const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

const csMax = (s: State) => CS_MAX_MA[clamp(s.cs_gain, 0, 7)]!;
const voltFs = (s: State) => (s.vm_gain === 1 ? 3920 : 15700);
// The chip is up (VCC above UVLO) — I²C answers, registers hold.
const chipOn = (s: State) => s.vplus_mv >= VCC_UVLO_MV;
// Bridge calls need a READY tile: powered and not put to sleep by the driver.
const ready = (s: State) => chipOn(s) && s.enabled === 1;
const driving = (s: State) => s.direction === FORWARD || s.direction === REVERSE;
// Stall with SMODE = latch: the outputs go Hi-Z until the fault is cleared.
const latchedOff = (s: State) =>
  s.stall_recovery === STALL_LATCH && (s.fault_byte & FLT_STALL) !== 0;

interface MotorOp {
  v: number; // applied terminal voltage, mV
  i: number; // motor current, mA
  rpm: number;
}

// What the bridge puts across the motor and what the motor does about it.
function motorOp(s: State): MotorOp {
  const off = { v: 0, i: 0, rpm: 0 };
  if (!ready(s) || s.control_mode !== CTRL_I2C || !driving(s) || latchedOff(s)) return off;
  if (s.vm_mv < VM_MIN_MV) return off;
  const L = clamp(s.load, 0, 100) / 100;
  const rpmPerMv = (1000 / Math.max(1, s.motor_kv)) * (1 - L);
  let v: number;
  if (s.regulation_mode === REG_SPEED) {
    // PI loop: the voltage that makes SPEED match WSET_VSET, up to VM.
    const targetRpm = radToRpm(s.target * s.w_scale, s.ripples_per_rev);
    v = rpmPerMv > 0 ? Math.min(s.vm_mv, targetRpm / rpmPerMv) : s.vm_mv;
  } else if (s.regulation_mode === REG_VOLTAGE) {
    v = Math.min(s.vm_mv, (s.target * voltFs(s)) / 255);
  } else {
    v = s.vm_mv; // current-regulation modes drive full VM
  }
  const stallMa = (v * 1000) / Math.max(1, s.motor_mohm);
  const i = stallMa * (NO_LOAD_CURRENT_FRAC + (1 - NO_LOAD_CURRENT_FRAC) * L);
  return { v, i, rpm: v * rpmPerMv };
}

// RC_STATUS1: ripple speed (rad/s) / W_SCALE, 8-bit; 0 unless EN_RC.
const speedRaw = (s: State, op: MotorOp) =>
  s.rc_enabled && op.rpm > 0
    ? clamp(Math.round(rpmToRad(op.rpm, s.ripples_per_rev) / s.w_scale), 0, 255)
    : 0;
// VMTR / IMTR round-trips at the register's resolution.
const vmtrMv = (s: State, op: MotorOp) => {
  const raw = clamp(Math.round((op.v * 255) / voltFs(s)), 0, 255);
  return Math.floor((raw * voltFs(s)) / 255);
};
const imtrMa = (s: State, op: MotorOp) => {
  const raw = clamp(Math.round((op.i * 192) / csMax(s)), 0, 255);
  return Math.floor((raw * csMax(s)) / 192);
};

// set_ripple_threshold: RC_THR (10 bit) × the smallest scale that fits.
function quantizeThreshold(count: number): number {
  const c = clamp(count, 0, 0xffff);
  if (c > 65472) return 0x3ff * 64;
  for (const scale of [2, 8, 16, 64]) {
    const v = Math.floor(c / scale);
    if (v <= 0x3ff) return v * scale;
  }
  return 0x3ff * 64;
}

// Engage the bridge; a fresh start re-arms the TINRUSH stall blanking.
function drive(s: State, dir: number): Partial<State> {
  return s.direction === dir
    ? { direction: dir }
    : { direction: dir, inrush_remaining_ms: s.inrush_time_ms };
}

// Stand the electrical ctx in for the supply stimuli when the twin is wired.
function withSupplies(s: State, ctx?: PowerCtx): State {
  if (!ctx) return s;
  const vm = ctx.padVoltage['9'];
  const vcc = ctx.padVoltage['10'];
  return { ...s, vm_mv: vm ?? 0, vplus_mv: vcc ?? 0 };
}

const sim: TileSim<State> = {
  tile: 'Drive.DC.H',

  // After tile_drive_dc_h_init() with no cfg: I²C bridge control, coasting,
  // voltage regulation, VM_GAIN_SEL = 1 (3.92 V range), CS_GAIN_SEL 0 (4 A),
  // target 0xFF, TINRUSH ≈ 100 ms, EN_STALL on, SMODE report, IMODE inrush,
  // FLT_GAIN ×4, EN_RC off, W_SCALE 128, 12 ripples/rev; CLR_FLT → NPOR high.
  // (Datasheet Table 8-29: REG_CTRL / IMODE / SMODE / INT_VREF / PMODE / I2C_BC
  // are writable only while EN_OUT = 0. Since driver 4.3.0 init writes them with
  // the outputs off and turns EN_OUT on last, and the runtime setters drop EN_OUT
  // around the write, so these settings do take effect.)
  defaultState: {
    vm_mv: 5000,
    vplus_mv: 3300,
    motor_mohm: 5000,
    motor_kv: MOTOR_KV_UV_PER_RPM,
    load: 30,

    direction: COAST,
    target: 0xff,
    enabled: 1,
    control_mode: CTRL_I2C,
    regulation_mode: REG_VOLTAGE,
    current_imode: 1,
    vm_gain: 1,
    cs_gain: 0,
    stall_enabled: 1,
    stall_recovery: 1,
    inrush_time_ms: 100,
    ripple_threshold: 0,
    ripple_filter_gain: 1,
    rc_enabled: 0,
    w_scale: 128,
    ripples_per_rev: 12,

    fault_byte: FLT_NPOR,
    ripple_count: 0,

    voltage_mv: 0,
    current_ma: 0,
    speed_raw: 0,

    inrush_remaining_ms: 0,
    move_target: 0,
    last_tick_ms: 0,
  },

  controls: [
    {
      type: 'slider',
      field: 'vm_mv',
      label: 'Motor supply (VM)',
      min: 0,
      max: 11000,
      step: 50,
      unit: 'mV',
      description: 'VM (pad 9), 1.65–11 V. In a wired system the net on pad 9 decides.',
    },
    {
      type: 'slider',
      field: 'motor_mohm',
      label: 'Motor resistance',
      min: 500,
      max: 50000,
      step: 100,
      unit: 'mΩ',
      description: 'Winding resistance: sets the locked-rotor current V/R.',
    },
    {
      type: 'slider',
      field: 'motor_kv',
      label: 'Motor Kv',
      min: 50,
      max: 3000,
      step: 10,
      unit: 'µV/RPM',
      description:
        "Back-EMF constant of the attached motor: sets speed per volt. A placeholder until the program's set_motor_params() gives the real value.",
    },
    {
      type: 'slider',
      field: 'load',
      label: 'Mechanical load',
      min: 0,
      max: 100,
      step: 1,
      unit: '%',
      description: 'More load → more current, less speed. 100 % locks the rotor (stall).',
    },
  ],

  hostCalls: {
    // ── lifecycle ──
    tile_drive_dc_h_find: ({ state }) => ({ scalar: chipOn(state) ? 1 : 0 }),
    tile_drive_dc_h_init: () => ({ scalar: 0 }),
    tile_drive_dc_h_sleep: ({ state }) => (ready(state) ? { nextState: { enabled: 0 } } : {}),
    tile_drive_dc_h_wake: ({ state }) =>
      chipOn(state) && state.enabled === 0 ? { nextState: { enabled: 1 } } : {},

    // ── bridge (no-ops when not ready or in a pad-control mode) ──
    tile_drive_dc_h_forward: ({ state }) =>
      ready(state) && state.control_mode === CTRL_I2C ? { nextState: drive(state, FORWARD) } : {},
    tile_drive_dc_h_reverse: ({ state }) =>
      ready(state) && state.control_mode === CTRL_I2C ? { nextState: drive(state, REVERSE) } : {},
    tile_drive_dc_h_brake: ({ state }) =>
      ready(state) && state.control_mode === CTRL_I2C
        ? { nextState: { direction: BRAKE, move_target: 0 } }
        : {},
    tile_drive_dc_h_coast: ({ state }) =>
      ready(state) && state.control_mode === CTRL_I2C
        ? { nextState: { direction: COAST, move_target: 0 } }
        : {},
    // CONFIG4's IN1/IN2 bits are cleared for every mode → the bridge coasts.
    tile_drive_dc_h_set_control_mode: ({ state, args }) => {
      if (!ready(state)) return {};
      const m = num(args, 0, 0);
      return {
        nextState: {
          control_mode: m === 1 || m === 2 ? m : CTRL_I2C,
          direction: COAST,
          move_target: 0,
        },
      };
    },

    // ── regulation ──
    tile_drive_dc_h_set_target: ({ state, args }) =>
      ready(state) ? { nextState: { target: num(args, 0, 0) & 0xff } } : {},
    tile_drive_dc_h_set_regulation_mode: ({ state, args }) => {
      if (!ready(state)) return {};
      const m = num(args, 0, REG_VOLTAGE) & 0x03;
      // Speed regulation forces EN_RC on (and it stays on after).
      return {
        nextState: m === REG_SPEED ? { regulation_mode: m, rc_enabled: 1 } : { regulation_mode: m },
      };
    },
    tile_drive_dc_h_set_current_regulation_mode: ({ state, args }) =>
      ready(state) ? { nextState: { current_imode: num(args, 0, 1) & 0x03 } } : {},
    tile_drive_dc_h_set_current_sense_gain: ({ state, args }) =>
      ready(state) ? { nextState: { cs_gain: clamp(num(args, 0, 0), 0, 5) } } : {},

    // ── stall detection ──
    tile_drive_dc_h_set_stall_enabled: ({ state, args }) =>
      ready(state) ? { nextState: { stall_enabled: num(args, 0, 1) ? 1 : 0 } } : {},
    tile_drive_dc_h_set_inrush_time_ms: ({ state, args }) => {
      if (!ready(state)) return {};
      // 102.4 µs ticks, 16-bit: store what the register can hold.
      const ticks = Math.min(
        0xffff,
        Math.floor((clamp(num(args, 0, 0), 0, 0xffff) * 10000) / 1024),
      );
      return { nextState: { inrush_time_ms: Math.round((ticks * 1024) / 10000) } };
    },
    tile_drive_dc_h_set_stall_recovery: ({ state, args }) =>
      ready(state) ? { nextState: { stall_recovery: num(args, 0, 1) === 1 ? 1 : 0 } } : {},

    // ── ripple counter ──
    tile_drive_dc_h_set_ripple_threshold: ({ state, args }) =>
      ready(state) ? { nextState: { ripple_threshold: quantizeThreshold(num(args, 0, 0)) } } : {},
    tile_drive_dc_h_set_ripple_filter_gain: ({ state, args }) =>
      ready(state) ? { nextState: { ripple_filter_gain: num(args, 0, 1) & 0x03 } } : {},

    // ── monitoring (not gated on READY in the driver; dead only without VCC) ──
    tile_drive_dc_h_get_fault: ({ state }) => ({ scalar: chipOn(state) ? state.fault_byte : 0 }),
    // CLR_FLT: clears the fault flags; NPOR latches high, CNT_DONE stays (CLR_CNT's).
    tile_drive_dc_h_clear_fault: ({ state }) =>
      chipOn(state)
        ? { nextState: { fault_byte: (state.fault_byte & FLT_CNT_DONE) | FLT_NPOR } }
        : {},
    tile_drive_dc_h_is_stalled: ({ state }) => ({
      scalar: chipOn(state) && state.fault_byte & FLT_STALL ? 1 : 0,
    }),
    tile_drive_dc_h_get_voltage_mv: ({ state }) => ({
      scalar: chipOn(state) ? vmtrMv(state, motorOp(state)) : 0,
    }),
    tile_drive_dc_h_get_current_ma: ({ state }) => ({
      scalar: chipOn(state) ? imtrMa(state, motorOp(state)) : 0,
    }),
    tile_drive_dc_h_get_speed: ({ state }) => ({
      scalar: chipOn(state) ? speedRaw(state, motorOp(state)) : 0,
    }),
    tile_drive_dc_h_get_speed_rpm: ({ state }) => {
      const raw = chipOn(state) ? speedRaw(state, motorOp(state)) : 0;
      // Driver: raw × W_SCALE × 60000 / (6283 × rpr), integer.
      return {
        scalar: Math.floor(
          (raw * state.w_scale * 60000) / (TWO_PI_X1000 * Math.max(1, state.ripples_per_rev)),
        ),
      };
    },
    tile_drive_dc_h_get_ripple_count: ({ state }) => ({
      scalar: chipOn(state) ? Math.floor(state.ripple_count) & 0xffff : 0,
    }),
    // CLR_CNT: zeroes RC_CNT and CNT_DONE.
    tile_drive_dc_h_clear_ripple_count: ({ state }) =>
      chipOn(state)
        ? { nextState: { ripple_count: 0, fault_byte: state.fault_byte & ~FLT_CNT_DONE } }
        : {},

    // ── tier-2 helpers ──
    tile_drive_dc_h_set_speed_rpm: ({ state, args }) => {
      if (!ready(state)) return {};
      const rpm = Math.min(400000, Math.max(0, num(args, 0, 0)));
      const rpr = Math.max(1, state.ripples_per_rev);
      // Finest W_SCALE whose 8-bit WSET still reaches the speed; round. Same
      // integer math as the driver: ripple rad/s = rpm × rpr × 6283 / 60000.
      const rpmRpr = Math.min(400000, rpm * rpr);
      let code = 3;
      let wset = 0xff;
      for (let k = 0; k < 4; k++) {
        const den = 60000 * W_SCALE[k]!;
        const v = Math.floor((rpmRpr * TWO_PI_X1000 + Math.floor(den / 2)) / den);
        if (v <= 0xff) {
          code = k;
          wset = v;
          break;
        }
      }
      if (rpm > 0 && wset === 0) wset = 1;
      const update: Partial<State> = {
        regulation_mode: REG_SPEED,
        rc_enabled: 1,
        w_scale: W_SCALE[code]!,
        target: wset,
      };
      const dir = num(args, 1, 0) === DIR_REVERSE ? REVERSE : FORWARD;
      if (state.control_mode === CTRL_I2C) Object.assign(update, drive(state, dir));
      return { nextState: update };
    },
    tile_drive_dc_h_set_motor_params: ({ state, args }) => {
      if (!ready(state)) return {};
      const rpr = num(args, 1, 12);
      const kv = num(args, 2, 0);
      // INV_R / KMC tune the chip's ripple estimator (not modeled); the profile
      // also describes the attached motor, so a nonzero kv sets the model's Kv.
      return {
        nextState: {
          ripples_per_rev: rpr <= 0 ? 12 : Math.min(255, rpr),
          ...(kv > 0 ? { motor_kv: kv } : {}),
        },
      };
    },
    // The driver blocks until CNT_DONE (or STALL) and then brakes; here the move
    // is armed and the tick brakes it when the count arrives.
    tile_drive_dc_h_move_distance: ({ state, args }) => {
      if (!ready(state) || state.control_mode !== CTRL_I2C) return {};
      const ripples = clamp(num(args, 0, 0), 0, 0xffff);
      if (ripples === 0) return {};
      const thr = quantizeThreshold(ripples);
      const dir = num(args, 1, 0) === DIR_REVERSE ? REVERSE : FORWARD;
      return {
        nextState: {
          ...drive({ ...state, direction: COAST }, dir),
          ripple_threshold: thr,
          rc_enabled: 1,
          ripple_count: 0,
          fault_byte: FLT_NPOR, // CLR_FLT + CLR_CNT
          move_target: Math.max(1, thr),
        },
      };
    },
    tile_drive_dc_h_is_running: ({ state }) => ({
      scalar: ready(state) && state.control_mode === CTRL_I2C && driving(state) ? 1 : 0,
    }),
    // Stopped = two 10 ms samples of the ripple counter agree; that takes more
    // than 10 ms of timeout, and a counter that isn't running never moves.
    tile_drive_dc_h_wait_for_stop: ({ state, args }) => {
      if (!ready(state) || num(args, 0, 0) <= 10) return { scalar: 0 };
      const moving = state.rc_enabled === 1 && motorOp(state).rpm > 0;
      return { scalar: moving ? 0 : 1 };
    },
  },

  provenance: {
    tile_drive_dc_h_find: 'canonical',
    tile_drive_dc_h_init: 'canonical', // defaultState = post-init registers
    tile_drive_dc_h_sleep: 'canonical', // EN_OUT = 0 → outputs Hi-Z
    tile_drive_dc_h_wake: 'canonical',
    tile_drive_dc_h_forward: 'canonical', // IN1=1 IN2=0
    tile_drive_dc_h_reverse: 'canonical',
    tile_drive_dc_h_brake: 'canonical',
    tile_drive_dc_h_coast: 'canonical',
    tile_drive_dc_h_set_control_mode: 'canonical', // clears IN1/IN2 → coast
    tile_drive_dc_h_set_target: 'canonical',
    tile_drive_dc_h_set_regulation_mode: 'canonical', // SPEED forces EN_RC
    tile_drive_dc_h_set_current_regulation_mode: 'inferred', // stored; ITRIP is hardware (VREF/RIPROPI)
    tile_drive_dc_h_set_current_sense_gain: 'canonical',
    tile_drive_dc_h_set_stall_enabled: 'canonical',
    tile_drive_dc_h_set_inrush_time_ms: 'canonical', // 102.4 µs ticks
    tile_drive_dc_h_set_stall_recovery: 'canonical',
    tile_drive_dc_h_set_ripple_threshold: 'canonical', // RC_THR × scale
    tile_drive_dc_h_set_ripple_filter_gain: 'inferred', // stored, no effect modeled
    tile_drive_dc_h_get_fault: 'canonical', // bit layout; only STALL / NPOR / CNT_DONE modeled
    tile_drive_dc_h_clear_fault: 'canonical',
    tile_drive_dc_h_is_stalled: 'canonical',
    tile_drive_dc_h_get_voltage_mv: 'canonical', // VMTR·FS/255 (motor response modeled)
    tile_drive_dc_h_get_current_ma: 'canonical', // IMTR·Imax/192 (motor response modeled)
    tile_drive_dc_h_get_speed: 'inferred', // RC_STATUS1 = ripple Hz / W_SCALE; motor constant modeled
    tile_drive_dc_h_get_speed_rpm: 'canonical', // driver's inverse conversion
    tile_drive_dc_h_get_ripple_count: 'inferred', // integrated from the modeled speed
    tile_drive_dc_h_clear_ripple_count: 'canonical',
    tile_drive_dc_h_set_speed_rpm: 'canonical', // W_SCALE pick + WSET rounding
    tile_drive_dc_h_set_motor_params: 'inferred', // INV_R / KMC not modeled
    tile_drive_dc_h_move_distance: 'inferred', // non-blocking; brakes on the tick
    tile_drive_dc_h_is_running: 'canonical', // CONFIG4 IN1/IN2 pattern
    tile_drive_dc_h_wait_for_stop: 'inferred',
    power: 'inferred', // chip currents canonical; motor current modeled
  },

  // Each tick: count down the inrush blanking, refresh the readings, advance the
  // ripple counter, latch STALL / CNT_DONE, and finish a move_distance.
  deriveState(state, { t }) {
    const update: Partial<State> = { last_tick_ms: t };
    const dt = state.last_tick_ms > 0 ? Math.max(0, t - state.last_tick_ms) : 0;
    const inrush = Math.max(0, state.inrush_remaining_ms - dt);
    if (inrush !== state.inrush_remaining_ms) update.inrush_remaining_ms = inrush;

    const op = motorOp(state);
    const v = vmtrMv(state, op);
    const i = imtrMa(state, op);
    const sp = speedRaw(state, op);
    if (v !== state.voltage_mv) update.voltage_mv = v;
    if (i !== state.current_ma) update.current_ma = i;
    if (sp !== state.speed_raw) update.speed_raw = sp;

    let count = state.ripple_count;
    if (state.rc_enabled && op.rpm > 0 && dt > 0) {
      count = (count + (((op.rpm * state.ripples_per_rev) / 60) * dt) / 1000) % 0x10000;
      update.ripple_count = count;
    }

    let fault = state.fault_byte;
    // Locked rotor, driving, past TINRUSH, stall detection on → STALL.
    if (
      state.stall_enabled &&
      ready(state) &&
      driving(state) &&
      op.v > 0 &&
      op.rpm <= 0 &&
      inrush === 0
    )
      fault |= FLT_STALL | FLT_FAULT;
    if (state.rc_enabled && state.ripple_threshold > 0 && count >= state.ripple_threshold)
      fault |= FLT_CNT_DONE;
    if (fault !== state.fault_byte) update.fault_byte = fault;

    // move_distance's loop exits on CNT_DONE or STALL, then brakes.
    if (state.move_target > 0 && (count >= state.move_target || fault & FLT_STALL)) {
      update.direction = BRAKE;
      update.move_target = 0;
    }
    return update;
  },

  // Bridge terminals (duty 0…1 of VM) and the NPROP current mirror.
  padOutputs(state) {
    const op = motorOp(state);
    const d = state.vm_mv > 0 ? clamp(op.v / state.vm_mv, 0, 1) : 0;
    return {
      '8': state.direction === FORWARD ? d : 0, // chip OUT1
      '7': state.direction === REVERSE ? d : 0, // chip OUT2
      '6': clamp(op.i / csMax(state), 0, 1),
    };
  },

  // Electrical. V+ (pad 10) is the chip's VCC, VM (pad 9) carries the motor
  // current; OUT1 (pad 8) / OUT2 (pad 7) are the bridge terminals. In a wired
  // system VM and V+ come from the nets on pads 9 / 10.
  power(state, ctx) {
    const s = withSupplies(state, ctx);
    const on = chipOn(s);
    const op = motorOp(s);
    const vccUa = on ? IVCC_ACTIVE_UA : 0;
    const vmUa = on && s.vm_mv > 0 ? IVM_ACTIVE_UA + Math.round(op.i * 1000) : 0;
    return {
      draw_ua: vccUa + vmUa,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: s.vplus_mv,
          i_ua: vccUa,
          pads: ['10'],
          note: 'logic supply (VCC)',
        },
        {
          name: 'VM',
          role: 'supply',
          v_mv: s.vm_mv,
          i_ua: vmUa,
          pads: ['9'],
          note:
            op.i > 0
              ? 'motor supply + motor current'
              : on
                ? 'motor supply — idle'
                : 'chip off (VCC)',
        },
        {
          name: 'OUT1',
          role: 'output',
          v_mv: s.direction === FORWARD ? Math.round(op.v) : 0,
          pads: ['8'],
          note: 'H-bridge terminal 1 (chip OUT1 → tile pad 8)',
        },
        {
          name: 'OUT2',
          role: 'output',
          v_mv: s.direction === REVERSE ? Math.round(op.v) : 0,
          pads: ['7'],
          note: 'H-bridge terminal 2 (chip OUT2 → tile pad 7)',
        },
      ],
    };
  },
};

export default sim;
