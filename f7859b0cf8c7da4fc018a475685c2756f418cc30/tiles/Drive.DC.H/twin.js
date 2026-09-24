// src/sims/drive_dc_h.ts
var CS_MAX_MA = [4e3, 2e3, 1e3, 500, 250, 125, 250, 125];
var W_SCALE = [24, 40, 64, 128];
var COAST = 0;
var FORWARD = 1;
var REVERSE = 2;
var BRAKE = 3;
var CTRL_I2C = 0;
var REG_SPEED = 2;
var REG_VOLTAGE = 3;
var DIR_REVERSE = 1;
var STALL_LATCH = 0;
var FLT_FAULT = 128;
var FLT_STALL = 32;
var FLT_NPOR = 2;
var FLT_CNT_DONE = 1;
var IVM_ACTIVE_UA = 1300;
var IVCC_ACTIVE_UA = 1500;
var VCC_UVLO_MV = 1650;
var VM_MIN_MV = 1650;
var MOTOR_KV_UV_PER_RPM = 300;
var NO_LOAD_CURRENT_FRAC = 0.1;
var num = (args, i, dflt) => args.length > i && Number.isFinite(args[i]) ? Math.trunc(args[i]) : dflt;
var clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
var csMax = (s) => CS_MAX_MA[clamp(s.cs_gain, 0, 7)];
var voltFs = (s) => s.vm_gain === 1 ? 3920 : 15700;
var chipOn = (s) => s.vplus_mv >= VCC_UVLO_MV;
var ready = (s) => chipOn(s) && s.enabled === 1;
var driving = (s) => s.direction === FORWARD || s.direction === REVERSE;
var latchedOff = (s) => s.stall_recovery === STALL_LATCH && (s.fault_byte & FLT_STALL) !== 0;
function motorOp(s) {
  const off = { v: 0, i: 0, rpm: 0 };
  if (!ready(s) || s.control_mode !== CTRL_I2C || !driving(s) || latchedOff(s)) return off;
  if (s.vm_mv < VM_MIN_MV) return off;
  const L = clamp(s.load, 0, 100) / 100;
  const rpmPerMv = 1e3 / MOTOR_KV_UV_PER_RPM * (1 - L);
  let v;
  if (s.regulation_mode === REG_SPEED) {
    const targetRpm = s.target * s.w_scale * 60 / Math.max(1, s.ripples_per_rev);
    v = rpmPerMv > 0 ? Math.min(s.vm_mv, targetRpm / rpmPerMv) : s.vm_mv;
  } else if (s.regulation_mode === REG_VOLTAGE) {
    v = Math.min(s.vm_mv, s.target * voltFs(s) / 255);
  } else {
    v = s.vm_mv;
  }
  const stallMa = v * 1e3 / Math.max(1, s.motor_mohm);
  const i = stallMa * (NO_LOAD_CURRENT_FRAC + (1 - NO_LOAD_CURRENT_FRAC) * L);
  return { v, i, rpm: v * rpmPerMv };
}
var speedRaw = (s, op) => s.rc_enabled && op.rpm > 0 ? clamp(Math.round(op.rpm * s.ripples_per_rev / 60 / s.w_scale), 0, 255) : 0;
var vmtrMv = (s, op) => {
  const raw = clamp(Math.round(op.v * 255 / voltFs(s)), 0, 255);
  return Math.floor(raw * voltFs(s) / 255);
};
var imtrMa = (s, op) => {
  const raw = clamp(Math.round(op.i * 192 / csMax(s)), 0, 255);
  return Math.floor(raw * csMax(s) / 192);
};
function quantizeThreshold(count) {
  const c = clamp(count, 0, 65535);
  if (c > 65472) return 1023 * 64;
  for (const scale of [2, 8, 16, 64]) {
    const v = Math.floor(c / scale);
    if (v <= 1023) return v * scale;
  }
  return 1023 * 64;
}
function drive(s, dir) {
  return s.direction === dir ? { direction: dir } : { direction: dir, inrush_remaining_ms: s.inrush_time_ms };
}
function withSupplies(s, ctx) {
  if (!ctx) return s;
  const vm = ctx.padVoltage["9"];
  const vcc = ctx.padVoltage["10"];
  return { ...s, vm_mv: vm ?? 0, vplus_mv: vcc ?? 0 };
}
var sim = {
  tile: "Drive.DC.H",
  // After tile_drive_dc_h_init() with no cfg: I²C bridge control, coasting,
  // voltage regulation, VM_GAIN_SEL = 1 (3.92 V range), CS_GAIN_SEL 0 (4 A),
  // target 0xFF, TINRUSH ≈ 100 ms, EN_STALL on, SMODE report, IMODE inrush,
  // FLT_GAIN ×4, EN_RC off, W_SCALE 128, 12 ripples/rev; CLR_FLT → NPOR high.
  defaultState: {
    vm_mv: 5e3,
    vplus_mv: 3300,
    motor_mohm: 5e3,
    load: 30,
    direction: COAST,
    target: 255,
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
    last_tick_ms: 0
  },
  controls: [
    {
      type: "slider",
      field: "vm_mv",
      label: "Motor supply (VM)",
      min: 0,
      max: 11e3,
      step: 50,
      unit: "mV",
      description: "VM (pad 9), 1.65\u201311 V. In a wired system the net on pad 9 decides."
    },
    {
      type: "slider",
      field: "motor_mohm",
      label: "Motor resistance",
      min: 500,
      max: 5e4,
      step: 100,
      unit: "m\u03A9",
      description: "Winding resistance: sets the locked-rotor current V/R."
    },
    {
      type: "slider",
      field: "load",
      label: "Mechanical load",
      min: 0,
      max: 100,
      step: 1,
      unit: "%",
      description: "More load \u2192 more current, less speed. 100 % locks the rotor (stall)."
    }
  ],
  hostCalls: {
    // ── lifecycle ──
    tile_drive_dc_h_find: ({ state }) => ({ scalar: chipOn(state) ? 1 : 0 }),
    tile_drive_dc_h_init: () => ({ scalar: 0 }),
    tile_drive_dc_h_sleep: ({ state }) => ready(state) ? { nextState: { enabled: 0 } } : {},
    tile_drive_dc_h_wake: ({ state }) => chipOn(state) && state.enabled === 0 ? { nextState: { enabled: 1 } } : {},
    // ── bridge (no-ops when not ready or in a pad-control mode) ──
    tile_drive_dc_h_forward: ({ state }) => ready(state) && state.control_mode === CTRL_I2C ? { nextState: drive(state, FORWARD) } : {},
    tile_drive_dc_h_reverse: ({ state }) => ready(state) && state.control_mode === CTRL_I2C ? { nextState: drive(state, REVERSE) } : {},
    tile_drive_dc_h_brake: ({ state }) => ready(state) && state.control_mode === CTRL_I2C ? { nextState: { direction: BRAKE, move_target: 0 } } : {},
    tile_drive_dc_h_coast: ({ state }) => ready(state) && state.control_mode === CTRL_I2C ? { nextState: { direction: COAST, move_target: 0 } } : {},
    // CONFIG4's IN1/IN2 bits are cleared for every mode → the bridge coasts.
    tile_drive_dc_h_set_control_mode: ({ state, args }) => {
      if (!ready(state)) return {};
      const m = num(args, 0, 0);
      return {
        nextState: {
          control_mode: m === 1 || m === 2 ? m : CTRL_I2C,
          direction: COAST,
          move_target: 0
        }
      };
    },
    // ── regulation ──
    tile_drive_dc_h_set_target: ({ state, args }) => ready(state) ? { nextState: { target: num(args, 0, 0) & 255 } } : {},
    tile_drive_dc_h_set_regulation_mode: ({ state, args }) => {
      if (!ready(state)) return {};
      const m = num(args, 0, REG_VOLTAGE) & 3;
      return {
        nextState: m === REG_SPEED ? { regulation_mode: m, rc_enabled: 1 } : { regulation_mode: m }
      };
    },
    tile_drive_dc_h_set_current_regulation_mode: ({ state, args }) => ready(state) ? { nextState: { current_imode: num(args, 0, 1) & 3 } } : {},
    tile_drive_dc_h_set_current_sense_gain: ({ state, args }) => ready(state) ? { nextState: { cs_gain: clamp(num(args, 0, 0), 0, 5) } } : {},
    // ── stall detection ──
    tile_drive_dc_h_set_stall_enabled: ({ state, args }) => ready(state) ? { nextState: { stall_enabled: num(args, 0, 1) ? 1 : 0 } } : {},
    tile_drive_dc_h_set_inrush_time_ms: ({ state, args }) => {
      if (!ready(state)) return {};
      const ticks = Math.min(
        65535,
        Math.floor(clamp(num(args, 0, 0), 0, 65535) * 1e4 / 1024)
      );
      return { nextState: { inrush_time_ms: Math.round(ticks * 1024 / 1e4) } };
    },
    tile_drive_dc_h_set_stall_recovery: ({ state, args }) => ready(state) ? { nextState: { stall_recovery: num(args, 0, 1) === 1 ? 1 : 0 } } : {},
    // ── ripple counter ──
    tile_drive_dc_h_set_ripple_threshold: ({ state, args }) => ready(state) ? { nextState: { ripple_threshold: quantizeThreshold(num(args, 0, 0)) } } : {},
    tile_drive_dc_h_set_ripple_filter_gain: ({ state, args }) => ready(state) ? { nextState: { ripple_filter_gain: num(args, 0, 1) & 3 } } : {},
    // ── monitoring (not gated on READY in the driver; dead only without VCC) ──
    tile_drive_dc_h_get_fault: ({ state }) => ({ scalar: chipOn(state) ? state.fault_byte : 0 }),
    // CLR_FLT: clears the fault flags; NPOR latches high, CNT_DONE stays (CLR_CNT's).
    tile_drive_dc_h_clear_fault: ({ state }) => chipOn(state) ? { nextState: { fault_byte: state.fault_byte & FLT_CNT_DONE | FLT_NPOR } } : {},
    tile_drive_dc_h_is_stalled: ({ state }) => ({
      scalar: chipOn(state) && state.fault_byte & FLT_STALL ? 1 : 0
    }),
    tile_drive_dc_h_get_voltage_mv: ({ state }) => ({
      scalar: chipOn(state) ? vmtrMv(state, motorOp(state)) : 0
    }),
    tile_drive_dc_h_get_current_ma: ({ state }) => ({
      scalar: chipOn(state) ? imtrMa(state, motorOp(state)) : 0
    }),
    tile_drive_dc_h_get_speed: ({ state }) => ({
      scalar: chipOn(state) ? speedRaw(state, motorOp(state)) : 0
    }),
    tile_drive_dc_h_get_speed_rpm: ({ state }) => {
      const raw = chipOn(state) ? speedRaw(state, motorOp(state)) : 0;
      return {
        scalar: Math.floor(raw * 60 * state.w_scale / Math.max(1, state.ripples_per_rev))
      };
    },
    tile_drive_dc_h_get_ripple_count: ({ state }) => ({
      scalar: chipOn(state) ? Math.floor(state.ripple_count) & 65535 : 0
    }),
    // CLR_CNT: zeroes RC_CNT and CNT_DONE.
    tile_drive_dc_h_clear_ripple_count: ({ state }) => chipOn(state) ? { nextState: { ripple_count: 0, fault_byte: state.fault_byte & ~FLT_CNT_DONE } } : {},
    // ── tier-2 helpers ──
    tile_drive_dc_h_set_speed_rpm: ({ state, args }) => {
      if (!ready(state)) return {};
      const rpm = Math.max(0, num(args, 0, 0));
      const rpr = Math.max(1, state.ripples_per_rev);
      let code = 3;
      let wset = 255;
      for (let k = 0; k < 4; k++) {
        const den = 60 * W_SCALE[k];
        const v = Math.floor((rpm * rpr + den / 2) / den);
        if (v <= 255) {
          code = k;
          wset = v;
          break;
        }
      }
      if (rpm > 0 && wset === 0) wset = 1;
      const update = {
        regulation_mode: REG_SPEED,
        rc_enabled: 1,
        w_scale: W_SCALE[code],
        target: wset
      };
      const dir = num(args, 1, 0) === DIR_REVERSE ? REVERSE : FORWARD;
      if (state.control_mode === CTRL_I2C) Object.assign(update, drive(state, dir));
      return { nextState: update };
    },
    tile_drive_dc_h_set_motor_params: ({ state, args }) => {
      if (!ready(state)) return {};
      const rpr = num(args, 1, 12);
      return { nextState: { ripples_per_rev: rpr <= 0 ? 12 : Math.min(255, rpr) } };
    },
    // The driver blocks until CNT_DONE (or STALL) and then brakes; here the move
    // is armed and the tick brakes it when the count arrives.
    tile_drive_dc_h_move_distance: ({ state, args }) => {
      if (!ready(state) || state.control_mode !== CTRL_I2C) return {};
      const ripples = clamp(num(args, 0, 0), 0, 65535);
      if (ripples === 0) return {};
      const thr = quantizeThreshold(ripples);
      const dir = num(args, 1, 0) === DIR_REVERSE ? REVERSE : FORWARD;
      return {
        nextState: {
          ...drive({ ...state, direction: COAST }, dir),
          ripple_threshold: thr,
          rc_enabled: 1,
          ripple_count: 0,
          fault_byte: FLT_NPOR,
          // CLR_FLT + CLR_CNT
          move_target: Math.max(1, thr)
        }
      };
    },
    tile_drive_dc_h_is_running: ({ state }) => ({
      scalar: ready(state) && state.control_mode === CTRL_I2C && driving(state) ? 1 : 0
    }),
    // Stopped = two 10 ms samples of the ripple counter agree; that takes more
    // than 10 ms of timeout, and a counter that isn't running never moves.
    tile_drive_dc_h_wait_for_stop: ({ state, args }) => {
      if (!ready(state) || num(args, 0, 0) <= 10) return { scalar: 0 };
      const moving = state.rc_enabled === 1 && motorOp(state).rpm > 0;
      return { scalar: moving ? 0 : 1 };
    }
  },
  provenance: {
    tile_drive_dc_h_find: "canonical",
    tile_drive_dc_h_init: "canonical",
    // defaultState = post-init registers
    tile_drive_dc_h_sleep: "canonical",
    // EN_OUT = 0 → outputs Hi-Z
    tile_drive_dc_h_wake: "canonical",
    tile_drive_dc_h_forward: "canonical",
    // IN1=1 IN2=0
    tile_drive_dc_h_reverse: "canonical",
    tile_drive_dc_h_brake: "canonical",
    tile_drive_dc_h_coast: "canonical",
    tile_drive_dc_h_set_control_mode: "canonical",
    // clears IN1/IN2 → coast
    tile_drive_dc_h_set_target: "canonical",
    tile_drive_dc_h_set_regulation_mode: "canonical",
    // SPEED forces EN_RC
    tile_drive_dc_h_set_current_regulation_mode: "inferred",
    // stored; ITRIP is hardware (VREF/RIPROPI)
    tile_drive_dc_h_set_current_sense_gain: "canonical",
    tile_drive_dc_h_set_stall_enabled: "canonical",
    tile_drive_dc_h_set_inrush_time_ms: "canonical",
    // 102.4 µs ticks
    tile_drive_dc_h_set_stall_recovery: "canonical",
    tile_drive_dc_h_set_ripple_threshold: "canonical",
    // RC_THR × scale
    tile_drive_dc_h_set_ripple_filter_gain: "inferred",
    // stored, no effect modeled
    tile_drive_dc_h_get_fault: "canonical",
    // bit layout; only STALL / NPOR / CNT_DONE modeled
    tile_drive_dc_h_clear_fault: "canonical",
    tile_drive_dc_h_is_stalled: "canonical",
    tile_drive_dc_h_get_voltage_mv: "canonical",
    // VMTR·FS/255 (motor response modeled)
    tile_drive_dc_h_get_current_ma: "canonical",
    // IMTR·Imax/192 (motor response modeled)
    tile_drive_dc_h_get_speed: "inferred",
    // RC_STATUS1 = ripple Hz / W_SCALE; motor constant modeled
    tile_drive_dc_h_get_speed_rpm: "canonical",
    // driver's inverse conversion
    tile_drive_dc_h_get_ripple_count: "inferred",
    // integrated from the modeled speed
    tile_drive_dc_h_clear_ripple_count: "canonical",
    tile_drive_dc_h_set_speed_rpm: "canonical",
    // W_SCALE pick + WSET rounding
    tile_drive_dc_h_set_motor_params: "inferred",
    // INV_R / KMC not modeled
    tile_drive_dc_h_move_distance: "inferred",
    // non-blocking; brakes on the tick
    tile_drive_dc_h_is_running: "canonical",
    // CONFIG4 IN1/IN2 pattern
    tile_drive_dc_h_wait_for_stop: "inferred",
    power: "inferred"
    // chip currents canonical; motor current modeled
  },
  // Each tick: count down the inrush blanking, refresh the readings, advance the
  // ripple counter, latch STALL / CNT_DONE, and finish a move_distance.
  deriveState(state, { t }) {
    const update = { last_tick_ms: t };
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
      count = (count + op.rpm * state.ripples_per_rev / 60 * dt / 1e3) % 65536;
      update.ripple_count = count;
    }
    let fault = state.fault_byte;
    if (state.stall_enabled && ready(state) && driving(state) && op.v > 0 && op.rpm <= 0 && inrush === 0)
      fault |= FLT_STALL | FLT_FAULT;
    if (state.rc_enabled && state.ripple_threshold > 0 && count >= state.ripple_threshold)
      fault |= FLT_CNT_DONE;
    if (fault !== state.fault_byte) update.fault_byte = fault;
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
      "8": state.direction === FORWARD ? d : 0,
      // chip OUT1
      "7": state.direction === REVERSE ? d : 0,
      // chip OUT2
      "6": clamp(op.i / csMax(state), 0, 1)
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
    const vmUa = on && s.vm_mv > 0 ? IVM_ACTIVE_UA + Math.round(op.i * 1e3) : 0;
    return {
      draw_ua: vccUa + vmUa,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: s.vplus_mv,
          i_ua: vccUa,
          pads: ["10"],
          note: "logic supply (VCC)"
        },
        {
          name: "VM",
          role: "supply",
          v_mv: s.vm_mv,
          i_ua: vmUa,
          pads: ["9"],
          note: op.i > 0 ? "motor supply + motor current" : on ? "motor supply \u2014 idle" : "chip off (VCC)"
        },
        {
          name: "OUT1",
          role: "output",
          v_mv: s.direction === FORWARD ? Math.round(op.v) : 0,
          pads: ["8"],
          note: "H-bridge terminal 1 (chip OUT1 \u2192 tile pad 8)"
        },
        {
          name: "OUT2",
          role: "output",
          v_mv: s.direction === REVERSE ? Math.round(op.v) : 0,
          pads: ["7"],
          note: "H-bridge terminal 2 (chip OUT2 \u2192 tile pad 7)"
        }
      ]
    };
  }
};
var drive_dc_h_default = sim;
export {
  drive_dc_h_default as default
};
