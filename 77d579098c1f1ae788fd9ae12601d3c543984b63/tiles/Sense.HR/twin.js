// src/sims/sense_hr.ts
var SLOTS = 6;
var FRAME_LEN = 12;
var FIFO_DEPTH = 256;
var FS = 524287;
var NO_SAMPLE = -2147483648;
var FR_CLK_HZ = 32768;
var PROX_FPS = 8;
var LED_LSB_UA = [125, 250, 375, 500];
var ADC_FS_UA = [4, 8, 16, 32];
var DACOFF_UA = [0, 4, 8, 12, 16, 20, 24, 28];
var TINT_US = [14.6, 29.2, 58.6, 117.1];
var ST_A_FULL = 128;
var ST_FRAME_RDY = 64;
var ST_FIFO_DATA_RDY = 32;
var ST_EXP_OVF = 8;
var ST_THRESH1 = 2;
var TAG_EXP_OVF = 14;
var LED_IR = 0;
var LED_GREEN = 1;
var LED_RED = 2;
var CROSSTALK_NA_PER_MA = 2200;
var RETURN_NA_PER_MA = [2600, 1300, 1870, 1300];
var LED_VF_MV = [1700, 3300, 2100, 3300];
var LED_COMPLIANCE_MV = 287;
var sf = (i, k) => `s${i}_${k}`;
var slot = (s, i, k) => s[sf(i, k)] ?? 0;
var arg = (args, i, fallback) => args.length > i && Number.isFinite(args[i]) ? args[i] : fallback;
var validSlot = (n) => Number.isInteger(n) && n >= 0 && n < SLOTS;
function ledQuantise(ua) {
  if (ua <= 0) return { pa: 0 };
  const u = Math.min(ua, 1e5);
  for (let r = 0; r < 4; r++) {
    const lsb = LED_LSB_UA[r];
    if (u <= 255 * lsb) {
      const code = Math.min(255, Math.max(1, Math.floor((u + Math.floor(lsb / 2)) / lsb)));
      return { rge: r, pa: code };
    }
  }
  return { rge: 3, pa: 255 };
}
var ledUa = (s, i) => slot(s, i, "pa") * LED_LSB_UA[slot(s, i, "rge") & 3];
var framePeriodMs = (s) => s.in_prox ? 1e3 / PROX_FPS : s.fr_div * 1e3 / FR_CLK_HZ;
var frameRate = (s) => s.fr_div ? Math.floor((FR_CLK_HZ + s.fr_div / 2) / s.fr_div) : 0;
function activeSlots(s) {
  if (s.in_prox) return [5];
  const out = [];
  for (let i = 0; i < SLOTS; i++) if (s.enable_mask & 1 << i) out.push(i);
  if (s.prox_auto && !(s.enable_mask & 32)) out.push(5);
  return out;
}
var channels = (s) => s.ppg2_enabled ? 2 : 1;
function driverFrameSamples(s) {
  let n = 0;
  for (let i = 0; i < SLOTS; i++) if (s.enable_mask & 1 << i) n++;
  if (s.prox_auto && !(s.enable_mask & 32)) n++;
  return Math.max(1, n * channels(s));
}
function pulse(s, tMs) {
  return 0.5 * (1 + Math.sin(2 * Math.PI * s.heart_rate_bpm * tMs / 6e4));
}
function photocurrentNa(s, i, tMs, withAc) {
  if (slot(s, i, "amb")) return s.ambient_ua * 1e3;
  const led = slot(s, i, "led") & 3;
  const ma = ledUa(s, i) / 1e3;
  let tissue = s.finger_present ? RETURN_NA_PER_MA[led] * ma : 0;
  if (withAc && s.finger_present) {
    const pi = s.perfusion_pct / 100;
    const ratio = Math.max(0.2, (110 - s.spo2_pct) / 25);
    const ac = led === LED_IR ? pi : led === LED_RED ? pi * ratio : 2 * pi;
    tissue *= 1 - ac * pulse(s, tMs);
  }
  return CROSSTALK_NA_PER_MA * ma + tissue;
}
function pdShare(pdsel, ch) {
  if (pdsel === 0) return ch === 0 ? 1 : 0;
  if (pdsel === 3) return ch === 0 ? 0 : 1;
  return 0.5;
}
function sampleCounts(s, i, ch, tMs, withAc) {
  const na = photocurrentNa(s, i, tMs, withAc) * pdShare(slot(s, i, "pdsel") & 3, ch);
  const counts = Math.round(
    (na - DACOFF_UA[slot(s, i, "dacoff") & 7] * 1e3) / (ADC_FS_UA[slot(s, i, "adc") & 3] * 1e3) * FS
  );
  return counts > FS || counts < -FS - 1 ? null : counts;
}
function drain(s) {
  const out = { fifo_count: 0, fifo_ovf: 0 };
  for (let k = 0; k < FRAME_LEN; k++) out[`f${k}`] = NO_SAMPLE;
  const perFrame = activeSlots(s).length * channels(s);
  const frames = perFrame ? Math.floor(s.fifo_count / perFrame) : 0;
  let ovf = 0;
  if (frames > 0) {
    for (const i of activeSlots(s)) {
      for (let ch = 0; ch < channels(s); ch++) {
        const v = sampleCounts(s, i, ch, s.last_frame_t, true);
        if (v === null) ovf += frames;
        else out[`f${i * 2 + ch}`] = v;
      }
    }
  }
  out.ovf_count = ovf;
  out.status1 = s.status1 & ~(ST_A_FULL | ST_FRAME_RDY | ST_FIFO_DATA_RDY);
  return out;
}
function cached(s, i, ch) {
  if (!validSlot(i)) return { value: NO_SAMPLE };
  if (s.fifo_count >= driverFrameSamples(s)) {
    const next = drain(s);
    const k = `f${i * 2 + ch}`;
    return { value: next[k] ?? s[k], next };
  }
  return { value: s[`f${i * 2 + ch}`] };
}
function probe(s, i) {
  const v = sampleCounts(s, i, 0, 0, false);
  return v === null ? { n: 0, dc: 0, ovf: 99 } : { n: 1, dc: v, ovf: 0 };
}
function autoGain(s0, i, maxUa) {
  if (!validSlot(i) || slot(s0, i, "amb")) return { ok: 0, next: {} };
  const max = maxUa || 1e4;
  const paCap = Math.max(1, max > 255 * 125 ? 255 : Math.floor(max / 125));
  const s = { ...s0 };
  s[sf(i, "rge")] = 0;
  s[sf(i, "pa")] = Math.min(paCap, 8);
  s[sf(i, "dacoff")] = 0;
  s[sf(i, "adc")] = 3;
  const g = (k) => s[sf(i, k)];
  const set = (k, v) => {
    s[sf(i, k)] = v;
  };
  for (let iter = 0; iter < 14; iter++) {
    const { n: n2, dc: dc2, ovf } = probe(s, i);
    if (ovf > 4 || n2 && dc2 > FS * 3 / 4) {
      if (g("dacoff") < 7) set("dacoff", g("dacoff") + 1);
      else if (g("pa") > 4) set("pa", Math.floor(g("pa") * 3 / 4));
      else break;
    } else if (n2 && g("pa") < paCap) {
      set("pa", Math.min(paCap, g("pa") + Math.floor(g("pa") / 2) + 1));
    } else break;
  }
  let { n, dc } = probe(s, i);
  const found = n !== 0 ? 1 : 0;
  const totalX100 = n ? Math.trunc(dc * ADC_FS_UA[g("adc")] * 100 / FS) + DACOFF_UA[g("dacoff")] * 100 : 0;
  if (n && totalX100 > 0) {
    const iX100 = totalX100;
    let best = 0;
    for (let d = 0; d < 8; d++) if (DACOFF_UA[d] * 100 + 50 <= iX100) best = d;
    set("dacoff", best);
    const resid = iX100 - DACOFF_UA[best] * 100;
    for (let r = 0; r < 4; r++) {
      if (resid * 10 < ADC_FS_UA[r] * 700) {
        set("adc", r);
        break;
      }
    }
    for (let guard = 0; guard < 4; guard++) {
      ({ n, dc } = probe(s, i));
      const ovf = n ? 0 : 99;
      if (n && ovf <= 4 && dc > 0 && dc < FS * 3 / 4) break;
      if (g("adc") < 3) set("adc", g("adc") + 1);
      else if (g("dacoff") > 0) set("dacoff", g("dacoff") - 1);
      else break;
    }
  }
  const next = {
    [sf(i, "rge")]: g("rge"),
    [sf(i, "pa")]: g("pa"),
    [sf(i, "dacoff")]: g("dacoff"),
    [sf(i, "adc")]: g("adc")
  };
  next.fifo_count = 0;
  next.fifo_ovf = 0;
  if (s0.running) next.restart_pending = 1;
  return { ok: found, next };
}
function status2(s) {
  let v = invalidCfg(s) ? 128 : 0;
  for (const i of activeSlots(s)) {
    if (slot(s, i, "amb") || ledUa(s, i) <= 0) continue;
    const led = slot(s, i, "led") & 3;
    if (s.vled_mv < (LED_VF_MV[led] ?? 3300) + LED_COMPLIANCE_MV) v |= 1 << led;
  }
  return v;
}
function invalidCfg(s) {
  let us = 0;
  for (const i of activeSlots(s))
    us += (1 << (slot(s, i, "aver") & 7)) * (TINT_US[slot(s, i, "tint") & 3] + 11);
  return us > framePeriodMs(s) * 1e3;
}
var IDD_1 = [
  [0, 1.5],
  [8, 3.5],
  [64, 17],
  [512, 95]
];
var IDD_4 = [
  [0, 1.5],
  [8, 6.5],
  [64, 40],
  [512, 300]
];
var IDD_1_DUAL = [
  [0, 1.5],
  [8, 4.5],
  [64, 24],
  [512, 150]
];
var IDD_4_DUAL = [
  [0, 1.5],
  [8, 9.5],
  [64, 60],
  [512, 480]
];
function interp(pts, x) {
  for (let k = 1; k < pts.length; k++) {
    const [x0, y0] = pts[k - 1];
    const [x1, y1] = pts[k];
    if (x <= x1 || k === pts.length - 1) return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
  }
  return pts[0][1];
}
var IDD_SHDN_UA = 1;
var IDD_IDLE_UA = 1.5;
var ILED_SHDN_UA = 0.5;
var MEAS_OVERHEAD_US = 70;
var LED_OVERHEAD_US = 11;
function iddUa(s) {
  if (s.shdn) return IDD_SHDN_UA;
  if (!s.running) return IDD_IDLE_UA;
  const fps = s.in_prox ? PROX_FPS : frameRate(s);
  const dual = s.ppg2_enabled === 1;
  const one = interp(dual ? IDD_1_DUAL : IDD_1, fps);
  const four = interp(dual ? IDD_4_DUAL : IDD_4, fps);
  const perMeas = (four - one) / 3;
  const base = one - perMeas;
  let meas = 0;
  for (const i of activeSlots(s)) {
    const pulses = 1 << (slot(s, i, "aver") & 7);
    meas += (pulses * TINT_US[slot(s, i, "tint") & 3] + MEAS_OVERHEAD_US) / (14.6 + MEAS_OVERHEAD_US);
  }
  return Math.round(base + perMeas * meas);
}
function iledUa(s) {
  if (s.shdn || !s.running) return ILED_SHDN_UA;
  const fps = s.in_prox ? PROX_FPS : frameRate(s);
  let ua = 0;
  for (const i of activeSlots(s)) {
    if (slot(s, i, "amb")) continue;
    const pulses = 1 << (slot(s, i, "aver") & 7);
    ua += ledUa(s, i) * fps * pulses * (TINT_US[slot(s, i, "tint") & 3] + LED_OVERHEAD_US) * 1e-6;
  }
  return Math.round(ua + ILED_SHDN_UA);
}
function buildDefault() {
  const s = {
    finger_present: 1,
    heart_rate_bpm: 72,
    perfusion_pct: 2,
    spo2_pct: 98,
    ambient_ua: 0.5,
    vled_mv: 5e3,
    last_reg: 0,
    last_reg_value: 0,
    tile_ready: 1,
    running: 0,
    shdn: 0,
    restart_pending: 1,
    fr_div: 256,
    // 32768 / 128 fps
    enable_mask: 1,
    ppg2_enabled: 0,
    prox_auto: 0,
    in_prox: 0,
    int_en1: 0,
    status1: 0,
    thresh_slot: 0,
    thresh_lower: 0,
    thresh_upper: 255,
    a_full_free: 127,
    fifo_count: 0,
    fifo_ovf: 0,
    ovf_count: 0,
    last_frame_t: 0,
    frame_seq: 0
  };
  for (let i = 0; i < SLOTS; i++) {
    s[sf(i, "led")] = 0;
    s[sf(i, "amb")] = 0;
    s[sf(i, "rge")] = 0;
    s[sf(i, "pa")] = 0;
    s[sf(i, "dacoff")] = 0;
    s[sf(i, "adc")] = 3;
    s[sf(i, "tint")] = 3;
    s[sf(i, "aver")] = 2;
    s[sf(i, "pdsel")] = 0;
  }
  s[sf(0, "led")] = LED_GREEN;
  s[sf(0, "pa")] = 8;
  for (let k = 0; k < FRAME_LEN; k++) s[`f${k}`] = NO_SAMPLE;
  return s;
}
var slotSetter = (field, map) => ({ args }) => {
  const i = arg(args, 0, -1);
  if (!validSlot(i)) return;
  return {
    nextState: { [sf(i, field)]: map(arg(args, 1, 0)) }
  };
};
var sim = {
  tile: "Sense.HR",
  defaultState: buildDefault(),
  controls: [
    {
      type: "slider",
      field: "ambient_ua",
      label: "Ambient light",
      min: 0,
      max: 30,
      step: 0.5,
      unit: "\xB5A",
      description: "Ambient photocurrent. Analog ALC removes it from LED slots; an ambient slot reads it."
    }
  ],
  // What a person does: put a finger on it (or not), and have a pulse.
  stimuli: [
    {
      id: "finger",
      label: "Finger",
      controls: [
        {
          kind: "toggle",
          id: "finger_present",
          label: "finger on sensor",
          fields: ["finger_present"]
        },
        {
          kind: "slider",
          id: "heart_rate_bpm",
          label: "heart rate",
          field: "heart_rate_bpm",
          min: 30,
          max: 220,
          step: 1,
          unit: "bpm"
        },
        {
          kind: "slider",
          id: "perfusion_pct",
          label: "perfusion",
          field: "perfusion_pct",
          min: 0,
          max: 10,
          step: 0.1,
          unit: "%"
        },
        {
          kind: "slider",
          id: "spo2_pct",
          label: "SpO\u2082",
          field: "spo2_pct",
          min: 70,
          max: 100,
          step: 1,
          unit: "%"
        }
      ]
    },
    {
      id: "supply",
      label: "LED supply",
      controls: [
        {
          kind: "slider",
          id: "vled_mv",
          label: "VLED",
          field: "vled_mv",
          min: 2700,
          max: 5500,
          step: 100,
          unit: "mV"
        }
      ]
    }
  ],
  hostCalls: {
    // ── lifecycle ──
    tile_sense_hr_find: () => ({ scalar: 1 }),
    // PART_ID 0x40
    tile_sense_hr_init: () => ({ scalar: 0, nextState: buildDefault() }),
    tile_sense_hr_sleep: () => ({ nextState: { shdn: 1, tile_ready: 2 } }),
    tile_sense_hr_wake: () => ({
      nextState: { shdn: 0, tile_ready: 1, restart_pending: 1 }
    }),
    // Soft reset: registers to POR (engine off, FIFO empty); tile state NONE.
    tile_sense_hr_reset: () => ({
      nextState: {
        running: 0,
        shdn: 0,
        tile_ready: 0,
        fifo_count: 0,
        fifo_ovf: 0,
        status1: 0,
        prox_auto: 0,
        in_prox: 0,
        int_en1: 0
      }
    }),
    tile_sense_hr_start: () => ({
      nextState: { running: 1, fifo_count: 0, fifo_ovf: 0, restart_pending: 1 }
    }),
    tile_sense_hr_stop: () => ({ nextState: { running: 0, in_prox: 0 } }),
    // ── events (the callback itself is not modeled) ──
    tile_sense_hr_process: ({ state }) => {
      if (state.tile_ready !== 1) return;
      return { nextState: { status1: 0 } };
    },
    tile_sense_hr_on_event: () => void 0,
    tile_sense_hr_set_interrupts: ({ args }) => ({
      nextState: { int_en1: arg(args, 0, 0) & 254 }
    }),
    // ── configuration ──
    tile_sense_hr_set_frame_rate: ({ state, args }) => {
      const fps = Math.max(1, arg(args, 0, 1));
      const div = Math.min(
        32766,
        Math.max(16, Math.floor((FR_CLK_HZ + Math.floor(fps / 2)) / fps))
      );
      return {
        scalar: frameRate({ ...state, fr_div: div }),
        nextState: { fr_div: div }
      };
    },
    tile_sense_hr_get_frame_rate: ({ state }) => ({ scalar: frameRate(state) }),
    tile_sense_hr_set_slot: ({ state, args }) => {
      const i = arg(args, 0, -1);
      if (!validSlot(i)) return;
      const q = ledQuantise(arg(args, 2, 0));
      const next = {
        [sf(i, "led")]: arg(args, 1, 0) & 3,
        [sf(i, "amb")]: 0,
        [sf(i, "pa")]: q.pa,
        enable_mask: state.enable_mask | 1 << i
      };
      if (q.rge !== void 0) next[sf(i, "rge")] = q.rge;
      return { nextState: next };
    },
    tile_sense_hr_set_ambient_slot: ({ state, args }) => {
      const i = arg(args, 0, -1);
      if (!validSlot(i)) return;
      return {
        nextState: {
          [sf(i, "amb")]: 1,
          [sf(i, "pa")]: 0,
          enable_mask: state.enable_mask | 1 << i
        }
      };
    },
    tile_sense_hr_enable_slot: ({ state, args }) => {
      const i = arg(args, 0, -1);
      if (!validSlot(i)) return;
      const mask = arg(args, 1, 0) ? state.enable_mask | 1 << i : state.enable_mask & ~(1 << i);
      return { nextState: { enable_mask: mask } };
    },
    tile_sense_hr_set_led_current: ({ args }) => {
      const i = arg(args, 0, -1);
      if (!validSlot(i)) return;
      const q = ledQuantise(arg(args, 1, 0));
      const next = { [sf(i, "pa")]: q.pa };
      if (q.rge !== void 0) next[sf(i, "rge")] = q.rge;
      return { nextState: next };
    },
    tile_sense_hr_get_led_current: ({ state, args }) => {
      const i = arg(args, 0, -1);
      return { scalar: validSlot(i) ? ledUa(state, i) : 0 };
    },
    tile_sense_hr_set_integration: slotSetter("tint", (v) => v & 3),
    tile_sense_hr_set_averaging: slotSetter("aver", (v) => v & 7),
    tile_sense_hr_set_adc_range: slotSetter("adc", (v) => v & 3),
    tile_sense_hr_set_dac_offset: slotSetter(
      "dacoff",
      (v) => Math.floor((Math.min(v & 255, 28) + 2) / 4)
    ),
    tile_sense_hr_set_photodiodes: slotSetter("pdsel", (v) => v & 3),
    tile_sense_hr_set_ppg2_enabled: ({ args }) => ({
      nextState: { ppg2_enabled: arg(args, 0, 0) ? 1 : 0 }
    }),
    tile_sense_hr_auto_gain: ({ state, args }) => {
      const { ok, next } = autoGain(state, arg(args, 0, -1), arg(args, 1, 0));
      return { scalar: ok, nextState: next };
    },
    // ── runtime ──
    tile_sense_hr_get_status: ({ state }) => ({
      scalar: state.status1,
      nextState: { status1: 0 }
    }),
    tile_sense_hr_get_status2: ({ state }) => ({
      scalar: status2(state)
    }),
    tile_sense_hr_frame_ready: ({ state }) => ({
      scalar: state.fifo_count >= driverFrameSamples(state) ? 1 : 0
    }),
    tile_sense_hr_get_fifo_count: ({ state }) => ({ scalar: state.fifo_count }),
    tile_sense_hr_get_fifo_overflow: ({ state }) => ({
      scalar: state.fifo_ovf
    }),
    tile_sense_hr_flush_fifo: () => ({
      nextState: { fifo_count: 0, fifo_ovf: 0 }
    }),
    tile_sense_hr_set_fifo_watermark: ({ args }) => {
      const n = Math.min(FIFO_DEPTH, Math.max(1, arg(args, 0, 1)));
      return { nextState: { a_full_free: FIFO_DEPTH - n } };
    },
    // frame[2·slot + channel]; 1 when a full frame was drained.
    tile_sense_hr_read_frame: ({ state }) => {
      const ok = state.fifo_count >= driverFrameSamples(state);
      const next = ok ? drain(state) : {};
      const merged = { ...state, ...next };
      const frame = [];
      for (let k = 0; k < FRAME_LEN; k++) frame.push(merged[`f${k}`]);
      return { scalar: ok ? 1 : 0, array: frame, nextState: next };
    },
    tile_sense_hr_read_ppg: ({ state, args }) => {
      const r = cached(state, arg(args, 0, -1), 0);
      return { scalar: r.value, nextState: r.next };
    },
    tile_sense_hr_read_ppg2: ({ state, args }) => {
      const r = cached(state, arg(args, 0, -1), 1);
      return { scalar: r.value, nextState: r.next };
    },
    tile_sense_hr_read_photocurrent_na: ({ state, args }) => {
      const i = arg(args, 0, -1);
      const r = cached(state, i, 0);
      if (r.value === NO_SAMPLE) return { scalar: NO_SAMPLE, nextState: r.next };
      const na = Math.trunc(r.value * ADC_FS_UA[slot(state, i, "adc") & 3] * 1e3 / FS) + DACOFF_UA[slot(state, i, "dacoff") & 7] * 1e3;
      return { scalar: na, nextState: r.next };
    },
    tile_sense_hr_get_overflow_count: ({ state }) => ({
      scalar: state.ovf_count
    }),
    // Raw 24-bit words [tag 23:20 | data 19:0] of the newest frame, repeated
    // oldest-first for the frames waiting; slot order, PPG1 then PPG2.
    tile_sense_hr_read_fifo_raw: ({ state, caps }) => {
      const cap = Math.min(caps?.buf ?? 0, state.fifo_count);
      const words = [];
      const perFrame = [];
      for (const i of activeSlots(state))
        for (let ch = 0; ch < channels(state); ch++) {
          const v = sampleCounts(state, i, ch, state.last_frame_t, true);
          perFrame.push(v === null ? TAG_EXP_OVF << 20 : ch * SLOTS + i << 20 | v & 1048575);
        }
      for (let k = 0; perFrame.length && words.length < cap; k++)
        words.push(perFrame[k % perFrame.length]);
      return {
        out: { buf: words },
        nextState: { fifo_count: state.fifo_count - words.length }
      };
    },
    // ── proximity / threshold ──
    tile_sense_hr_set_threshold: ({ args }) => {
      const i = arg(args, 0, -1);
      if (!validSlot(i)) return;
      return {
        nextState: {
          thresh_slot: i + 1,
          thresh_lower: arg(args, 1, 0) & 255,
          thresh_upper: arg(args, 2, 0) & 255
        }
      };
    },
    // Enabling writes THRESH1_LO; disabling leaves it.
    tile_sense_hr_set_proximity_mode: ({ args }) => arg(args, 0, 0) ? { nextState: { prox_auto: 1, thresh_lower: arg(args, 1, 0) & 255 } } : { nextState: { prox_auto: 0, in_prox: 0 } },
    // ── raw registers: identity and status modeled, the rest reads 0 ──
    tile_sense_hr_read_reg: ({ state, args }) => {
      switch (arg(args, 0, 0) & 255) {
        case 0:
          return { scalar: state.status1, nextState: { status1: 0 } };
        case 1:
          return { scalar: status2(state) };
        case 5:
          return {
            scalar: (state.fifo_count & 256 ? 128 : 0) | state.fifo_ovf & 127
          };
        case 6:
          return { scalar: state.fifo_count & 255 };
        case 18:
          return { scalar: state.running ? state.enable_mask & 63 : 0 };
        case 27:
          return { scalar: state.fr_div >> 8 & 127 };
        case 28:
          return { scalar: state.fr_div & 255 };
        case 255:
          return { scalar: 64 };
        default:
          return {
            scalar: (arg(args, 0, 0) & 255) === state.last_reg ? state.last_reg_value : 0
          };
      }
    },
    // Stored for read_reg; the modeled chip state is set through the typed
    // calls, so a raw write does not change sampling behavior.
    tile_sense_hr_write_reg: ({ args }) => ({
      nextState: { last_reg: arg(args, 0, 0) & 255, last_reg_value: arg(args, 1, 0) & 255 }
    })
  },
  provenance: {
    tile_sense_hr_find: "canonical",
    // PART_ID 0x40 at 0x6B
    tile_sense_hr_init: "inferred",
    tile_sense_hr_sleep: "canonical",
    // SHDN, < 1 µA
    tile_sense_hr_wake: "inferred",
    tile_sense_hr_reset: "inferred",
    tile_sense_hr_start: "inferred",
    tile_sense_hr_stop: "inferred",
    tile_sense_hr_process: "inferred",
    tile_sense_hr_on_event: "inferred",
    tile_sense_hr_set_interrupts: "canonical",
    // INT_EN1, PWR_RDY non-maskable
    tile_sense_hr_set_frame_rate: "canonical",
    // FR_CLK_DIV = 32768 / fps, 16..32766
    tile_sense_hr_get_frame_rate: "canonical",
    tile_sense_hr_set_slot: "canonical",
    // led_quantise: 125/250/375/500 µA per code
    tile_sense_hr_set_ambient_slot: "inferred",
    tile_sense_hr_enable_slot: "inferred",
    tile_sense_hr_set_led_current: "canonical",
    tile_sense_hr_get_led_current: "canonical",
    tile_sense_hr_set_integration: "canonical",
    tile_sense_hr_set_averaging: "canonical",
    tile_sense_hr_set_adc_range: "canonical",
    // 4/8/16/32 µA full scale
    tile_sense_hr_set_dac_offset: "canonical",
    // 4 µA steps, 0..28
    tile_sense_hr_set_photodiodes: "inferred",
    // split routing modeled as half the summed current
    tile_sense_hr_set_ppg2_enabled: "inferred",
    tile_sense_hr_auto_gain: "inferred",
    // the driver's algorithm replayed on the DC level
    tile_sense_hr_get_status: "inferred",
    tile_sense_hr_get_status2: "inferred",
    // COMPB from Vf + compliance vs VLED; INVALID_CFG from a fitted per-pulse time (no datasheet formula)
    tile_sense_hr_frame_ready: "inferred",
    tile_sense_hr_get_fifo_count: "inferred",
    tile_sense_hr_get_fifo_overflow: "inferred",
    tile_sense_hr_flush_fifo: "inferred",
    tile_sense_hr_set_fifo_watermark: "canonical",
    // FIFO_A_FULL = free slots
    tile_sense_hr_read_frame: "inferred",
    // optics from the bench (green; no barrier), IR/red scaled
    tile_sense_hr_read_ppg: "inferred",
    tile_sense_hr_read_ppg2: "inferred",
    tile_sense_hr_read_photocurrent_na: "inferred",
    // the driver's conversion, placeholder optics under it
    tile_sense_hr_get_overflow_count: "inferred",
    tile_sense_hr_read_fifo_raw: "inferred",
    // word format canonical, sample order assumed
    tile_sense_hr_set_threshold: "inferred",
    tile_sense_hr_set_proximity_mode: "inferred",
    tile_sense_hr_read_reg: "inferred",
    tile_sense_hr_write_reg: "inferred",
    // last write echoes through read_reg, not applied to the modeled chip
    power: "inferred"
    // datasheet IDD / ILED rows, scaled to the configuration (see header of power)
  },
  // The frame engine: one frame per period while running and awake. Each frame
  // pushes its samples into the FIFO (rolling over when full), sets the STATUS1
  // flags, evaluates threshold 1 and, under PROX_AUTO, proximity.
  deriveState(state, { t }) {
    if (!state.running || state.shdn) return {};
    if (state.restart_pending) return { restart_pending: 0, last_frame_t: t };
    const period = framePeriodMs(state);
    if (t - state.last_frame_t < period) return {};
    const frames = Math.floor((t - state.last_frame_t) / period);
    const out = {
      last_frame_t: state.last_frame_t + frames * period,
      frame_seq: state.frame_seq + frames
    };
    const perFrame = activeSlots(state).length * channels(state);
    const total = state.fifo_count + Math.min(frames, FIFO_DEPTH) * perFrame;
    out.fifo_count = Math.min(FIFO_DEPTH, total);
    out.fifo_ovf = Math.min(127, state.fifo_ovf + Math.max(0, total - FIFO_DEPTH));
    let st = state.status1 | ST_FRAME_RDY | ST_FIFO_DATA_RDY;
    if (out.fifo_count >= FIFO_DEPTH - state.a_full_free) st |= ST_A_FULL;
    const now = { ...state, ...out };
    for (const i of activeSlots(state))
      for (let ch = 0; ch < channels(state); ch++)
        if (sampleCounts(now, i, ch, now.last_frame_t, true) === null) st |= ST_EXP_OVF;
    if (state.thresh_slot > 0) {
      const v = sampleCounts(now, state.thresh_slot - 1, 0, now.last_frame_t, true) ?? FS;
      const x = Math.max(0, v);
      if (state.thresh_upper < 255 && x > state.thresh_upper * 2048 || x < state.thresh_lower * 2048)
        st |= ST_THRESH1;
    }
    if (state.prox_auto) {
      const v = Math.max(0, sampleCounts(now, 5, 0, now.last_frame_t, false) ?? FS);
      const prox = v < state.thresh_lower * 2048 ? 1 : 0;
      if (prox && !state.in_prox) st |= ST_THRESH1;
      out.in_prox = prox;
    }
    out.status1 = st;
    return out;
  },
  // INTB (pad 8): open-drain active-low; 1 = asserted.
  padOutputs(state) {
    return { "8": state.status1 & state.int_en1 & 254 ? 1 : 0 };
  },
  power(state, ctx) {
    const idd = iddUa(state);
    const iled = iledUa(state);
    const on = state.running && !state.shdn;
    return {
      draw_ua: idd + iled,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: ctx?.padVoltage["10"] ?? 1800,
          i_ua: idd,
          pads: ["10"],
          note: state.shdn ? "shutdown" : on ? `PPG engine at ${state.in_prox ? PROX_FPS : frameRate(state)} fps` : "awake, engine stopped"
        },
        {
          name: "VLED",
          role: "supply",
          v_mv: ctx?.padVoltage["9"] ?? 5e3,
          i_ua: iled,
          pads: ["9"],
          note: on ? "LED current x duty" : "LEDs off"
        }
      ]
    };
  }
};
var sense_hr_default = sim;
export {
  sense_hr_default as default,
  iddUa,
  iledUa
};
