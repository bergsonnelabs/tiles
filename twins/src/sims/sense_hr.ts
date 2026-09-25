// Digital twin for Sense.HR — Analog Devices MAX86174A PPG front end with two
// Vishay VEMD1060X01 photodiodes and four emitters (IR, green x2, red).
//
// Driver tile_sense_hr.{h,c} 1.0.0 (tiles). The optics are modeled from the
// one fully assembled front side (bring-up, August 2026; portal page
// "Bring-up findings: electrical, bus, and data path"). The optical design is
// still being improved, so expect these figures to move with it.
//
// The world is a finger (present or not) with a pulse: heart rate, perfusion
// (the pulsatile AC as a % of the DC return) and SpO₂ (which sets the red/IR
// modulation ratio), plus ambient light. Each frame the chip pulses every
// enabled slot's LED; the photocurrent is emitter-to-detector crosstalk plus
// tissue return (when a finger is present), minus the slot's offset DAC,
// digitized against the slot's ADC range. Samples queue in the 256-word FIFO
// at the frame rate; the driver's reads drain it and keep the newest sample
// per tag, as tile_sense_hr.c does. Over-range samples are replaced by an
// overflow tag and counted, never reported as a value.
//
// Pads: GND 1, ADDR 2, I2C_SEL 3, SCL 4, SDA 5, INTB 8, VLED 9, V+ 10.
// INTB is open-drain, active-low: the pad output is 1 while asserted.
//
// Optics (no crosstalk barrier on this board): the emitters couple straight
// into the photodiodes across the tile. Bench: green at 16 mA overflows even
// the 32 µA range with nothing in front (>= 2 µA of photocurrent per mA), and
// with a finger crosstalk is 49-80 % of the DC. Only green was measured; the
// IR and red figures keep the earlier wavelength ratios.
import type { TileSim } from '../tileSim';

// Global fields are declared; per-slot fields (`s<i>_<name>`, i = 0..5) and the
// decoded-frame cache (`f<k>`, k = 0..11) come from the builders below.
interface State {
  [field: string]: number;
  // ── the world (stimuli) ──
  finger_present: number;
  heart_rate_bpm: number;
  perfusion_pct: number;
  spo2_pct: number;
  ambient_ua: number;
  vled_mv: number; // VLED supply (pad 9); sets the COMPB flags
  last_reg: number; // write_reg / read_reg echo
  last_reg_value: number;

  // ── driver / chip state ──
  tile_ready: number; // tile->state: 1 READY, 2 SLEEPING, 0 NONE (after reset)
  running: number; // frame engine on (start() .. stop())
  shdn: number; // SYS_CFG1.SHDN (sleep)
  restart_pending: number; // re-anchor the frame clock on the next tick
  fr_div: number; // FR_CLK_DIV
  enable_mask: number; // SYS_CFG2 slot enables (driver cache)
  ppg2_enabled: number; // !PPG2_PWRDN
  prox_auto: number; // SYS_CFG3.PROX_AUTO
  in_prox: number; // chip currently in proximity mode (slot 5 alone at 8 fps)
  int_en1: number; // INT_EN1 mask (STATUS1 sources on INTB)
  status1: number; // latched STATUS1
  thresh_slot: number; // THRESH_SEL: 0 = off, n = MEASn
  thresh_lower: number; // THRESH1_LO (x 2048 counts)
  thresh_upper: number; // THRESH1_HI (x 2048 counts)
  a_full_free: number; // FIFO_CFG1: free slots at which A_FULL asserts
  fifo_count: number; // unread samples, 0..256
  fifo_ovf: number; // OVF_COUNTER, saturates at 127
  ovf_count: number; // overflow-tagged samples in the last drain (driver)
  last_frame_t: number;
  frame_seq: number; // frames produced (a synthetic counter)
}

const SLOTS = 6;
const FRAME_LEN = 12;
const FIFO_DEPTH = 256;
const FS = 524287; // SENSE_HR_ADC_FULL_SCALE
const NO_SAMPLE = -2147483648; // SENSE_HR_NO_SAMPLE
const FR_CLK_HZ = 32768;
const PROX_FPS = 8;

// Register decodes (datasheet MEASx fields; driver tables)
const LED_LSB_UA = [125, 250, 375, 500]; // per LED_RGE (32/64/96/128 mA full scale)
const ADC_FS_UA = [4, 8, 16, 32]; // per PPGy_ADC_RGE
const DACOFF_UA = [0, 4, 8, 12, 16, 20, 24, 28];
const TINT_US = [14.6, 29.2, 58.6, 117.1];

// STATUS1 bits
const ST_A_FULL = 0x80;
const ST_FRAME_RDY = 0x40;
const ST_FIFO_DATA_RDY = 0x20;
const ST_EXP_OVF = 0x08;
const ST_THRESH1 = 0x02;
// FIFO tags
const TAG_EXP_OVF = 0xe;

// sense_hr_led_t
const LED_IR = 0;
const LED_GREEN = 1;
const LED_RED = 2;

// ── optics (nA of photocurrent per mA of LED, both PDs summed) ──
// Crosstalk 2200 nA/mA: 16 mA → 35 µA, just past the 32 µA range, as measured.
// Tissue return for green 1300 nA/mA puts crosstalk at ~63 % of the DC, mid
// of the measured 49-80 %. IR / red scale green by the earlier ratios.
const CROSSTALK_NA_PER_MA = 2200;
const RETURN_NA_PER_MA = [2600, 1300, 1870, 1300]; // IR, green, red, green2

// ── LED compliance (STATUS2 LEDn_COMPB) ──
// Typical forward voltage at 20 mA (BOM / IN-S42AT datasheet: IR 1.7 V,
// red 2.1 V, green 3.3 V) plus the driver's 287 mV compliance (datasheet,
// LED_RGE 0x1 typ). VLED below that for an enabled LED sets its COMPB bit.
const LED_VF_MV = [1700, 3300, 2100, 3300]; // LED1 IR, LED2 green, LED3 red, LED4 green
const LED_COMPLIANCE_MV = 287;

const sf = (i: number, k: string) => `s${i}_${k}`;
const slot = (s: State, i: number, k: string) => s[sf(i, k)] ?? 0;
const arg = (args: number[], i: number, fallback: number) =>
  args.length > i && Number.isFinite(args[i]) ? args[i] : fallback;
const validSlot = (n: number) => Number.isInteger(n) && n >= 0 && n < SLOTS;

/** led_quantise(): smallest LED range covering the request, nearest code. */
function ledQuantise(ua: number): { rge?: number; pa: number } {
  if (ua <= 0) return { pa: 0 };
  const u = Math.min(ua, 100000); // SENSE_HR_LED_MAX_UA: the emitters' pulse rating
  for (let r = 0; r < 4; r++) {
    const lsb = LED_LSB_UA[r];
    if (u <= 255 * lsb) {
      const code = Math.min(255, Math.max(1, Math.floor((u + Math.floor(lsb / 2)) / lsb)));
      return { rge: r, pa: code };
    }
  }
  return { rge: 3, pa: 255 };
}
const ledUa = (s: State, i: number) => slot(s, i, 'pa') * LED_LSB_UA[slot(s, i, 'rge') & 3];

const framePeriodMs = (s: State) => (s.in_prox ? 1000 / PROX_FPS : (s.fr_div * 1000) / FR_CLK_HZ);
const frameRate = (s: State) => (s.fr_div ? Math.floor((FR_CLK_HZ + s.fr_div / 2) / s.fr_div) : 0);

/** Slots the chip runs this frame (proximity mode: slot 5 alone). */
function activeSlots(s: State): number[] {
  if (s.in_prox) return [5];
  const out: number[] = [];
  for (let i = 0; i < SLOTS; i++) if (s.enable_mask & (1 << i)) out.push(i);
  if (s.prox_auto && !(s.enable_mask & 0x20)) out.push(5);
  return out;
}
const channels = (s: State) => (s.ppg2_enabled ? 2 : 1);
/** The driver's frame_samples(): enabled slots (+ slot 5 under PROX_AUTO) x channels. */
function driverFrameSamples(s: State): number {
  let n = 0;
  for (let i = 0; i < SLOTS; i++) if (s.enable_mask & (1 << i)) n++;
  if (s.prox_auto && !(s.enable_mask & 0x20)) n++;
  return Math.max(1, n * channels(s));
}

// ── the optical front end ──

/** Pulse phase 0..1 (1 = systole: most blood, least light returned). */
function pulse(s: State, tMs: number): number {
  return 0.5 * (1 + Math.sin((2 * Math.PI * s.heart_rate_bpm * tMs) / 60000));
}
/** Photocurrent (nA, both photodiodes summed) a slot sees at time t. `withAc`
 * false gives the DC level (what auto_gain's averages see). */
function photocurrentNa(s: State, i: number, tMs: number, withAc: boolean): number {
  if (slot(s, i, 'amb')) return s.ambient_ua * 1000; // direct ambient slot
  const led = slot(s, i, 'led') & 3;
  const ma = ledUa(s, i) / 1000;
  let tissue = s.finger_present ? RETURN_NA_PER_MA[led] * ma : 0;
  if (withAc && s.finger_present) {
    const pi = s.perfusion_pct / 100;
    const ratio = Math.max(0.2, (110 - s.spo2_pct) / 25); // R = (AC/DC)red / (AC/DC)ir
    const ac = led === LED_IR ? pi : led === LED_RED ? pi * ratio : 2 * pi; // green: strongest AC
    tissue *= 1 - ac * pulse(s, tMs);
  }
  // Analog ALC removes ambient from LED slots.
  return CROSSTALK_NA_PER_MA * ma + tissue;
}
/** PPG1 / PPG2 share of the summed photocurrent (MEASx_PDSEL). */
function pdShare(pdsel: number, ch: number): number {
  if (pdsel === 0) return ch === 0 ? 1 : 0;
  if (pdsel === 3) return ch === 0 ? 0 : 1;
  return 0.5;
}
/** One sample in ADC counts, or null when it overflows (tagged, not a value). */
function sampleCounts(
  s: State,
  i: number,
  ch: number,
  tMs: number,
  withAc: boolean,
): number | null {
  const na = photocurrentNa(s, i, tMs, withAc) * pdShare(slot(s, i, 'pdsel') & 3, ch);
  const counts = Math.round(
    ((na - DACOFF_UA[slot(s, i, 'dacoff') & 7] * 1000) /
      (ADC_FS_UA[slot(s, i, 'adc') & 3] * 1000)) *
      FS,
  );
  return counts > FS || counts < -FS - 1 ? null : counts;
}

/** drain_into_frame(): newest sample per tag into the cache; overflow words counted. */
function drain(s: State): Partial<State> {
  const out: Partial<State> = { fifo_count: 0, fifo_ovf: 0 };
  // Every entry starts as NO_SAMPLE; an overflowed sample leaves its entry so.
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
  // FIFO_STAT_CLR: FIFO reads clear the FIFO status bits too.
  out.status1 = s.status1 & ~(ST_A_FULL | ST_FRAME_RDY | ST_FIFO_DATA_RDY);
  return out;
}
/** read_ppg / read_ppg2 / read_photocurrent_na: drain if a frame is waiting. */
function cached(s: State, i: number, ch: number): { value: number; next?: Partial<State> } {
  if (!validSlot(i)) return { value: NO_SAMPLE };
  if (s.fifo_count >= driverFrameSamples(s)) {
    const next = drain(s);
    const k = `f${i * 2 + ch}`;
    return { value: (next[k] as number | undefined) ?? s[k], next };
  }
  return { value: s[`f${i * 2 + ch}`] };
}

// ── auto_gain(), replayed on the DC level (tile_sense_hr.c): phase 2 sizes
// the offset DAC from the full photocurrent, the residual plus the offset
// phase 1 already applied ──
function probe(s: State, i: number): { n: number; dc: number; ovf: number } {
  const v = sampleCounts(s, i, 0, 0, false);
  return v === null ? { n: 0, dc: 0, ovf: 99 } : { n: 1, dc: v, ovf: 0 };
}
function autoGain(s0: State, i: number, maxUa: number): { ok: number; next: Partial<State> } {
  if (!validSlot(i) || slot(s0, i, 'amb')) return { ok: 0, next: {} };
  const max = maxUa || 10000;
  const paCap = Math.max(1, max > 255 * 125 ? 255 : Math.floor(max / 125));
  const s: State = { ...s0 };
  s[sf(i, 'rge')] = 0;
  s[sf(i, 'pa')] = Math.min(paCap, 8);
  s[sf(i, 'dacoff')] = 0;
  s[sf(i, 'adc')] = 3;
  const g = (k: string) => s[sf(i, k)];
  const set = (k: string, v: number) => {
    s[sf(i, k)] = v;
  };

  for (let iter = 0; iter < 14; iter++) {
    const { n, dc, ovf } = probe(s, i);
    if (ovf > 4 || (n && dc > (FS * 3) / 4)) {
      if (g('dacoff') < 7) set('dacoff', g('dacoff') + 1);
      else if (g('pa') > 4) set('pa', Math.floor((g('pa') * 3) / 4));
      else break;
    } else if (n && g('pa') < paCap) {
      set('pa', Math.min(paCap, g('pa') + Math.floor(g('pa') / 2) + 1));
    } else break;
  }
  let { n, dc } = probe(s, i);
  const found = n !== 0 ? 1 : 0;
  const totalX100 = n
    ? Math.trunc((dc * ADC_FS_UA[g('adc')] * 100) / FS) + DACOFF_UA[g('dacoff')] * 100
    : 0;
  if (n && totalX100 > 0) {
    const iX100 = totalX100;
    let best = 0;
    for (let d = 0; d < 8; d++) if (DACOFF_UA[d] * 100 + 50 <= iX100) best = d;
    set('dacoff', best);
    const resid = iX100 - DACOFF_UA[best] * 100;
    for (let r = 0; r < 4; r++) {
      if (resid * 10 < ADC_FS_UA[r] * 700) {
        set('adc', r);
        break;
      }
    }
    for (let guard = 0; guard < 4; guard++) {
      ({ n, dc } = probe(s, i));
      const ovf = n ? 0 : 99;
      if (n && ovf <= 4 && dc > 0 && dc < (FS * 3) / 4) break;
      if (g('adc') < 3) set('adc', g('adc') + 1);
      else if (g('dacoff') > 0) set('dacoff', g('dacoff') - 1);
      else break;
    }
  }
  const next: Partial<State> = {
    [sf(i, 'rge')]: g('rge'),
    [sf(i, 'pa')]: g('pa'),
    [sf(i, 'dacoff')]: g('dacoff'),
    [sf(i, 'adc')]: g('adc'),
  };
  // Run-alone probes flushed the FIFO; a running engine is restarted (flush).
  next.fifo_count = 0;
  next.fifo_ovf = 0;
  if (s0.running) next.restart_pending = 1;
  return { ok: found, next };
}

/** STATUS2: INVALID_CFG (bit 7) and LED1..4_COMPB (bits 0-3). */
function status2(s: State): number {
  let v = invalidCfg(s) ? 0x80 : 0;
  for (const i of activeSlots(s)) {
    if (slot(s, i, 'amb') || ledUa(s, i) <= 0) continue;
    const led = slot(s, i, 'led') & 3;
    if (s.vled_mv < (LED_VF_MV[led] ?? 3300) + LED_COMPLIANCE_MV) v |= 1 << led;
  }
  return v;
}

/** Frame-overrun check: exposures of the enabled slots vs the frame period.
 * Per pulse: integration + ~11 µs of LED settling (fitted, see power). The
 * datasheet gives no formula, only the flag. */
function invalidCfg(s: State): boolean {
  let us = 0;
  for (const i of activeSlots(s))
    us += (1 << (slot(s, i, 'aver') & 7)) * (TINT_US[slot(s, i, 'tint') & 3] + 11);
  return us > framePeriodMs(s) * 1000;
}

// ── Supply current ──
// V+ (1.8 V): datasheet "Average VDD Supply Current" rows (TINT 14.6 µs,
// 1 pulse): single channel 1 measure 3.5 / 17 / 95 µA and 4 measures
// 6.5 / 40 / 300 µA at 8 / 64 / 512 fps; dual channel 4.5 / 24 / 150 and
// 9.5 / 60 / 480. Interpolated in frame rate, linear in measures per frame;
// each measure's share is scaled by its pulses x integration against the
// datasheet condition plus an assumed 70 µs fixed overhead. Shutdown 1 µA.
// VLED: average = LED current x duty, duty = fps x pulses x (TINT + 11 µs);
// the 11 µs reproduces the datasheet's ILED rows (870 / 98 / 12.5 µA at
// 512 / 64 / 8 fps, PA 0x7F on the 128 mA range) within 5 %. VLED shutdown 0.5 µA.
type Pts = readonly (readonly [number, number])[];
const IDD_1 = [
  [0, 1.5],
  [8, 3.5],
  [64, 17],
  [512, 95],
] as const;
const IDD_4 = [
  [0, 1.5],
  [8, 6.5],
  [64, 40],
  [512, 300],
] as const;
const IDD_1_DUAL = [
  [0, 1.5],
  [8, 4.5],
  [64, 24],
  [512, 150],
] as const;
const IDD_4_DUAL = [
  [0, 1.5],
  [8, 9.5],
  [64, 60],
  [512, 480],
] as const;
function interp(pts: Pts, x: number): number {
  for (let k = 1; k < pts.length; k++) {
    const [x0, y0] = pts[k - 1];
    const [x1, y1] = pts[k];
    if (x <= x1 || k === pts.length - 1) return y0 + ((x - x0) * (y1 - y0)) / (x1 - x0);
  }
  return pts[0][1];
}
const IDD_SHDN_UA = 1;
const IDD_IDLE_UA = 1.5; // awake, frame engine off (not in the datasheet)
const ILED_SHDN_UA = 0.5;
const MEAS_OVERHEAD_US = 70;
const LED_OVERHEAD_US = 11;

export function iddUa(s: State): number {
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
    const pulses = 1 << (slot(s, i, 'aver') & 7);
    meas +=
      (pulses * TINT_US[slot(s, i, 'tint') & 3] + MEAS_OVERHEAD_US) / (14.6 + MEAS_OVERHEAD_US);
  }
  return Math.round(base + perMeas * meas);
}
export function iledUa(s: State): number {
  if (s.shdn || !s.running) return ILED_SHDN_UA;
  const fps = s.in_prox ? PROX_FPS : frameRate(s);
  let ua = 0;
  for (const i of activeSlots(s)) {
    if (slot(s, i, 'amb')) continue;
    const pulses = 1 << (slot(s, i, 'aver') & 7);
    ua += ledUa(s, i) * fps * pulses * (TINT_US[slot(s, i, 'tint') & 3] + LED_OVERHEAD_US) * 1e-6;
  }
  return Math.round(ua + ILED_SHDN_UA);
}

// ── default state: after init(cfg = NULL) ──
function buildDefault(): State {
  const s: State = {
    finger_present: 1,
    heart_rate_bpm: 72,
    perfusion_pct: 2,
    spo2_pct: 98,
    ambient_ua: 0.5,
    vled_mv: 5000,
    last_reg: 0,
    last_reg_value: 0,

    tile_ready: 1,
    running: 0,
    shdn: 0,
    restart_pending: 1,
    fr_div: 256, // 32768 / 128 fps
    enable_mask: 0x01,
    ppg2_enabled: 0,
    prox_auto: 0,
    in_prox: 0,
    int_en1: 0,
    status1: 0,
    thresh_slot: 0,
    thresh_lower: 0,
    thresh_upper: 0xff,
    a_full_free: 0x7f,
    fifo_count: 0,
    fifo_ovf: 0,
    ovf_count: 0,
    last_frame_t: 0,
    frame_seq: 0,
  };
  for (let i = 0; i < SLOTS; i++) {
    s[sf(i, 'led')] = 0;
    s[sf(i, 'amb')] = 0;
    s[sf(i, 'rge')] = 0;
    s[sf(i, 'pa')] = 0;
    s[sf(i, 'dacoff')] = 0;
    s[sf(i, 'adc')] = 3; // 32 µA
    s[sf(i, 'tint')] = 3; // 117.1 µs
    s[sf(i, 'aver')] = 2; // 4 pulses
    s[sf(i, 'pdsel')] = 0; // both PDs → PPG1
  }
  // Slot 0: green at 1 mA (range 0, code 8).
  s[sf(0, 'led')] = LED_GREEN;
  s[sf(0, 'pa')] = 8;
  for (let k = 0; k < FRAME_LEN; k++) s[`f${k}`] = NO_SAMPLE;
  return s;
}

/** A setter over one slot field: validates the slot like the driver does. */
const slotSetter =
  (field: string, map: (v: number) => number) =>
  ({ args }: { args: number[] }) => {
    const i = arg(args, 0, -1);
    if (!validSlot(i)) return;
    return {
      nextState: { [sf(i, field)]: map(arg(args, 1, 0)) } as Partial<State>,
    };
  };

const sim: TileSim<State> = {
  tile: 'Sense.HR',

  defaultState: buildDefault(),

  controls: [
    {
      type: 'slider',
      field: 'ambient_ua',
      label: 'Ambient light',
      min: 0,
      max: 30,
      step: 0.5,
      unit: 'µA',
      description:
        'Ambient photocurrent. Analog ALC removes it from LED slots; an ambient slot reads it.',
    },
  ],

  // What a person does: put a finger on it (or not), and have a pulse.
  stimuli: [
    {
      id: 'finger',
      label: 'Finger',
      controls: [
        {
          kind: 'toggle',
          id: 'finger_present',
          label: 'finger on sensor',
          fields: ['finger_present'],
        },
        {
          kind: 'slider',
          id: 'heart_rate_bpm',
          label: 'heart rate',
          field: 'heart_rate_bpm',
          min: 30,
          max: 220,
          step: 1,
          unit: 'bpm',
        },
        {
          kind: 'slider',
          id: 'perfusion_pct',
          label: 'perfusion',
          field: 'perfusion_pct',
          min: 0,
          max: 10,
          step: 0.1,
          unit: '%',
        },
        {
          kind: 'slider',
          id: 'spo2_pct',
          label: 'SpO₂',
          field: 'spo2_pct',
          min: 70,
          max: 100,
          step: 1,
          unit: '%',
        },
      ],
    },
    {
      id: 'supply',
      label: 'LED supply',
      controls: [
        {
          kind: 'slider',
          id: 'vled_mv',
          label: 'VLED',
          field: 'vled_mv',
          min: 2700,
          max: 5500,
          step: 100,
          unit: 'mV',
        },
      ],
    },
  ],

  hostCalls: {
    // ── lifecycle ──
    tile_sense_hr_find: () => ({ scalar: 1 }), // PART_ID 0x40
    tile_sense_hr_init: () => ({ scalar: 0, nextState: buildDefault() }),
    tile_sense_hr_sleep: () => ({ nextState: { shdn: 1, tile_ready: 2 } }),
    tile_sense_hr_wake: () => ({
      nextState: { shdn: 0, tile_ready: 1, restart_pending: 1 },
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
        int_en1: 0,
      },
    }),
    tile_sense_hr_start: () => ({
      nextState: { running: 1, fifo_count: 0, fifo_ovf: 0, restart_pending: 1 },
    }),
    tile_sense_hr_stop: () => ({ nextState: { running: 0, in_prox: 0 } }),

    // ── events (the callback itself is not modeled) ──
    tile_sense_hr_process: ({ state }) => {
      if (state.tile_ready !== 1) return;
      return { nextState: { status1: 0 } };
    },
    tile_sense_hr_on_event: () => undefined,
    tile_sense_hr_set_interrupts: ({ args }) => ({
      nextState: { int_en1: arg(args, 0, 0) & 0xfe },
    }),

    // ── configuration ──
    tile_sense_hr_set_frame_rate: ({ state, args }) => {
      const fps = Math.max(1, arg(args, 0, 1));
      const div = Math.min(
        32766,
        Math.max(16, Math.floor((FR_CLK_HZ + Math.floor(fps / 2)) / fps)),
      );
      return {
        scalar: frameRate({ ...state, fr_div: div }),
        nextState: { fr_div: div },
      };
    },
    tile_sense_hr_get_frame_rate: ({ state }) => ({ scalar: frameRate(state) }),
    tile_sense_hr_set_slot: ({ state, args }) => {
      const i = arg(args, 0, -1);
      if (!validSlot(i)) return;
      const q = ledQuantise(arg(args, 2, 0));
      const next: Partial<State> = {
        [sf(i, 'led')]: arg(args, 1, 0) & 3,
        [sf(i, 'amb')]: 0,
        [sf(i, 'pa')]: q.pa,
        enable_mask: state.enable_mask | (1 << i),
      };
      if (q.rge !== undefined) next[sf(i, 'rge')] = q.rge;
      return { nextState: next };
    },
    tile_sense_hr_set_ambient_slot: ({ state, args }) => {
      const i = arg(args, 0, -1);
      if (!validSlot(i)) return;
      return {
        nextState: {
          [sf(i, 'amb')]: 1,
          [sf(i, 'pa')]: 0,
          enable_mask: state.enable_mask | (1 << i),
        },
      };
    },
    tile_sense_hr_enable_slot: ({ state, args }) => {
      const i = arg(args, 0, -1);
      if (!validSlot(i)) return;
      const mask = arg(args, 1, 0) ? state.enable_mask | (1 << i) : state.enable_mask & ~(1 << i);
      return { nextState: { enable_mask: mask } };
    },
    tile_sense_hr_set_led_current: ({ args }) => {
      const i = arg(args, 0, -1);
      if (!validSlot(i)) return;
      const q = ledQuantise(arg(args, 1, 0));
      const next: Partial<State> = { [sf(i, 'pa')]: q.pa };
      if (q.rge !== undefined) next[sf(i, 'rge')] = q.rge;
      return { nextState: next };
    },
    tile_sense_hr_get_led_current: ({ state, args }) => {
      const i = arg(args, 0, -1);
      return { scalar: validSlot(i) ? ledUa(state, i) : 0 };
    },
    tile_sense_hr_set_integration: slotSetter('tint', (v) => v & 3),
    tile_sense_hr_set_averaging: slotSetter('aver', (v) => v & 7),
    tile_sense_hr_set_adc_range: slotSetter('adc', (v) => v & 3),
    tile_sense_hr_set_dac_offset: slotSetter('dacoff', (v) =>
      Math.floor((Math.min(v & 0xff, 28) + 2) / 4),
    ),
    tile_sense_hr_set_photodiodes: slotSetter('pdsel', (v) => v & 3),
    tile_sense_hr_set_ppg2_enabled: ({ args }) => ({
      nextState: { ppg2_enabled: arg(args, 0, 0) ? 1 : 0 },
    }),
    tile_sense_hr_auto_gain: ({ state, args }) => {
      const { ok, next } = autoGain(state, arg(args, 0, -1), arg(args, 1, 0));
      return { scalar: ok, nextState: next };
    },

    // ── runtime ──
    tile_sense_hr_get_status: ({ state }) => ({
      scalar: state.status1,
      nextState: { status1: 0 },
    }),
    tile_sense_hr_get_status2: ({ state }) => ({
      scalar: status2(state),
    }),
    tile_sense_hr_frame_ready: ({ state }) => ({
      scalar: state.fifo_count >= driverFrameSamples(state) ? 1 : 0,
    }),
    tile_sense_hr_get_fifo_count: ({ state }) => ({ scalar: state.fifo_count }),
    tile_sense_hr_get_fifo_overflow: ({ state }) => ({
      scalar: state.fifo_ovf,
    }),
    tile_sense_hr_flush_fifo: () => ({
      nextState: { fifo_count: 0, fifo_ovf: 0 },
    }),
    tile_sense_hr_set_fifo_watermark: ({ args }) => {
      const n = Math.min(FIFO_DEPTH, Math.max(1, arg(args, 0, 1)));
      return { nextState: { a_full_free: FIFO_DEPTH - n } };
    },
    // frame[2·slot + channel]; 1 when a full frame was drained.
    tile_sense_hr_read_frame: ({ state }) => {
      const ok = state.fifo_count >= driverFrameSamples(state);
      const next = ok ? drain(state) : {};
      const merged = { ...state, ...next } as State;
      const frame: number[] = [];
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
      const na =
        Math.trunc((r.value * ADC_FS_UA[slot(state, i, 'adc') & 3] * 1000) / FS) +
        DACOFF_UA[slot(state, i, 'dacoff') & 7] * 1000;
      return { scalar: na, nextState: r.next };
    },
    tile_sense_hr_get_overflow_count: ({ state }) => ({
      scalar: state.ovf_count,
    }),
    // Raw 24-bit words [tag 23:20 | data 19:0] of the newest frame, repeated
    // oldest-first for the frames waiting; slot order, PPG1 then PPG2.
    tile_sense_hr_read_fifo_raw: ({ state, caps }) => {
      const cap = Math.min(caps?.buf ?? 0, state.fifo_count);
      const words: number[] = [];
      const perFrame: number[] = [];
      for (const i of activeSlots(state))
        for (let ch = 0; ch < channels(state); ch++) {
          const v = sampleCounts(state, i, ch, state.last_frame_t, true);
          perFrame.push(v === null ? TAG_EXP_OVF << 20 : ((ch * SLOTS + i) << 20) | (v & 0xfffff));
        }
      for (let k = 0; perFrame.length && words.length < cap; k++)
        words.push(perFrame[k % perFrame.length]);
      return {
        out: { buf: words },
        nextState: { fifo_count: state.fifo_count - words.length },
      };
    },

    // ── proximity / threshold ──
    tile_sense_hr_set_threshold: ({ args }) => {
      const i = arg(args, 0, -1);
      if (!validSlot(i)) return;
      return {
        nextState: {
          thresh_slot: i + 1,
          thresh_lower: arg(args, 1, 0) & 0xff,
          thresh_upper: arg(args, 2, 0) & 0xff,
        },
      };
    },
    // Enabling writes THRESH1_LO; disabling leaves it.
    tile_sense_hr_set_proximity_mode: ({ args }) =>
      arg(args, 0, 0)
        ? { nextState: { prox_auto: 1, thresh_lower: arg(args, 1, 0) & 0xff } }
        : { nextState: { prox_auto: 0, in_prox: 0 } },

    // ── raw registers: identity and status modeled, the rest reads 0 ──
    tile_sense_hr_read_reg: ({ state, args }) => {
      switch (arg(args, 0, 0) & 0xff) {
        case 0x00:
          return { scalar: state.status1, nextState: { status1: 0 } };
        case 0x01:
          return { scalar: status2(state) };
        case 0x05:
          return {
            scalar: (state.fifo_count & 0x100 ? 0x80 : 0) | (state.fifo_ovf & 0x7f),
          };
        case 0x06:
          return { scalar: state.fifo_count & 0xff };
        case 0x12:
          return { scalar: state.running ? state.enable_mask & 0x3f : 0 };
        case 0x1b:
          return { scalar: (state.fr_div >> 8) & 0x7f };
        case 0x1c:
          return { scalar: state.fr_div & 0xff };
        case 0xff:
          return { scalar: 0x40 };
        default:
          // The last raw write reads back; other registers read 0.
          return {
            scalar: (arg(args, 0, 0) & 0xff) === state.last_reg ? state.last_reg_value : 0,
          };
      }
    },
    // Stored for read_reg; the modeled chip state is set through the typed
    // calls, so a raw write does not change sampling behavior.
    tile_sense_hr_write_reg: ({ args }) => ({
      nextState: { last_reg: arg(args, 0, 0) & 0xff, last_reg_value: arg(args, 1, 0) & 0xff },
    }),
  },

  provenance: {
    tile_sense_hr_find: 'canonical', // PART_ID 0x40 at 0x6B
    tile_sense_hr_init: 'inferred',
    tile_sense_hr_sleep: 'canonical', // SHDN, < 1 µA
    tile_sense_hr_wake: 'inferred',
    tile_sense_hr_reset: 'inferred',
    tile_sense_hr_start: 'inferred',
    tile_sense_hr_stop: 'inferred',
    tile_sense_hr_process: 'inferred',
    tile_sense_hr_on_event: 'inferred',
    tile_sense_hr_set_interrupts: 'canonical', // INT_EN1, PWR_RDY non-maskable
    tile_sense_hr_set_frame_rate: 'canonical', // FR_CLK_DIV = 32768 / fps, 16..32766
    tile_sense_hr_get_frame_rate: 'canonical',
    tile_sense_hr_set_slot: 'canonical', // led_quantise: 125/250/375/500 µA per code
    tile_sense_hr_set_ambient_slot: 'inferred',
    tile_sense_hr_enable_slot: 'inferred',
    tile_sense_hr_set_led_current: 'canonical',
    tile_sense_hr_get_led_current: 'canonical',
    tile_sense_hr_set_integration: 'canonical',
    tile_sense_hr_set_averaging: 'canonical',
    tile_sense_hr_set_adc_range: 'canonical', // 4/8/16/32 µA full scale
    tile_sense_hr_set_dac_offset: 'canonical', // 4 µA steps, 0..28
    tile_sense_hr_set_photodiodes: 'inferred', // split routing modeled as half the summed current
    tile_sense_hr_set_ppg2_enabled: 'inferred',
    tile_sense_hr_auto_gain: 'inferred', // the driver's algorithm replayed on the DC level
    tile_sense_hr_get_status: 'inferred',
    tile_sense_hr_get_status2: 'inferred', // COMPB from Vf + compliance vs VLED; INVALID_CFG from a fitted per-pulse time (no datasheet formula)
    tile_sense_hr_frame_ready: 'inferred',
    tile_sense_hr_get_fifo_count: 'inferred',
    tile_sense_hr_get_fifo_overflow: 'inferred',
    tile_sense_hr_flush_fifo: 'inferred',
    tile_sense_hr_set_fifo_watermark: 'canonical', // FIFO_A_FULL = free slots
    tile_sense_hr_read_frame: 'inferred', // optics from the bench (green; no barrier), IR/red scaled
    tile_sense_hr_read_ppg: 'inferred',
    tile_sense_hr_read_ppg2: 'inferred',
    tile_sense_hr_read_photocurrent_na: 'inferred', // the driver's conversion, placeholder optics under it
    tile_sense_hr_get_overflow_count: 'inferred',
    tile_sense_hr_read_fifo_raw: 'inferred', // word format canonical, sample order assumed
    tile_sense_hr_set_threshold: 'inferred',
    tile_sense_hr_set_proximity_mode: 'inferred',
    tile_sense_hr_read_reg: 'inferred',
    tile_sense_hr_write_reg: 'inferred', // last write echoes through read_reg, not applied to the modeled chip
    power: 'inferred', // datasheet IDD / ILED rows, scaled to the configuration (see header of power)
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
    const out: Partial<State> = {
      last_frame_t: state.last_frame_t + frames * period,
      frame_seq: state.frame_seq + frames,
    };
    const perFrame = activeSlots(state).length * channels(state);
    const total = state.fifo_count + Math.min(frames, FIFO_DEPTH) * perFrame;
    out.fifo_count = Math.min(FIFO_DEPTH, total);
    out.fifo_ovf = Math.min(127, state.fifo_ovf + Math.max(0, total - FIFO_DEPTH));

    let st = state.status1 | ST_FRAME_RDY | ST_FIFO_DATA_RDY;
    if (out.fifo_count >= FIFO_DEPTH - state.a_full_free) st |= ST_A_FULL;
    const now = { ...state, ...out } as State;
    for (const i of activeSlots(state))
      for (let ch = 0; ch < channels(state); ch++)
        if (sampleCounts(now, i, ch, now.last_frame_t, true) === null) st |= ST_EXP_OVF;
    if (state.thresh_slot > 0) {
      const v = sampleCounts(now, state.thresh_slot - 1, 0, now.last_frame_t, true) ?? FS;
      const x = Math.max(0, v);
      if (
        (state.thresh_upper < 0xff && x > state.thresh_upper * 2048) ||
        x < state.thresh_lower * 2048
      )
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
    return { '8': state.status1 & state.int_en1 & 0xfe ? 1 : 0 };
  },

  power(state, ctx) {
    const idd = iddUa(state);
    const iled = iledUa(state);
    const on = state.running && !state.shdn;
    return {
      draw_ua: idd + iled,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: ctx?.padVoltage['10'] ?? 1800,
          i_ua: idd,
          pads: ['10'],
          note: state.shdn
            ? 'shutdown'
            : on
              ? `PPG engine at ${state.in_prox ? PROX_FPS : frameRate(state)} fps`
              : 'awake, engine stopped',
        },
        {
          name: 'VLED',
          role: 'supply',
          v_mv: ctx?.padVoltage['9'] ?? 5000,
          i_ua: iled,
          pads: ['9'],
          note: on ? 'LED current x duty' : 'LEDs off',
        },
      ],
    };
  },
};

export default sim;
