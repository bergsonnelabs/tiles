// Digital twin for Sense.ADC.6 — six-channel analog input tile on I2C.
//
// Unlike the sensor tiles, this one has no external chip to model: the tile is
// a purpose-burned Core.ST.L0.1 (STM32L011E4) whose firmware we own, so the
// "datasheet" for its behavior is the tile's own register map, verified on
// hardware on 2026-09-22. That makes most of the modeled behavior canonical
// rather than inferred; the electrical numbers come from the STM32L011
// datasheet (DocID027973 Rev 5).
//
// Physical world (controls): a voltage on each of the six inputs. Readings
// clamp to the supply rail, because the ADC's full scale IS the rail — a
// source above it simply reads full scale.
//
// Sampling: TIM2 paces sweeps at rate x oversamp per second, and the tile
// publishes the average of the most recent window x oversamp sweeps. The
// sequence number advances once per output period, which is how a host tells
// a fresh set from a repeat. Modeled here by deriving the sequence from sim
// time, so changing the rate changes how fast it advances.
//
// Pad map: GND (1), CH0 (2), CH1 (3), SCL (4), SDA (5), CH2 (6), CH3 (7),
// CH4 (8), CH5 (9), V+ (10). The inputs are analog, and the tile drives no
// digital output — there is no data-ready line, because no pad is left.
import type { TileSim } from '../tileSim';

const CHANNELS = 6;

// Limits enforced by the tile firmware; an out-of-range set is rejected and
// the tile keeps running as it was.
const RATE_MAX = 250;
const OVERSAMP_MAX = 32;
const WINDOW_MAX = 2;
const RING_SCANS_MAX = 32; // oversamp * window — the DMA ring is 12 B per sweep
const SCAN_HZ_MAX = 8000; // rate * oversamp — a sweep takes ~65 us

// Status bits (tile_sense_adc_6.h).
const ST_VALID = 1 << 0;
const ST_SAMPLING = 1 << 1;
const ST_CALIBRATED = 1 << 2;

// STM32L011 datasheet: Run mode at 32 MHz from HSI16+PLL, VCORE 1.8 V,
// 4.7 mA typ. The ADC adds 40 uA on VDDA at 10 ksps rising to 200 uA at
// 1.14 Msps; the default settings sweep 6 channels 800 times a second, so
// about 4.8 ksps and the low end of that range.
const RUN_UA = 4700;
const ADC_UA_AT_10KSPS = 40;
const ADC_UA_AT_1140KSPS = 200;

type State = {
  // ── physical world: the voltage on each input (controls / live signals) ──
  ch0_mv: number;
  ch1_mv: number;
  ch2_mv: number;
  ch3_mv: number;
  ch4_mv: number;
  ch5_mv: number;
  supply_mv: number; // the rail, which is also full scale
  simulate: boolean; // drive the six inputs with live signals
  // ── the tile's acquisition (what set_rate changes) ──
  rate_hz: number;
  oversamp: number;
  window: number;
  seq: number; // the tile's sequence register: +1 per output period, wraps 255
  // ── the driver's copy, latched by update() — zero until the first update,
  // exactly as the driver's state is after init (memzero) ──
  read_status: number;
  read_seq: number;
  read_supply_mv: number;
  read0_mv: number;
  read1_mv: number;
  read2_mv: number;
  read3_mv: number;
  read4_mv: number;
  read5_mv: number;
};

const READS = ['read0_mv', 'read1_mv', 'read2_mv', 'read3_mv', 'read4_mv', 'read5_mv'] as const;

const chMv = (s: State, ch: number): number => {
  const raw = [s.ch0_mv, s.ch1_mv, s.ch2_mv, s.ch3_mv, s.ch4_mv, s.ch5_mv][ch] ?? 0;
  // Full scale is the rail: anything above it reads as the rail, and the ADC
  // cannot go negative.
  return Math.max(0, Math.min(Math.round(raw), Math.round(s.supply_mv)));
};

const settingsValid = (rate: number, oversamp: number, window: number): boolean =>
  rate >= 1 &&
  rate <= RATE_MAX &&
  oversamp >= 1 &&
  oversamp <= OVERSAMP_MAX &&
  window >= 1 &&
  window <= WINDOW_MAX &&
  oversamp * window <= RING_SCANS_MAX &&
  rate * oversamp <= SCAN_HZ_MAX;

const adcUa = (state: State): number => {
  const sps = state.rate_hz * state.oversamp * CHANNELS;
  const span = (ADC_UA_AT_1140KSPS - ADC_UA_AT_10KSPS) / (1_140_000 - 10_000);
  return Math.round(Math.max(ADC_UA_AT_10KSPS, ADC_UA_AT_10KSPS + (sps - 10_000) * span));
};

const sim: TileSim<State> = {
  tile: 'Sense.ADC.6',

  defaultState: {
    ch0_mv: 0,
    ch1_mv: 0,
    ch2_mv: 0,
    ch3_mv: 0,
    ch4_mv: 0,
    ch5_mv: 0,
    supply_mv: 3300,
    rate_hz: 100,
    oversamp: 8,
    window: 1,
    seq: 0,
    simulate: true,
    read_status: 0,
    read_seq: 0,
    read_supply_mv: 0,
    read0_mv: 0,
    read1_mv: 0,
    read2_mv: 0,
    read3_mv: 0,
    read4_mv: 0,
    read5_mv: 0,
  },

  controls: [
    {
      type: 'toggle',
      field: 'simulate',
      label: 'Simulate live signals',
      description: 'Drives all six inputs. Turn it off to set them by hand.',
    },
    {
      type: 'slider',
      field: 'ch0_mv',
      label: 'CH0 (pad 2)',
      min: 0,
      max: 3300,
      step: 10,
      unit: 'mV',
    },
    {
      type: 'slider',
      field: 'ch1_mv',
      label: 'CH1 (pad 3)',
      min: 0,
      max: 3300,
      step: 10,
      unit: 'mV',
    },
    {
      type: 'slider',
      field: 'ch2_mv',
      label: 'CH2 (pad 6)',
      min: 0,
      max: 3300,
      step: 10,
      unit: 'mV',
    },
    {
      type: 'slider',
      field: 'ch3_mv',
      label: 'CH3 (pad 7)',
      min: 0,
      max: 3300,
      step: 10,
      unit: 'mV',
    },
    {
      type: 'slider',
      field: 'ch4_mv',
      label: 'CH4 (pad 8)',
      min: 0,
      max: 3300,
      step: 10,
      unit: 'mV',
    },
    {
      type: 'slider',
      field: 'ch5_mv',
      label: 'CH5 (pad 9)',
      min: 0,
      max: 3300,
      step: 10,
      unit: 'mV',
    },
  ],

  hostCalls: {
    tile_sense_adc_6_find: () => ({ scalar: 1 }),
    tile_sense_adc_6_init: () => ({ scalar: 0 }),

    // One transaction fetches status, sequence and all six channels, so the
    // set is coherent by construction; the driver latches it (plus the supply)
    // and every getter below reads that copy.
    tile_sense_adc_6_update: ({ state }) => {
      const next: Partial<State> = {
        read_status: ST_VALID | ST_SAMPLING | ST_CALIBRATED,
        read_seq: state.seq & 0xff,
        read_supply_mv: Math.round(state.supply_mv),
      };
      for (let ch = 0; ch < CHANNELS; ch++) next[READS[ch]!] = chMv(state, ch);
      return { scalar: 1, nextState: next };
    },

    tile_sense_adc_6_read_mv: ({ state, args }) => {
      const key = READS[args[0] ?? 0];
      return { scalar: key ? state[key] : 0 };
    },
    tile_sense_adc_6_sequence: ({ state }) => ({ scalar: state.read_seq }),
    tile_sense_adc_6_is_valid: ({ state }) => ({ scalar: state.read_status & ST_VALID ? 1 : 0 }),
    tile_sense_adc_6_status: ({ state }) => ({ scalar: state.read_status }),
    tile_sense_adc_6_supply_mv: ({ state }) => ({ scalar: state.read_supply_mv }),

    // Applied as a set, and rejected as a set: on rejection the tile keeps
    // running exactly as it was.
    tile_sense_adc_6_set_rate: ({ args }) => {
      const [rate = 0, oversamp = 0, window = 0] = args;
      if (!settingsValid(rate, oversamp, window)) return { scalar: 0 };
      return { scalar: 1, nextState: { rate_hz: rate, oversamp, window } };
    },

    tile_sense_adc_6_get_rate: ({ state }) => ({ scalar: state.rate_hz }),
    tile_sense_adc_6_get_oversamp: ({ state }) => ({ scalar: state.oversamp }),
    tile_sense_adc_6_get_window: ({ state }) => ({ scalar: state.window }),

    tile_sense_adc_6_save: () => ({ scalar: 1 }),

    // The stored address only takes effect at the tile's next power-up, so the
    // modeled state keeps answering on the old one, as the real tile does.
    tile_sense_adc_6_set_address: ({ args }) => {
      const addr = args[0] ?? 0;
      return { scalar: addr >= 0x08 && addr <= 0x77 ? 1 : 0 };
    },
  },

  // Everything here follows the tile's own register map and firmware, which we
  // own and have verified on hardware — canonical rather than guessed. The
  // exception is the address call, whose effect is deferred to a power cycle
  // the twin does not model.
  provenance: {
    tile_sense_adc_6_find: 'canonical',
    tile_sense_adc_6_init: 'canonical',
    tile_sense_adc_6_update: 'canonical',
    tile_sense_adc_6_read_mv: 'canonical',
    tile_sense_adc_6_sequence: 'canonical',
    tile_sense_adc_6_is_valid: 'canonical',
    tile_sense_adc_6_status: 'canonical',
    tile_sense_adc_6_supply_mv: 'canonical',
    tile_sense_adc_6_set_rate: 'canonical',
    tile_sense_adc_6_get_rate: 'canonical',
    tile_sense_adc_6_get_oversamp: 'canonical',
    tile_sense_adc_6_get_window: 'canonical',
    tile_sense_adc_6_save: 'inferred',
    tile_sense_adc_6_set_address: 'inferred',
  },

  /**
   * Live signals, when `simulate` is on.
   *
   * This runs from deriveState, not from the `automatic` hook: the canvas
   * twin runner ticks deriveState and never calls automatic, so signals put
   * there were invisible on the canvas. SimulatorPane ticks deriveState too,
   * so one implementation covers both surfaces. Gating behavior on a state
   * flag here follows display_rgbw.
   *
   *   CH0  slow sweep across most of the range (a pot or flex sensor)
   *   CH1  press events: a resting baseline with occasional firm presses
   *   CH2  the neighbouring taxel, pressed later and less hard
   *   CH3  a slowly drifting baseline (settling, or temperature)
   *   CH4  four discrete levels (a resistor ladder or selector)
   *   CH5  quiet, resting near the top of the range
   *
   * Pure functions of `t`, so a run reproduces exactly; no Math.random. The
   * small wobble is two mismatched sines, which reads as sensor noise.
   *
   * Deliberately NOT modeled: mains hum. These sims return their state as the
   * reading, so a 50 Hz ripple would pass straight through and imply the tile
   * does not filter, when on hardware the averaging window removes it.
   */
  deriveState: (state, { t }) => {
    // The sequence number advances once per output period whatever else
    // happens, so a host polling faster than the rate sees repeats.
    const seq = Math.floor((t * state.rate_hz) / 1000) & 0xff;
    if (!state.simulate) return { seq };

    const full = state.supply_mv;
    const clamp = (mv: number) => Math.max(0, Math.min(Math.round(mv), Math.round(full)));
    const wobble = (scale: number) => (Math.sin(t * 0.017) + Math.sin(t * 0.0413)) * scale;

    // A press rises and falls on a raised cosine, `width` ms wide, once per
    // `period`. Feels like a finger rather than a square edge.
    const press = (period: number, width: number, phase: number) => {
      const into = (t + phase) % period;
      if (into > width) return 0;
      return 0.5 * (1 - Math.cos((2 * Math.PI * into) / width));
    };

    const steps = [0.1, 0.4, 0.7, 0.95];
    const step = steps[Math.floor(t / 2500) % steps.length] ?? 0.1;

    return {
      seq,
      ch0_mv: clamp(full * (0.5 + 0.45 * Math.sin((2 * Math.PI * t) / 6000)) + wobble(3)),
      ch1_mv: clamp(full * (0.06 + 0.7 * press(5000, 900, 0)) + wobble(4)),
      ch2_mv: clamp(full * (0.06 + 0.45 * press(5000, 1400, 2200)) + wobble(4)),
      ch3_mv: clamp(full * (0.35 + 0.08 * Math.sin((2 * Math.PI * t) / 23000)) + wobble(6)),
      ch4_mv: clamp(full * step + wobble(2)),
      ch5_mv: clamp(full * 0.92 + wobble(3)),
    };
  },

  power: (state, ctx) => {
    // Track the rail the tile is actually wired to: full scale follows it.
    const railMv = ctx?.padVoltage?.['10'];
    const supply_mv = railMv && railMv > 0 ? railMv : state.supply_mv;
    const ua = RUN_UA + adcUa(state);
    return {
      draw_ua: ua,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: supply_mv,
          i_ua: ua,
          pads: ['10'],
          note: `STM32L011 run 32 MHz + ADC at ${state.rate_hz * state.oversamp} sweeps/s`,
        },
      ],
    };
  },
};

export default sim;
