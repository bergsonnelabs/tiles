// Generic load twins — passive sinks whose current is set by the voltage applied
// to them (read from PowerCtx), not by a host call. The LED is the simplest: a
// channel conducts (V_applied − Vf) / R_series amps once its drive exceeds the
// forward voltage, so it draws nothing until something actually drives it.
//
// This is the first `ctx`-reading sink — it proves the solver's two-pass: net
// voltages resolve first, then each sink's draw is computed against them.
import type { PowerRail, SimState, TileSim } from '../tileSim';

/** A multi-channel LED. Each channel draws from its own drive pad through a
 * series resistor; brightness/colour follow the per-channel current. */
export function makeLed(opts: {
  tile: string;
  channels: { pad: string; label: string }[];
  /** Forward voltage (mV) and series resistance (Ω). Defaults: a 2 V LED + 330 Ω. */
  vfMv?: number;
  rOhm?: number;
}): TileSim<SimState> {
  const vf = opts.vfMv ?? 2000;
  const r = opts.rOhm ?? 330;
  return {
    tile: opts.tile,
    defaultState: {},
    power: (_state, ctx) => {
      const rails: PowerRail[] = [];
      let total = 0;
      for (const ch of opts.channels) {
        const v = ctx?.padVoltage?.[ch.pad] ?? 0; // applied drive on this channel
        // (mV − mV) / Ω = mA; ×1000 → µA.
        const i = v > vf ? Math.round(((v - vf) / r) * 1000) : 0;
        total += i;
        rails.push({ name: ch.label, role: 'supply', v_mv: v, i_ua: i, pads: [ch.pad] });
      }
      return { draw_ua: total, rails };
    },
    provenance: { power: 'inferred' },
  };
}

export const rgbLed = makeLed({
  tile: 'Generic.RGB LED',
  channels: [
    { pad: 'R', label: 'R' },
    { pad: 'G', label: 'G' },
    { pad: 'B', label: 'B' },
  ],
});

export const led = makeLed({
  tile: 'Generic.LED',
  channels: [{ pad: 'anode', label: 'LED' }],
});

export const loads = [rgbLed, led];
