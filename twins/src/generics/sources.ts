// Generic power-source twins (battery, coin cell, USB, …) built from one
// archetype. A source is a passive part — no driver, no host calls — that puts a
// voltage onto its output net. Variable (a level slider over its rail envelope)
// and switchable (a Connected toggle: removed/dead → 0 V). The simulator's
// electrical solver reads the `output` rail to set the net voltage; the current
// delivered is computed by the solver from what draws on that net.
//
// Envelopes + the positive-pad name mirror the generic specs in the studio's
// genericComponents.ts so a twin lines up with the part the user wired.
import type { SimState, TileSim } from '../tileSim';

export interface SourceState extends SimState {
  level_mv: number;
}

/** A variable voltage source on `posPad`. The part is always wired in (the twin
 * can't be "disconnected" — that's a wiring fact, not a control); a dead/dark
 * source is just its level dialed to the bottom of the range. */
export function makeSource(opts: {
  tile: string;
  minMv: number;
  nomMv: number;
  maxMv: number;
  posPad: string;
  /** Slider label (e.g. "Cell voltage"). */
  label?: string;
}): TileSim<SourceState> {
  const { tile, minMv, nomMv, maxMv, posPad } = opts;
  return {
    tile,
    defaultState: { level_mv: nomMv },
    controls: [
      {
        type: 'slider',
        field: 'level_mv',
        label: opts.label ?? 'Voltage',
        min: minMv,
        max: maxMv,
        step: 50,
        unit: 'mV',
      },
    ],
    power: (s) => ({
      draw_ua: 0,
      rails: [
        { name: posPad, role: 'output', v_mv: s.level_mv, pads: [posPad], note: 'source output' },
      ],
    }),
    // Idealized source — voltage tracks the dialed level; internal resistance /
    // capacity / discharge curve aren't modeled yet.
    provenance: { power: 'inferred' },
  };
}

export const battery = makeSource({
  tile: 'Generic.Battery',
  minMv: 3000,
  nomMv: 3700,
  maxMv: 4200,
  posPad: 'V+',
  label: 'Cell voltage',
});

export const coinCell = makeSource({
  tile: 'Generic.Coin Cell',
  minMv: 2000,
  nomMv: 3000,
  maxMv: 3300,
  posPad: 'V+',
  label: 'Cell voltage',
});

export const batteryHolder = makeSource({
  tile: 'Generic.Battery Holder',
  minMv: 1800,
  nomMv: 3000,
  maxMv: 3300,
  posPad: 'V+',
  label: 'Pack voltage',
});

export const solarPanel = makeSource({
  tile: 'Generic.Solar Panel',
  minMv: 0,
  nomMv: 5000,
  maxMv: 6000,
  posPad: 'V+',
  label: 'Panel voltage',
});

const VBUS_MV = 5000;
/** What an ordinary USB port gives a device that has not negotiated for more. */
const USB_PORT_LIMIT_UA = 500_000;

/** USB-C port. Unlike a battery it isn't always live: `pow` is whether VBUS is
 * delivering power (a charger/cable plugged in) and drives the 5V0 output; `conn`
 * is whether a data host is attached (informational for now). */
export const usb: TileSim<SimState> = {
  tile: 'Generic.USB',
  defaultState: { pow: 1, conn: 0 },
  controls: [
    { type: 'toggle', field: 'pow', label: 'POW' }, // VBUS delivering power
    { type: 'toggle', field: 'conn', label: 'CONN' }, // data host attached
  ],
  // Plugged in or not: to a person that is ONE fact, power and the data host.
  stimuli: [
    {
      id: 'connected',
      label: 'Connected',
      controls: [{ kind: 'toggle', id: 'connected', label: 'connected', fields: ['pow', 'conn'] }],
    },
  ],
  power: (s) => ({
    draw_ua: 0,
    rails: [
      {
        name: '5V0',
        role: 'output',
        v_mv: s.pow ? VBUS_MV : 0,
        pads: ['5V0'],
        limit_ua: USB_PORT_LIMIT_UA,
        note: s.pow ? 'VBUS 5 V' : 'VBUS off',
      },
    ],
  }),
  provenance: { power: 'inferred' },
};

/** A regulated rail brought in from outside (a bench supply, another board). It is
 * either on or off — there is nothing to dial: the part DECLARES its voltage, which
 * is the whole reason it can stand in for a regulator. */
function makeExternal(tile: string, mv: number, posPad: string): TileSim<SimState> {
  return {
    tile,
    defaultState: { on: 1 },
    controls: [{ type: 'toggle', field: 'on', label: 'ON' }],
    power: (s) => ({
      draw_ua: 0,
      rails: [
        {
          name: posPad,
          role: 'output',
          v_mv: s.on ? mv : 0,
          pads: [posPad],
          note: s.on ? 'supply on' : 'supply off',
        },
      ],
    }),
    provenance: { power: 'inferred' },
  };
}

// Pad names mirror the studio's genericComponents.ts specs.
export const external3v3 = makeExternal('Generic.External 3.3V', 3300, '3V3');
export const external5v = makeExternal('Generic.External 5V', 5000, '5V0');

export const sources = [battery, coinCell, batteryHolder, solarPanel, usb, external3v3, external5v];
