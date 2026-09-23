// Digital-twin contract: the ONE shape every tile twin exports (`export default`).
//
// Lives in the tiles repo next to the drivers, because a twin and its driver
// change together; Studio and the portal consume it.
//
// A twin is the tile's behavior, and the same module serves every surface:
//   - Studio's simulator worker answers the firmware's driver calls with
//     `hostCalls` and ticks `deriveState`;
//   - Studio's main thread reads `power`, `indicators` and `padOutputs` from the
//     SAME state the firmware changed (one field vocabulary, no translation);
//   - the portal runner scores it.
//
// Handlers never touch Wasm memory. They return plain numbers (`array`, `out`,
// `outScalars`) and the simulator writes them into linear memory at the width
// the driver manifest declares (uint8 / int16 / int32). That is what lets one
// module run in the worker, on the main thread and in the portal alike.
//
// Field names follow the DRIVER (what the firmware sets), and `defaultState` is
// the tile's state AFTER the driver's init — the driver and datasheet decide
// behavior, and the twin matches them.

// The contract's version. Bump it on any change a consumer must understand (a
// new hook, a new control kind, a changed result shape) so a Studio built for an
// older contract refuses twins it can't run instead of misrunning them.
export const CONTRACT_VERSION = 1;

// Per-tile state: a flat bag of scalar fields. Structured-cloneable, because it
// crosses the worker boundary.
export interface SimState {
  [field: string]: number | boolean | string;
}

// A user-adjustable input. `field` names the state key it writes.
export interface SliderControl<F extends string = string> {
  type: 'slider';
  field: F;
  label: string;
  min: number;
  max: number;
  step?: number;
  unit?: string;
  description?: string;
}

// An on/off input: `field` is written 1 (on) / 0 (off).
export interface ToggleControl<F extends string = string> {
  type: 'toggle';
  field: F;
  label: string;
  description?: string;
}

export type SimControl<S> = SliderControl<keyof S & string> | ToggleControl<keyof S & string>;

// Something a person DOES to the part in the Simulate panel: a physical
// stimulus (move an object, tilt the board, plug in USB), never a mirror of a
// setting the program owns. Plain data, so a twin can ship without Studio code.
export type Stimulus =
  // A slider over one field. `maxBy` makes the reach follow a setting the
  // program chose: `values[String(state[field])]`, falling back to `max`.
  | {
      kind: 'slider';
      id: string;
      label: string;
      field: string;
      min: number;
      max: number;
      step: number;
      unit?: string;
      maxBy?: { field: string; values: Readonly<Record<string, number>> };
    }
  // One switch over one or more fields that move together (USB: power + data).
  | { kind: 'toggle'; id: string; label: string; fields: readonly string[] }
  // Tilt the board: its attitude sets gravity on these accelerometer fields (mg).
  | { kind: 'attitude'; id: string; label: string; accel: AccelFields }
  // Shake it: acceleration oscillating about rest, up to `max` peak.
  | { kind: 'shake'; id: string; label: string; max: number; unit: string; accel: AccelFields };

export interface AccelFields {
  x: string;
  y: string;
  z: string;
}

// A handful of stimuli that belong together ("Accelerations"). When a twin has
// several, the Simulate panel lets a person pick one.
export interface StimulusGroup {
  id: string;
  label: string;
  controls: readonly Stimulus[];
}

// What a host-call handler is given.
export interface SimCallCtx<S = SimState> {
  // The slot's current state (controls already applied).
  readonly state: S;
  // The scalar arguments the caller passed, in declaration order, without the
  // tile handle and without any buffer pointers / lengths / caps.
  readonly args: number[];
  // Array-IN parameters (`play_sequence(values, n)`), keyed by the parameter
  // name: a snapshot of the caller's values at call time.
  readonly bufferIn?: Readonly<Record<string, readonly number[]>>;
  // Capacity of each caller-sized array-OUT parameter (`read_fifo(buf, cap)`),
  // keyed by the parameter name. Fill it through `out`.
  readonly caps?: Readonly<Record<string, number>>;
}

// What a host-call handler may return. Every field optional.
export interface SimCallResult<S = SimState> {
  // The function's return value.
  scalar?: number;
  // Merged into the slot's state.
  nextState?: Partial<S>;
  // The fixed-length array a call returns (`get_raw_accels` → [x, y, z]).
  array?: readonly number[];
  // Caller-sized array-OUT parameters, keyed by parameter name. At most `cap`
  // values are written; when `scalar` is not given, the call returns the count.
  out?: Readonly<Record<string, readonly number[]>>;
  // Scalar out-parameters (`read_distance_with_confidence(&mm, &pct)`), keyed
  // by parameter name. Missing entries read as 0.
  outScalars?: Readonly<Record<string, number>>;
}

export type SimCallHandler<S = SimState> = (ctx: SimCallCtx<S>) => SimCallResult<S> | void;

// Provenance of a modeled behavior — how much to trust it:
//   canonical    — verified against the component datasheet (numbers/bits/timing)
//   inferred     — obvious from the driver header / function name, not datasheet-checked
//   hallucinated — plausible placeholder of uncertain origin; treat as a guess
export type Provenance = 'canonical' | 'inferred' | 'hallucinated';

// One electrical node on the tile: a supply rail the tile draws from, or an
// output it sources/sinks (motor/haptic drive, a regulated rail, charge
// current, LED channel). Currents in µA, voltages in mV.
export interface PowerRail {
  name: string; // "V+", "V_MOTOR", "OUT", "VOUT", "ICHG", "LED.R"
  // `supply` = drawn FROM this net (sink side); `output` = SOURCED onto it
  // (battery/regulator output, actuator drive). The net's voltage is set by its
  // output rail(s); its current is the sum of the supply draws on it.
  role: 'supply' | 'output';
  v_mv?: number;
  i_ua?: number; // draw (supply) or delivered (output)
  // Component pad id(s) this rail sits on (matching the tile/port pad names in
  // the netlist). Lets the simulator's electrical solver place the rail on a net
  // — and lets a future wire-probe address it. Omitted ⇒ not net-resolved.
  pads?: string[];
  note?: string;
  // The most current this rail can carry (µA): what a `supply` may DRAW (a PMIC's
  // USB input limit) or an `output` may DELIVER (a USB port, a buck). The solver
  // does not clip to it — it reports a rail asked for more (`snapshot.limits`), so
  // the design's problem is shown rather than hidden.
  limit_ua?: number;
}

// Electrical behavior of the tile in its current state — the power layer.
// `draw_ua` is the headline current pulled from the primary supply.
export interface PowerReport {
  draw_ua: number;
  rails?: PowerRail[];
}

// Electrical context the solver hands a twin so its draw can respond to what
// it's wired to — `padVoltage[pad]` is the resolved net voltage (mV) on each of
// the component's pads (undefined where the net is undriven). A sink (motor, LED)
// reads this to compute current from applied voltage; self-contained twins ignore it.
export interface PowerCtx {
  padVoltage: Readonly<Record<string, number>>;
}

// A status indicator the tile presents — an on-board LED, not an electrical pad.
// The twin declares these so the canvas can render them generically: any tile
// that returns indicators gets dots on its node, no per-tile UI code. `level` is
// 0 (off) … 1 (full) so brightness/PWM can ride the same channel later.
export interface TileIndicator {
  id: string; // stable within the tile (e.g. "led.charge")
  color: string; // CSS colour when lit
  level: number; // 0 = off … 1 = full
  label?: string; // tooltip ("Charging", "Fault", …)
}

export interface TileSim<S = SimState> {
  // Must match the tile identity (e.g. "Sense.I.6P6").
  tile: string;
  // State after the driver's init, for a fresh slot.
  defaultState: S;
  // Interactive inputs. Physical stimuli a person applies (distance, a battery
  // level), not mirrors of settings the program owns.
  controls?: readonly SimControl<S>[];
  // What a person can do to the part in the Simulate panel, grouped. The first
  // group is shown until someone picks another.
  stimuli?: readonly StimulusGroup[];
  // Public driver API, keyed by C symbol name → behavior. Optional: passive
  // generics (battery, motor, connector) and Cores have no driver.
  hostCalls?: Readonly<Record<string, SimCallHandler<S>>>;
  // Per-host-call provenance, keyed by the same C symbol names (plus `power`).
  // Unlisted calls read as "inferred".
  provenance?: Readonly<Record<string, Provenance>>;
  // Evolve state one tick. `t` is a monotonic clock in ms: compare it against a
  // timestamp kept in state, never read it as absolute time.
  deriveState?: (state: S, ctx: { t: number }) => Partial<S>;
  // Pad drive levels in [0,1], keyed by the tile's pad id as the tile JSON
  // numbers it (e.g. { '9': 1 } for an INT pad).
  padOutputs?: (state: S) => Record<string, number>;
  // On-board status indicators (LEDs) the canvas renders on the tile. Gets the
  // same electrical `ctx` as `power()`.
  indicators?: (state: S, ctx?: PowerCtx) => TileIndicator[];
  // Electrical behavior in the current state: supply draw and output rails.
  power?: (state: S, ctx?: PowerCtx) => PowerReport;
  // Legacy SimulatorPane only (the canvas never calls it): returns DELTAS added
  // to a per-slot anchor, so a dialed value keeps meaning something. Prefer a
  // `deriveState` gated on a state flag, which every surface ticks.
  automatic?: (ctx: { state: S; t: number; dt: number }) => Partial<S>;
}
