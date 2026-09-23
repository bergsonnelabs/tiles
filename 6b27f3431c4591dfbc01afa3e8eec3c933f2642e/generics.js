// src/generics/sources.ts
function makeSource(opts) {
  const { tile, minMv, nomMv, maxMv, posPad } = opts;
  return {
    tile,
    defaultState: { level_mv: nomMv },
    controls: [
      {
        type: "slider",
        field: "level_mv",
        label: opts.label ?? "Voltage",
        min: minMv,
        max: maxMv,
        step: 50,
        unit: "mV"
      }
    ],
    power: (s) => ({
      draw_ua: 0,
      rails: [
        { name: posPad, role: "output", v_mv: s.level_mv, pads: [posPad], note: "source output" }
      ]
    }),
    // Idealized source — voltage tracks the dialed level; internal resistance /
    // capacity / discharge curve aren't modeled yet.
    provenance: { power: "inferred" }
  };
}
var battery = makeSource({
  tile: "Generic.Battery",
  minMv: 3e3,
  nomMv: 3700,
  maxMv: 4200,
  posPad: "V+",
  label: "Cell voltage"
});
var coinCell = makeSource({
  tile: "Generic.Coin Cell",
  minMv: 2e3,
  nomMv: 3e3,
  maxMv: 3300,
  posPad: "V+",
  label: "Cell voltage"
});
var batteryHolder = makeSource({
  tile: "Generic.Battery Holder",
  minMv: 1800,
  nomMv: 3e3,
  maxMv: 3300,
  posPad: "V+",
  label: "Pack voltage"
});
var solarPanel = makeSource({
  tile: "Generic.Solar Panel",
  minMv: 0,
  nomMv: 5e3,
  maxMv: 6e3,
  posPad: "V+",
  label: "Panel voltage"
});
var VBUS_MV = 5e3;
var USB_PORT_LIMIT_UA = 5e5;
var usb = {
  tile: "Generic.USB",
  defaultState: { pow: 1, conn: 0 },
  controls: [
    { type: "toggle", field: "pow", label: "POW" },
    // VBUS delivering power
    { type: "toggle", field: "conn", label: "CONN" }
    // data host attached
  ],
  // Plugged in or not: to a person that is ONE fact, power and the data host.
  stimuli: [
    {
      id: "connected",
      label: "Connected",
      controls: [{ kind: "toggle", id: "connected", label: "connected", fields: ["pow", "conn"] }]
    }
  ],
  power: (s) => ({
    draw_ua: 0,
    rails: [
      {
        name: "5V0",
        role: "output",
        v_mv: s.pow ? VBUS_MV : 0,
        pads: ["5V0"],
        limit_ua: USB_PORT_LIMIT_UA,
        note: s.pow ? "VBUS 5 V" : "VBUS off"
      }
    ]
  }),
  provenance: { power: "inferred" }
};
function makeExternal(tile, mv, posPad) {
  return {
    tile,
    defaultState: { on: 1 },
    controls: [{ type: "toggle", field: "on", label: "ON" }],
    power: (s) => ({
      draw_ua: 0,
      rails: [
        {
          name: posPad,
          role: "output",
          v_mv: s.on ? mv : 0,
          pads: [posPad],
          note: s.on ? "supply on" : "supply off"
        }
      ]
    }),
    provenance: { power: "inferred" }
  };
}
var external3v3 = makeExternal("Generic.External 3.3V", 3300, "3V3");
var external5v = makeExternal("Generic.External 5V", 5e3, "5V0");
var sources = [battery, coinCell, batteryHolder, solarPanel, usb, external3v3, external5v];

// src/generics/loads.ts
function makeLed(opts) {
  const vf = opts.vfMv ?? 2e3;
  const r = opts.rOhm ?? 330;
  return {
    tile: opts.tile,
    defaultState: {},
    power: (_state, ctx) => {
      const rails = [];
      let total = 0;
      for (const ch of opts.channels) {
        const v = ctx?.padVoltage?.[ch.pad] ?? 0;
        const i = v > vf ? Math.round((v - vf) / r * 1e3) : 0;
        total += i;
        rails.push({ name: ch.label, role: "supply", v_mv: v, i_ua: i, pads: [ch.pad] });
      }
      return { draw_ua: total, rails };
    },
    provenance: { power: "inferred" }
  };
}
var rgbLed = makeLed({
  tile: "Generic.RGB LED",
  channels: [
    { pad: "R", label: "R" },
    { pad: "G", label: "G" },
    { pad: "B", label: "B" }
  ]
});
var led = makeLed({
  tile: "Generic.LED",
  channels: [{ pad: "anode", label: "LED" }]
});
var loads = [rgbLed, led];

// src/generics/switches.ts
var spst = {
  tile: "Generic.SPST",
  defaultState: { closed: 1 },
  controls: [{ type: "toggle", field: "closed", label: "Closed" }],
  power: (s, ctx) => {
    const va = ctx?.padVoltage?.["A1"] ?? 0;
    const vb = ctx?.padVoltage?.["B1"] ?? 0;
    const closed = (s.closed ?? 1) !== 0;
    const [supplyPad, outPad, driven] = va >= vb ? ["A1", "B1", va] : ["B1", "A1", vb];
    return {
      draw_ua: 0,
      rails: [
        // Undeclared current on the driven side → the solver fills it from the
        // out-net load, so the source delivers the system through a closed switch.
        { name: supplyPad, role: "supply", v_mv: driven, pads: [supplyPad], note: "contact" },
        {
          name: outPad,
          role: "output",
          v_mv: closed ? driven : 0,
          pads: [outPad],
          note: closed ? "closed" : "open"
        }
      ]
    };
  },
  provenance: { power: "inferred" }
};
var switches = [spst];

// <stdin>
var stdin_default = [...sources, ...loads, ...switches];
export {
  stdin_default as default
};
