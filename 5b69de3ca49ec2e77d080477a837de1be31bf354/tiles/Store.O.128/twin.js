// src/sims/store_o_128.ts
var JEDEC_ID = 2048536;
var CAPACITY = 16777216;
var SR_WIP = 1;
var SR_WEL = 2;
var MODE_STANDBY = 0;
var MODE_READ = 1;
var MODE_PROGRAM = 2;
var MODE_ERASE = 3;
var MODE_DPD = 4;
var UVLO_MV = 1710;
var I_STANDBY_UA = 12;
var I_READ_UA = 1e4;
var I_PROGRAM_UA = 2e4;
var I_ERASE_UA = 25e3;
var I_DPD_UA = 1;
function modeUa(m) {
  if (m === MODE_READ) return I_READ_UA;
  if (m === MODE_PROGRAM) return I_PROGRAM_UA;
  if (m === MODE_ERASE) return I_ERASE_UA;
  if (m === MODE_DPD) return I_DPD_UA;
  return I_STANDBY_UA;
}
var sim = {
  tile: "Store.O.128",
  defaultState: { mode: MODE_STANDBY, wel: 0, busy: 0, vplus_mv: 1800 },
  controls: [
    { type: "slider", field: "mode", label: "Mode (0std1rd2pg3er4dpd)", min: 0, max: 4, step: 1 },
    {
      type: "slider",
      field: "vplus_mv",
      label: "V+ supply",
      min: 1500,
      max: 2e3,
      step: 10,
      unit: "mV"
    }
  ],
  hostCalls: {
    // ── lifecycle ──
    tile_store_o_128_find: () => ({ scalar: 1 }),
    // JEDEC ID matches
    tile_store_o_128_init: () => ({
      scalar: 0,
      nextState: { mode: MODE_STANDBY, wel: 0, busy: 0 }
    }),
    // ── identity / status ──
    tile_store_o_128_read_jedec_id: () => ({ scalar: JEDEC_ID }),
    tile_store_o_128_read_status: ({ state }) => ({
      scalar: (state.busy ? SR_WIP : 0) | (state.wel ? SR_WEL : 0)
    }),
    tile_store_o_128_is_busy: ({ state }) => ({ scalar: state.busy }),
    tile_store_o_128_get_capacity: () => ({ scalar: CAPACITY }),
    // ── read (data bytes not modeled) ──
    tile_store_o_128_read: () => ({ nextState: { mode: MODE_STANDBY } }),
    tile_store_o_128_fast_read: () => ({ nextState: { mode: MODE_STANDBY } }),
    // ── write / erase ──
    tile_store_o_128_write_enable: () => ({ nextState: { wel: 1 } }),
    // blocking in the driver: returns once WIP clears (WEL auto-clears too)
    tile_store_o_128_page_program: () => ({ nextState: { mode: MODE_STANDBY, busy: 0, wel: 0 } }),
    tile_store_o_128_write: () => ({ nextState: { mode: MODE_STANDBY, busy: 0, wel: 0 } }),
    tile_store_o_128_erase_sector: () => ({ nextState: { mode: MODE_STANDBY, busy: 0, wel: 0 } }),
    tile_store_o_128_erase_block: () => ({ nextState: { mode: MODE_STANDBY, busy: 0, wel: 0 } }),
    tile_store_o_128_erase_chip: () => ({ nextState: { mode: MODE_STANDBY, busy: 0, wel: 0 } }),
    tile_store_o_128_wait_ready: () => ({ scalar: 1, nextState: { busy: 0, mode: MODE_STANDBY } }),
    // ── power ──
    tile_store_o_128_deep_power_down: () => ({ nextState: { mode: MODE_DPD } }),
    tile_store_o_128_release: () => ({ nextState: { mode: MODE_STANDBY } })
  },
  provenance: {
    tile_store_o_128_find: "canonical",
    // JEDEC ID 0x1F4218
    tile_store_o_128_read_jedec_id: "canonical",
    tile_store_o_128_read_status: "canonical",
    // WIP/WEL bits
    tile_store_o_128_is_busy: "canonical",
    tile_store_o_128_get_capacity: "canonical",
    // 16 MB
    tile_store_o_128_write_enable: "canonical",
    // WEL set
    tile_store_o_128_erase_sector: "canonical",
    // 4 KB sector erase
    tile_store_o_128_erase_block: "canonical",
    // 64 KB block erase
    tile_store_o_128_erase_chip: "canonical",
    tile_store_o_128_deep_power_down: "canonical",
    tile_store_o_128_release: "canonical",
    tile_store_o_128_init: "inferred",
    tile_store_o_128_wait_ready: "inferred",
    // read/program model the op + power but not actual stored bytes
    tile_store_o_128_read: "inferred",
    tile_store_o_128_fast_read: "inferred",
    tile_store_o_128_page_program: "inferred",
    tile_store_o_128_write: "inferred",
    power: "inferred"
    // mode topology canonical; per-mode currents representative
  },
  // Data out on pad 5 (SPI.MISO / QSPI.IO1) while reading. The part has no
  // RDY/BSY pin — busy is only visible through the status register.
  padOutputs(state) {
    const on = state.vplus_mv >= UVLO_MV ? 1 : 0;
    return { "5": on && state.mode === MODE_READ ? 1 : 0 };
  },
  // Single V+ rail (pad 10) draws per operating mode.
  power(state) {
    const i = state.vplus_mv >= UVLO_MV ? modeUa(state.mode) : 0;
    const label = ["standby", "read", "program", "erase", "deep power-down"][state.mode] ?? "standby";
    return {
      draw_ua: i,
      rails: [
        {
          name: "V+",
          role: "supply",
          v_mv: state.vplus_mv,
          i_ua: i,
          pads: ["10"],
          note: `QSPI NOR flash \u2014 ${label}`
        }
      ]
    };
  }
};
var store_o_128_default = sim;
export {
  store_o_128_default as default
};
