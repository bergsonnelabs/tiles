// Digital twin for Store.O.128 — Renesas/Adesto AT25QL128A 128 Mbit QSPI NOR flash.
//
// Now driver-backed (tile_store_o_128): the twin mirrors the SPI-NOR command set
// — JEDEC ID, status (WIP/WEL), capacity, read / fast-read, write-enable, page
// program, write (page-split), 4 KB sector / 64 KB block / chip erase, wait-ready,
// deep power-down / release. SPI/QSPI on pads 2/3/4/5/6/9; V+ pad 10 (1.71–2.0 V);
// GND pad 1.
//
// Identity + command/status semantics are canonical (datasheet/driver). Actual
// stored bytes aren't modeled (read/program return/accept no real array data), and
// the per-mode supply currents are representative AT25QL128A figures, so the power
// layer is inferred. Program/erase calls block in the driver: each does
// write_enable → command → wait_ready before returning (tile_store_o_128.c:162-220),
// so by the time the firmware sees the call return the part is idle again (WIP and
// WEL both clear on completion). Only write_enable leaves a latched state.
import type { TileSim } from '../tileSim';

const JEDEC_ID = 0x1f4218; // mfr 0x1F, type 0x42, capacity 0x18 (128 Mbit)
const CAPACITY = 0x1000000; // 16 MB

const SR_WIP = 0x01; // write in progress (busy)
const SR_WEL = 0x02; // write enable latch

const MODE_STANDBY = 0;
const MODE_READ = 1;
const MODE_PROGRAM = 2;
const MODE_ERASE = 3;
const MODE_DPD = 4;

const UVLO_MV = 1710;

// representative AT25QL128A supply currents by mode (µA)
const I_STANDBY_UA = 12;
const I_READ_UA = 10_000;
const I_PROGRAM_UA = 20_000;
const I_ERASE_UA = 25_000;
const I_DPD_UA = 1;

interface State {
  mode: number; // 0 standby, 1 read, 2 program, 3 erase, 4 deep power-down
  wel: number; // write-enable latch
  busy: number; // program/erase in progress (WIP)
  vplus_mv: number;
  [field: string]: number;
}

function modeUa(m: number): number {
  if (m === MODE_READ) return I_READ_UA;
  if (m === MODE_PROGRAM) return I_PROGRAM_UA;
  if (m === MODE_ERASE) return I_ERASE_UA;
  if (m === MODE_DPD) return I_DPD_UA;
  return I_STANDBY_UA;
}

const sim: TileSim<State> = {
  tile: 'Store.O.128',

  defaultState: { mode: MODE_STANDBY, wel: 0, busy: 0, vplus_mv: 1800 },

  controls: [
    { type: 'slider', field: 'mode', label: 'Mode (0std1rd2pg3er4dpd)', min: 0, max: 4, step: 1 },
    {
      type: 'slider',
      field: 'vplus_mv',
      label: 'V+ supply',
      min: 1500,
      max: 2000,
      step: 10,
      unit: 'mV',
    },
  ],

  hostCalls: {
    // ── lifecycle ──
    tile_store_o_128_find: () => ({ scalar: 1 }), // JEDEC ID matches
    tile_store_o_128_init: () => ({
      scalar: 0,
      nextState: { mode: MODE_STANDBY, wel: 0, busy: 0 },
    }),

    // ── identity / status ──
    tile_store_o_128_read_jedec_id: () => ({ scalar: JEDEC_ID }),
    tile_store_o_128_read_status: ({ state }) => ({
      scalar: (state.busy ? SR_WIP : 0) | (state.wel ? SR_WEL : 0),
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
    tile_store_o_128_release: () => ({ nextState: { mode: MODE_STANDBY } }),
  },

  provenance: {
    tile_store_o_128_find: 'canonical', // JEDEC ID 0x1F4218
    tile_store_o_128_read_jedec_id: 'canonical',
    tile_store_o_128_read_status: 'canonical', // WIP/WEL bits
    tile_store_o_128_is_busy: 'canonical',
    tile_store_o_128_get_capacity: 'canonical', // 16 MB
    tile_store_o_128_write_enable: 'canonical', // WEL set
    tile_store_o_128_erase_sector: 'canonical', // 4 KB sector erase
    tile_store_o_128_erase_block: 'canonical', // 64 KB block erase
    tile_store_o_128_erase_chip: 'canonical',
    tile_store_o_128_deep_power_down: 'canonical',
    tile_store_o_128_release: 'canonical',
    tile_store_o_128_init: 'inferred',
    tile_store_o_128_wait_ready: 'inferred',
    // read/program model the op + power but not actual stored bytes
    tile_store_o_128_read: 'inferred',
    tile_store_o_128_fast_read: 'inferred',
    tile_store_o_128_page_program: 'inferred',
    tile_store_o_128_write: 'inferred',
    power: 'inferred', // mode topology canonical; per-mode currents representative
  },

  // Data out on pad 5 (SPI.MISO / QSPI.IO1) while reading. The part has no
  // RDY/BSY pin — busy is only visible through the status register.
  padOutputs(state) {
    const on = state.vplus_mv >= UVLO_MV ? 1 : 0;
    return { '5': on && state.mode === MODE_READ ? 1 : 0 };
  },

  // Single V+ rail (pad 10) draws per operating mode.
  power(state) {
    const i = state.vplus_mv >= UVLO_MV ? modeUa(state.mode) : 0;
    const label =
      ['standby', 'read', 'program', 'erase', 'deep power-down'][state.mode] ?? 'standby';
    return {
      draw_ua: i,
      rails: [
        {
          name: 'V+',
          role: 'supply',
          v_mv: state.vplus_mv,
          i_ua: i,
          pads: ['10'],
          note: `QSPI NOR flash — ${label}`,
        },
      ],
    };
  },
};

export default sim;
