/**
 * One twin per tile, held to one contract.
 *
 * A tile's twin answers the firmware (host calls, in the worker) AND drives the
 * power layer, indicators and pads (main thread) from the SAME state. These
 * checks keep those two halves honest with each other and with the driver's
 * manifest — the drift they guard against already shipped once: `power()`
 * reading `accel_pm` while the firmware wrote `power_accel`.
 */
import { readdirSync, readFileSync } from 'node:fs';
import { join } from 'node:path';
import { describe, expect, it } from 'vitest';
import { twins } from '../src/index';
import type { SimCallResult, SimState, TileSim } from '../src/tileSim';

/** The slice of a driver manifest (`manifests/tile_*.json`, generated from the
 * driver header) these checks read. */
interface ManifestHost {
  symbol: string;
  params: { name: string }[];
  c_out_buffer?: { type: string; length?: number; name?: string; cap_param?: string };
  c_in_buffer?: { name: string; type: string; length?: number };
  c_out_scalars?: { name: string; type: string }[];
}
interface Manifest {
  tile: string;
  hosts: ManifestHost[];
}

const MANIFEST_DIR = join(__dirname, '..', '..', 'manifests');
const manifests = new Map<string, Manifest>(
  readdirSync(MANIFEST_DIR)
    .filter((f) => /^tile_.*\.json$/.test(f))
    .map((f) => JSON.parse(readFileSync(join(MANIFEST_DIR, f), 'utf8')) as Manifest)
    .map((m) => [m.tile, m]),
);
const tileManifestFor = (tile: string) => manifests.get(tile);

/** The twins with a driver (host calls): what the simulator runs. */
const tileSimRegistry = Object.fromEntries(
  Object.entries(twins).filter(([, t]) => Object.keys(t.hostCalls ?? {}).length > 0),
);

type AnyState = Record<string, number | boolean | string>;

/** Run `fn` against a copy of `state` and report every field it READ that the
 * state doesn't have — a name the twin's firmware half never writes. */
function unknownReads(state: AnyState, fn: (s: AnyState) => unknown): string[] {
  const missing = new Set<string>();
  const probe = new Proxy(
    { ...state },
    {
      get(target, key) {
        if (typeof key === 'string' && !(key in target)) missing.add(key);
        return Reflect.get(target, key);
      },
    },
  );
  fn(probe);
  return [...missing];
}

const driverTwins = Object.entries(tileSimRegistry) as [string, TileSim<SimState>][];

describe('every driver twin', () => {
  it('there are driver twins to check', () => {
    expect(driverTwins.length).toBeGreaterThan(10);
  });

  for (const [tile, twin] of driverTwins) {
    const state = twin.defaultState as AnyState;
    const fields = new Set(Object.keys(state));
    const manifest = tileManifestFor(tile);

    describe(tile, () => {
      it('is registered under its own tile name', () => {
        expect(twin.tile).toBe(tile);
        expect(twins[tile]).toBe(twin);
      });

      it('answers only calls its driver has', () => {
        if (!manifest) return; // no driver manifest yet: nothing to match
        const known = new Set(manifest.hosts.map((h) => h.symbol));
        // `find` / `init` are driver functions the generated main() calls, not
        // program-callable hosts, so the manifest doesn't list them.
        const extra = Object.keys(twin.hostCalls ?? {}).filter(
          (s) => !known.has(s) && !/_(find|init)$/.test(s),
        );
        expect(extra, `host calls with no driver function`).toEqual([]);
      });

      it('returns what each call declares, and writes only fields it has', () => {
        if (!manifest) return;
        const problems: string[] = [];
        for (const [symbol, handler] of Object.entries(twin.hostCalls ?? {})) {
          const host = manifest.hosts.find((h) => h.symbol === symbol);
          if (!host) continue;
          const outScalarNames = new Set((host.c_out_scalars ?? []).map((s) => s.name));
          const inName = host.c_in_buffer?.name;
          const capOut = host.c_out_buffer?.cap_param ? host.c_out_buffer.name : undefined;
          const fixedLen =
            host.c_out_buffer && !host.c_out_buffer.cap_param
              ? host.c_out_buffer.length
              : undefined;
          const scalarArgs = host.params.filter(
            (p) => p.name !== inName && p.name !== capOut && !outScalarNames.has(p.name),
          ).length;
          let result: SimCallResult<SimState> | void;
          try {
            result = handler({
              state: { ...state },
              args: new Array(scalarArgs).fill(0),
              ...(inName
                ? { bufferIn: { [inName]: new Array(host.c_in_buffer?.length ?? 4).fill(0) } }
                : {}),
              ...(capOut ? { caps: { [capOut]: 8 } } : {}),
            });
          } catch (e) {
            problems.push(`${symbol} threw: ${e instanceof Error ? e.message : String(e)}`);
            continue;
          }
          if (!result) continue;
          if (result.array && fixedLen === undefined)
            problems.push(`${symbol} returns an array but its driver function returns none`);
          if (result.array && fixedLen !== undefined && result.array.length > fixedLen)
            problems.push(
              `${symbol} returns ${result.array.length} values, driver has ${fixedLen}`,
            );
          for (const k of Object.keys(result.out ?? {}))
            if (k !== capOut) problems.push(`${symbol} fills '${k}', not a caller-sized out array`);
          for (const k of Object.keys(result.outScalars ?? {}))
            if (!outScalarNames.has(k))
              problems.push(`${symbol} sets out-scalar '${k}' the driver lacks`);
          for (const k of Object.keys(result.nextState ?? {}))
            if (!fields.has(k))
              problems.push(`${symbol} writes '${k}', which is not in defaultState`);
        }
        expect(problems).toEqual([]);
      });

      it('power, indicators, pads and ticks read only fields the state has', () => {
        const reads = [
          ...(twin.power ? unknownReads(state, (s) => twin.power!(s, { padVoltage: {} })) : []),
          ...(twin.indicators
            ? unknownReads(state, (s) => twin.indicators!(s, { padVoltage: {} }))
            : []),
          ...(twin.padOutputs ? unknownReads(state, (s) => twin.padOutputs!(s)) : []),
          ...(twin.deriveState
            ? unknownReads(state, (s) => twin.deriveState!(s, { t: 1000 }))
            : []),
        ];
        expect([...new Set(reads)]).toEqual([]);
      });

      it('ticks change only fields it has', () => {
        const next = twin.deriveState?.({ ...state }, { t: 1000 }) ?? {};
        expect(Object.keys(next).filter((k) => !fields.has(k))).toEqual([]);
      });

      it('controls and stimuli move fields it has', () => {
        const stimulusFields = (twin.stimuli ?? []).flatMap((g) =>
          g.controls.flatMap((c) =>
            c.kind === 'slider'
              ? [c.field, ...(c.maxBy ? [c.maxBy.field] : [])]
              : c.kind === 'toggle'
                ? [...c.fields]
                : [c.accel.x, c.accel.y, c.accel.z],
          ),
        );
        const bad = [...(twin.controls ?? []).map((c) => c.field), ...stimulusFields].filter(
          (f) => !fields.has(f),
        );
        expect(bad).toEqual([]);
      });

      it('pad outputs are keyed by the pad numbers of the tile', () => {
        const keys = Object.keys(twin.padOutputs?.(state) ?? {});
        expect(keys.filter((k) => !/^\d+$/.test(k))).toEqual([]);
      });
    });
  }
});

describe('every twin file stands alone', () => {
  // The portal runs twin SOURCE with no module loader, and a runtime bundle
  // ships each twin as one file: a twin may import types only.
  const dirs = ['sims', 'generics'].map((d) => join(__dirname, '..', 'src', d));
  for (const dir of dirs) {
    for (const f of readdirSync(dir).filter((n) => n.endsWith('.ts'))) {
      it(f, () => {
        const src = readFileSync(join(dir, f), 'utf8');
        const runtimeImports = [...src.matchAll(/^import\s+(?!type\b)[^;]*;/gm)].map((m) => m[0]);
        expect(runtimeImports).toEqual([]);
      });
    }
  }
});
