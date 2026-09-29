#!/usr/bin/env node
/**
 * Build the tiles bundle: everything Studio needs to know about the tiles at one
 * tiles commit, as files it can load at runtime instead of copies baked into
 * the web repo.
 *
 *   index.json                  what's here, with a sha256 per file
 *   core.json                   the Core SDK host catalog (public)
 *   generics.js                 the passive parts' twins (batteries, USB, …)
 *   tiles/<Tile>/manifest.json  the driver's Studio manifest   (manifests/tile_*.json)
 *   tiles/<Tile>/docs.json      the driver's parsed header      (manifests/tile-docs/*.json)
 *   tiles/<Tile>/twin.js        the twin, compiled to one ES module (default export)
 *   tiles/<Tile>/def-<rev>.json each revision's definition       (definitions/*.json, minus `twin`)
 *
 * One directory per tile so the site can hand a viewer only the tiles they may
 * see (a dark tile's files never leave the server for anyone else).
 *
 * `--overlay <dir>` (repeatable) adds a checkout of a private tiles repo
 * (tiles-alpha, tiles-internal, tile-<family>-<name>): its definitions/*.json
 * join the public ones, and win where both have the same file, since the
 * private repos are synced straight from the DB and public definitions/ can
 * trail it by an unmerged sync PR. Only definitions are read from overlays
 * so far; their drivers/ and twins/src/sims/ are not.
 *
 * The bundle's id (index `sha`, and the `<id>/` it is published under) names
 * everything that went in: sha1 of `tiles@<tiles commit>` and each overlay's
 * `<repo>@<commit>`, sorted, one per line. So a private definition change
 * makes a new bundle at the same tiles commit, and the site, which caches
 * `<id>/…` for good, sees it. The id is 40 hex like a commit sha, but it is
 * not one: the tiles commit is `tilesSha`, the overlay commits `overlays`.
 * An overlay must be a git checkout (its HEAD is part of the id).
 *
 *   node scripts/build-bundle.mjs --out <dir> [--sha <tiles sha>] [--fingerprint <fp>]
 *                                 [--overlay <dir> …]
 */
import { execFileSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readdirSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { basename, dirname, join, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { build } from 'esbuild';

const HERE = dirname(fileURLToPath(import.meta.url));
const TWINS = resolve(HERE, '..');
const TILES = resolve(TWINS, '..');

const arg = (name) => {
  const i = process.argv.indexOf(`--${name}`);
  return i > 0 ? process.argv[i + 1] : undefined;
};
const OUT = resolve(arg('out') ?? join(TWINS, 'dist', 'bundle'));
const SHA = arg('sha') ?? '';
const FINGERPRINT = arg('fingerprint') ?? '';
const OVERLAYS = process.argv.flatMap((a, i) =>
  a === '--overlay' && process.argv[i + 1] ? [resolve(process.argv[i + 1])] : [],
);

// ── bundle id: the tiles commit plus each overlay's commit ───────────────────
const overlays = OVERLAYS.map((dir) => {
  let commit;
  try {
    commit = execFileSync('git', ['-C', dir, 'rev-parse', 'HEAD'], { encoding: 'utf8' }).trim();
  } catch {
    throw new Error(`--overlay ${dir}: not a git checkout (its commit is part of the bundle id)`);
  }
  return `${basename(dir)}@${commit}`;
}).sort();
const ID = createHash('sha1')
  .update([`tiles@${SHA}`, ...overlays].join('\n'))
  .digest('hex');

rmSync(OUT, { recursive: true, force: true });
mkdirSync(OUT, { recursive: true });

const files = {};
const write = (rel, content) => {
  const path = join(OUT, rel);
  mkdirSync(dirname(path), { recursive: true });
  writeFileSync(path, content);
  files[rel] = createHash('sha256').update(content).digest('hex');
};
const readJson = (p) => JSON.parse(readFileSync(p, 'utf8'));
const json = (v) => JSON.stringify(v, null, 2) + '\n';

/** Compile one entry to a self-contained ES module (twins import types only, so
 * this is a transpile plus the registry's glue). */
async function compile(options) {
  const r = await build({
    bundle: true,
    format: 'esm',
    platform: 'neutral',
    target: 'es2022',
    write: false,
    logLevel: 'silent',
    ...options,
  });
  return r.outputFiles[0].text;
}

// ── contract version ─────────────────────────────────────────────────────────
const contract = Number(
  /export const CONTRACT_VERSION = (\d+);/.exec(
    readFileSync(join(TWINS, 'src', 'tileSim.ts'), 'utf8'),
  )?.[1],
);
if (!Number.isInteger(contract)) throw new Error('CONTRACT_VERSION not found in src/tileSim.ts');

// ── per-tile: twin, manifest, docs, definitions ──────────────────────────────
const tiles = {};
const entry = (tile) => (tiles[tile] ??= { files: {} });

const scratch = join(OUT, '.scratch');
mkdirSync(scratch, { recursive: true });
for (const f of readdirSync(join(TWINS, 'src', 'sims')).filter((n) => n.endsWith('.ts'))) {
  const code = await compile({ entryPoints: [join(TWINS, 'src', 'sims', f)] });
  // Load it once to learn which tile it is (the module's `tile` field).
  const probe = join(scratch, f.replace(/\.ts$/, '.mjs'));
  writeFileSync(probe, code);
  const twin = (await import(pathToFileURL(probe).href)).default;
  if (!twin?.tile) throw new Error(`${f}: no default export with a 'tile'`);
  const rel = `tiles/${twin.tile}/twin.js`;
  write(rel, code);
  entry(twin.tile).files.twin = rel;
}
rmSync(scratch, { recursive: true, force: true });

for (const f of readdirSync(join(TILES, 'manifests')).filter((n) => /^tile_.*\.json$/.test(n))) {
  const m = readJson(join(TILES, 'manifests', f));
  const rel = `tiles/${m.tile}/manifest.json`;
  write(rel, json(m));
  entry(m.tile).files.manifest = rel;
}

for (const f of readdirSync(join(TILES, 'manifests', 'tile-docs')).filter((n) => n.endsWith('.json'))) {
  const d = readJson(join(TILES, 'manifests', 'tile-docs', f));
  const tile = d.display_name ?? `${d.tile_family}.${d.tile_name}`;
  const rel = `tiles/${tile}/docs.json`;
  write(rel, json(d));
  entry(tile).files.docs = rel;
}

// Public first, then each overlay, so an overlay's copy of a revision wins.
const defDirs = [join(TILES, 'definitions'), ...OVERLAYS.map((o) => join(o, 'definitions'))];
const defPaths = defDirs.flatMap((dir) =>
  existsSync(dir)
    ? readdirSync(dir)
        .filter((n) => n.endsWith('.json'))
        .map((n) => join(dir, n))
    : [],
);
for (const p of defPaths) {
  const { twin: _twin, ...def } = readJson(p);
  if (!def.family || !def.name || !def.rev) continue;
  const tile = `${def.family}.${def.name}`;
  const rel = `tiles/${tile}/def-${def.rev}.json`;
  write(rel, json(def));
  (entry(tile).files.definitions ??= {})[def.rev] = rel;
}

// ── shared: the Core catalog and the passive parts ───────────────────────────
write('core.json', json(readJson(join(TILES, 'manifests', 'core.json'))));
write(
  'generics.js',
  await compile({
    stdin: {
      contents:
        "import { sources } from './sources';\n" +
        "import { loads } from './loads';\n" +
        "import { switches } from './switches';\n" +
        'export default [...sources, ...loads, ...switches];\n',
      resolveDir: join(TWINS, 'src', 'generics'),
      loader: 'ts',
    },
  }),
);

// ── index ────────────────────────────────────────────────────────────────────
const index = {
  schema: 1,
  contract,
  sha: ID,
  tilesSha: SHA,
  overlays,
  fingerprint: FINGERPRINT,
  generatedAt: new Date().toISOString(),
  core: 'core.json',
  generics: 'generics.js',
  tiles: Object.fromEntries(Object.entries(tiles).sort(([a], [b]) => a.localeCompare(b))),
  files: Object.fromEntries(Object.entries(files).sort(([a], [b]) => a.localeCompare(b))),
};
writeFileSync(join(OUT, 'index.json'), json(index));
console.log(
  `bundle ${ID}: ${Object.keys(tiles).length} tiles, ${Object.keys(files).length} files, contract ${contract} → ${OUT}`,
);
