#!/usr/bin/env node
/**
 * Hold private overlay repos' twins to the contract: the same strict
 * typecheck and contract tests as `npm run check`, over the public twins plus
 * each overlay's twins/src/sims/ and manifests/.
 *
 *   TILES_OVERLAY="/abs/tile-sense-x /abs/tiles-alpha" npm run check:overlay
 *   npm run check:overlay -- /abs/tile-sense-x [/abs/…]
 *
 * An overlay twin imports its types as `../tileSim`, which only exists here:
 * the typecheck merges the two src/ trees with `rootDirs` so it resolves.
 */
import { execFileSync } from 'node:child_process';
import { existsSync, mkdtempSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const TWINS = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const overlays = [...process.argv.slice(2), ...(process.env.TILES_OVERLAY ?? '').split(/\s+/)]
  .filter(Boolean)
  .map((d) => resolve(d));
if (overlays.length === 0) {
  console.error('check-overlay: name the overlay checkouts (arguments or TILES_OVERLAY)');
  process.exit(2);
}
const srcs = overlays.map((o) => join(o, 'twins', 'src'));
for (const s of srcs) {
  if (!existsSync(join(s, 'sims'))) {
    console.error(`check-overlay: ${s}/sims does not exist`);
    process.exit(2);
  }
}

const bin = (name) => join(TWINS, 'node_modules', '.bin', name);
const tmp = mkdtempSync(join(tmpdir(), 'twins-overlay-'));
try {
  const tsconfig = join(tmp, 'tsconfig.json');
  writeFileSync(
    tsconfig,
    JSON.stringify(
      {
        extends: join(TWINS, 'tsconfig.json'),
        compilerOptions: {
          rootDirs: [join(TWINS, 'src'), ...srcs],
          typeRoots: [join(TWINS, 'node_modules', '@types')],
        },
        include: [join(TWINS, 'src'), join(TWINS, 'test'), ...srcs],
      },
      null,
      2,
    ),
  );
  execFileSync(bin('tsc'), ['--noEmit', '-p', tsconfig], { cwd: TWINS, stdio: 'inherit' });
  execFileSync(bin('vitest'), ['run'], {
    cwd: TWINS,
    stdio: 'inherit',
    env: { ...process.env, TILES_OVERLAY: overlays.join(' ') },
  });
} catch (e) {
  process.exitCode = typeof e?.status === 'number' ? e.status : 1;
} finally {
  rmSync(tmp, { recursive: true, force: true });
}
