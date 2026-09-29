#!/usr/bin/env bash
# Clone the private tiles overlay repos (tiles-alpha, tiles-internal,
# tile-<family>-<name>) into ROOT/<repo>, for tools/sync_definitions.py
# --private-root and twins/scripts/build-bundle.mjs --overlay.
#
#   GH_TOKEN=<tile-json-sync installation token> \
#     tools/clone_private_repos.sh ROOT [NEEDED_FILE]
#
# Clones every overlay repo the token's installation can see, so one that no
# longer has any tiles still gets emptied by the faithful mirror, plus every
# repo named in NEEDED_FILE (one per line, from sync_definitions.py
# --private-repos-out). A needed repo that doesn't exist, or that the app isn't
# installed on, is reported and skipped: the sync then finds no checkout for it,
# publishes its tiles nowhere and exits 3. Never a public fallback.
#
# It runs in public tiles' Actions, whose logs are public: it prints counts,
# never a private repo's name.
set -euo pipefail

root="$1"
needed="${2:-}"
owner="bergsonnelabs"
pattern='^(tiles-alpha|tiles-internal|tile-[a-z0-9._-]+)$'

mkdir -p "$root"
# Two steps so a failing API call fails the script instead of reading as "none".
all="$(gh api --paginate /installation/repositories -q '.repositories[].name')"
installed="$(grep -E "$pattern" <<<"$all" || true)"
wanted="$(printf '%s\n' $installed $( [ -n "$needed" ] && cat "$needed" ) | sort -u)"

cloned=0 missing=0
for repo in $wanted; do
  if ! grep -qxF "$repo" <<<"$installed"; then
    missing=$((missing + 1))
    continue
  fi
  git clone --quiet --depth 1 \
    "https://x-access-token:${GH_TOKEN}@github.com/$owner/$repo.git" "$root/$repo" 2>/dev/null \
    || { echo "::error::cloning a private overlay repo failed"; exit 1; }
  cloned=$((cloned + 1))
done
echo "cloned $cloned private overlay repos"
if [ "$missing" -gt 0 ]; then
  echo "::error::$missing needed private repo(s) missing, or tile-json-sync isn't installed on them; their tiles are published nowhere"
fi
