#!/usr/bin/env bash
# Commit a built bundle (twins/scripts/build-bundle.mjs) to a bundles branch as
# `<id>/…`, point `latest.json` at it, and keep only the newest 30 there (git
# history holds the rest). Used by the `bundle` workflow.
#
#   twins/scripts/publish-bundle.sh BUNDLE_DIR REPO_URL BRANCH "AUTHOR NAME" AUTHOR_EMAIL
#
# Everything else comes from the bundle's index.json: `sha` (the bundle id,
# which names the tiles commit plus the overlay commits; not a commit itself),
# `tilesSha`, `overlays`, `fingerprint`, `contract`.
# latest.json: {sha, tilesSha, overlays, fingerprint, contract, publishedAt}.
# A bundle whose id is already the latest is not published again: same id,
# same inputs. A branch that doesn't exist yet is started as an orphan; an
# existing one keeps whatever else is on it (a README), since pruning only
# looks at directories.
set -euo pipefail

bundle="$(cd "$1" && pwd)" url="$2" branch="$3" name="$4" email="$5"
id="$(jq -r .sha "$bundle/index.json")"
tiles_sha="$(jq -r '.tilesSha // empty' "$bundle/index.json")"
fp="$(jq -r .fingerprint "$bundle/index.json")"
if ! [[ "$id" =~ ^[0-9a-f]{40}$ ]]; then
  echo "error: $bundle/index.json has no 40-hex bundle id (sha: '$id')" >&2
  exit 1
fi
dest="$(mktemp -d)"

# Work from the temp dir, not the caller's checkout: actions/checkout leaves
# the workflow token in that repo's config as an auth header, which overrides
# the token in $url and can't see another repo ("Repository not found").
cd "$dest"
if git ls-remote --exit-code --heads "$url" "$branch" >/dev/null; then
  git clone --quiet --depth 1 --branch "$branch" "$url" .
  if [ -d "$id" ] && [ "$(jq -r .sha latest.json 2>/dev/null)" = "$id" ]; then
    echo "bundle ${id::7} is already the latest on $branch; nothing to publish."
    exit 0
  fi
else
  git init --quiet
  git checkout --quiet --orphan "$branch"
  git remote add origin "$url"
fi
rm -rf "$id"
cp -R "$bundle" "$id"
jq --arg at "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
  '{sha, tilesSha, overlays, fingerprint, contract, publishedAt: $at}' \
  "$id/index.json" > latest.json
# Keep the newest 30 bundles on the branch.
ls -1d */ 2>/dev/null | sed 's#/$##' | while read -r d; do
  echo "$(jq -r .generatedAt "$d/index.json") $d"
done | sort -r | tail -n +31 | awk '{print $2}' | xargs -r rm -rf
revs="$(jq -r '.overlays // [] | map(split("@") | .[0] + "@" + (.[1][0:7])) | join(", ")' "$id/index.json")"
git add -A
git -c user.name="$name" -c user.email="$email" \
  commit --quiet -m "bundle ${id::7}: tiles@${tiles_sha::7}${revs:+, $revs} (fingerprint $fp)"
git push --quiet origin "HEAD:$branch"
