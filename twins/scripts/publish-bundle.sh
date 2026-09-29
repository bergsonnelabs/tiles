#!/usr/bin/env bash
# Commit a built bundle (twins/scripts/build-bundle.mjs) to a bundles branch as
# `<sha>/…`, point `latest.json` at it, and keep only the newest 30 there (git
# history holds the rest). Used by the `bundle` workflow for each destination.
#
#   twins/scripts/publish-bundle.sh BUNDLE_DIR REPO_URL BRANCH SHA FINGERPRINT \
#     "AUTHOR NAME" AUTHOR_EMAIL [MESSAGE_SUFFIX]
#
# latest.json: {sha, fingerprint, contract, publishedAt}. A branch that doesn't
# exist yet is started as an orphan; an existing one keeps whatever else is on
# it (a README), since pruning only looks at directories.
set -euo pipefail

bundle="$1" url="$2" branch="$3" sha="$4" fp="$5" name="$6" email="$7" suffix="${8:-}"
dest="$(mktemp -d)"

if git ls-remote --exit-code --heads "$url" "$branch" >/dev/null; then
  git clone --quiet --depth 1 --branch "$branch" "$url" "$dest"
else
  git init --quiet "$dest"
  git -C "$dest" checkout --quiet --orphan "$branch"
  git -C "$dest" remote add origin "$url"
fi
cd "$dest"
rm -rf "$sha"
cp -R "$bundle" "$sha"
jq -n --arg sha "$sha" --arg fp "$fp" \
  --argjson contract "$(jq .contract "$sha/index.json")" \
  --arg at "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
  '{sha: $sha, fingerprint: $fp, contract: $contract, publishedAt: $at}' > latest.json
# Keep the newest 30 bundles on the branch.
ls -1d */ 2>/dev/null | sed 's#/$##' | while read -r d; do
  echo "$(jq -r .generatedAt "$d/index.json") $d"
done | sort -r | tail -n +31 | awk '{print $2}' | xargs -r rm -rf
git add -A
git -c user.name="$name" -c user.email="$email" \
  commit --quiet -m "bundle ${sha::7} (fingerprint $fp)${suffix:+ $suffix}"
git push --quiet origin "HEAD:$branch"
