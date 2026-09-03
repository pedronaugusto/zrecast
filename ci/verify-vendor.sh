#!/usr/bin/env bash
#
# zrecast — prove libs/recastnavigation is upstream, byte for byte.
#
# UPSTREAM.md says the vendored tree is a specific commit, verbatim. On its own
# that is a claim in a markdown file: nothing stops an edit to libs/ from
# landing while the documentation still says otherwise. This script turns the
# claim into a check by fetching that exact commit and diffing. Any local
# change to the vendored tree fails here — behaviour this package needs that
# upstream lacks belongs in ffi/, on our side of the boundary.
#
# It is the reason a git submodule is not needed here. A submodule would have
# git record the upstream commit, which is genuinely useful — but Zig's package
# manager fetches a source archive and never resolves submodules, so consumers
# would receive an empty libs/ and a build that cannot work. This gets the
# guarantee without the breakage.
#
# Needs network, so it is a separate CI job rather than part of ci/run.sh.
#
# Usage: ci/verify-vendor.sh

set -euo pipefail
cd "$(dirname "$0")/.."

# Kept in step with UPSTREAM.md by the check below, so the two cannot drift.
UPSTREAM_URL="https://github.com/recastnavigation/recastnavigation.git"
UPSTREAM_TAG="v1.6.0"
UPSTREAM_COMMIT="6dc1667f580357e8a2154c28b7867bea7e8ad3a7"

# Directories and files copied verbatim from upstream.
VENDORED=(Recast Detour DetourCrowd DetourTileCache DebugUtils
          License.txt README.md CHANGELOG.md)

# Removed deliberately after copying; upstream has them, we do not.
# Documented in UPSTREAM.md under "What was excluded, and why".
EXCLUDE_ARGS=(-x CMakeLists.txt)

if [ -t 1 ]; then
  RED=$'\033[31m'; GREEN=$'\033[32m'; DIM=$'\033[2m'; OFF=$'\033[0m'
else
  RED=; GREEN=; DIM=; OFF=
fi

fail() { printf '%s%s%s\n' "$RED" "$1" "$OFF" >&2; exit 1; }

# The pin must appear in UPSTREAM.md verbatim. If someone bumps one and not the
# other, that is exactly the drift this script exists to catch.
grep -q "$UPSTREAM_COMMIT" UPSTREAM.md ||
  fail "UPSTREAM.md does not mention $UPSTREAM_COMMIT — the pin has drifted."
grep -q "$UPSTREAM_TAG" UPSTREAM.md ||
  fail "UPSTREAM.md does not mention $UPSTREAM_TAG — the pin has drifted."

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

printf '%sfetching %s at %s%s\n' "$DIM" "$UPSTREAM_URL" "$UPSTREAM_TAG" "$OFF"
# autocrlf off, eol lf: on a Windows host a default clone would check upstream
# out with CRLF endings and every text file would "differ" while libs/ is
# byte-identical. The vendored tree is LF (`libs/** -text` in .gitattributes),
# so the reference copy has to be too — this is a byte comparison, not a text
# one.
git clone --quiet --depth 1 --branch "$UPSTREAM_TAG" \
  -c core.autocrlf=false -c core.eol=lf "$UPSTREAM_URL" \
  "$work/upstream" 2>/dev/null || fail "clone failed"

# A tag can be moved; the commit cannot. Check the SHA, not the label.
actual=$(git -C "$work/upstream" rev-parse HEAD)
[ "$actual" = "$UPSTREAM_COMMIT" ] ||
  fail "tag $UPSTREAM_TAG is $actual, expected $UPSTREAM_COMMIT"
printf '%scommit %s confirmed%s\n' "$DIM" "$UPSTREAM_COMMIT" "$OFF"

status=0
for path in "${VENDORED[@]}"; do
  if [ ! -e "libs/recastnavigation/$path" ]; then
    printf '  %-24s %sMISSING locally%s\n' "$path" "$RED" "$OFF"
    status=1
    continue
  fi
  if diff -r "${EXCLUDE_ARGS[@]}" \
       "$work/upstream/$path" "libs/recastnavigation/$path" > "$work/diff" 2>&1
  then
    printf '  %-24s %sidentical%s\n' "$path" "$GREEN" "$OFF"
  else
    printf '  %-24s %sDIFFERS%s\n' "$path" "$RED" "$OFF"
    sed 's/^/      /' "$work/diff" | head -30
    status=1
  fi
done

# Nothing else may live under libs/. A stray file sits outside every comparison
# above, so it would otherwise be an unnoticed local addition to a tree this
# script has just called pristine.
printf '%s\n' "${VENDORED[@]}" > "$work/expected"
find libs/recastnavigation -mindepth 1 -maxdepth 1 |
  sed 's|libs/recastnavigation/||' | sort > "$work/present"
unexpected=$(comm -23 "$work/present" <(sort "$work/expected"))
if [ -n "$unexpected" ]; then
  printf '  %sunexpected entries under libs/recastnavigation:%s\n' "$RED" "$OFF"
  printf '%s\n' "$unexpected" | sed 's/^/      /'
  status=1
fi

# The build compiles an explicit list of sources rather than a glob; every one
# of them has to exist, or a re-vendor has quietly removed a translation unit
# and the failure surfaces as a link error much later.
missing_sources=0
for source in $(grep -o '"libs/recastnavigation/[^"]*\.cpp"' build.zig | tr -d '"'); do
  [ -f "$source" ] || {
    printf '  %ssource listed in build.zig is missing: %s%s\n' "$RED" "$source" "$OFF"
    missing_sources=1
  }
done
[ $missing_sources -eq 0 ] || status=1

if [ $status -ne 0 ]; then
  printf '\n%slibs/recastnavigation is not upstream %s verbatim.%s\n' \
    "$RED" "$UPSTREAM_COMMIT" "$OFF" >&2
  printf 'Revert the local change. Behaviour this package needs that upstream\n' >&2
  printf 'lacks belongs in ffi/, on our side of the boundary.\n' >&2
  exit 1
fi

printf '\n%slibs/recastnavigation matches upstream %s verbatim%s\n' \
  "$GREEN" "$UPSTREAM_COMMIT" "$OFF"
