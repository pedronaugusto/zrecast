#!/usr/bin/env bash
#
# zrecast — mutate an invariant and require a named test to notice.
#
# A test that passes proves nothing on its own: it may be asserting something
# the code could not violate anyway. Each probe in ci/probes/ breaks one
# invariant on purpose and names the test that has to fail because of it. A
# probe the suite survives is a test that was never really testing anything.
#
# This is not theory. The tiled bake removes the border padding from a tile's
# bounds, and the first probe of that passed silently: every tile shifts by the
# same amount, so they still stitch to each other and only their agreement with
# the world is lost. The test only became real once it compared a tiled bake
# against an untiled one.
#
# A probe is a unified diff applicable with `patch -p1` from the repository
# root, carrying two header lines:
#
#   # run: <the command that should notice, run from the tree root>
#   # expect: <text that must appear in that command's failing output>
#
# `run` defaults to the native Debug test build, which is what a probe against
# the library itself wants. A probe against the tooling names its own gate
# instead — ci/check-coverage.sh has invariants of its own, and they are worth
# the same treatment.
#
# Everything runs against a copy of the tree, so the sources here are never
# touched.
#
# Usage:
#   ci/probe.sh                # every probe
#   ci/probe.sh tile-bounds    # only probes whose filename contains this

set -uo pipefail
cd "$(dirname "$0")/.."

# The zig to build with, as in ci/run.sh. On CI this is plain `zig`; on a
# machine shared with another build it can be a wrapper that serialises them.
# Exported so a probe's own `# run:` command and any gate it calls see it too.
ZIG="${ZIG:-zig}"
export ZIG

FILTER="${1:-}"

if [ -t 1 ]; then
  RED=$'\033[31m'; GREEN=$'\033[32m'; DIM=$'\033[2m'; BOLD=$'\033[1m'; OFF=$'\033[0m'
else RED=; GREEN=; DIM=; BOLD=; OFF=; fi

work=$(mktemp -d); trap 'rm -rf "$work"' EXIT

# tar, not rsync: Git Bash on Windows has no rsync, and a copy tool that is
# missing does not fail loudly here — it leaves an empty tree that turns every
# probe into "does not apply". tar exists on all three CI hosts and this one.
copy_tree() {
  rm -rf "$work/tree"
  mkdir -p "$work/tree"
  tar --exclude=.git --exclude=.zig-cache --exclude=zig-out -cf - . |
    tar -xf - -C "$work/tree"
}

# One copy, reused. Each probe is applied and then reversed, so the build cache
# stays warm across the whole run instead of paying a cold build per probe.
copy_tree

CAUGHT=0
MISSED=0
MISSED_NAMES=()

printf '%szrecast mutation probes%s\n\n' "$BOLD" "$OFF"

for probe in ci/probes/*.patch; do
  [ -e "$probe" ] || { printf '%sno probes in ci/probes%s\n' "$RED" "$OFF" >&2; exit 1; }
  name=$(basename "$probe" .patch)
  case "$name" in *"$FILTER"*) ;; *) continue ;; esac

  expect=$(sed -n 's/^# expect: //p' "$probe" | head -1)
  run=$(sed -n 's/^# run: //p' "$probe" | head -1)
  [ -n "$run" ] || run="$ZIG build test -Doptimize=Debug -Dsanitize_c=true"
  if [ -z "$expect" ]; then
    printf '  %-42s %sno "# expect:" line%s\n' "$name" "$RED" "$OFF"
    MISSED=$((MISSED + 1)); MISSED_NAMES+=("$name (malformed)")
    continue
  fi

  if ! patch -s -p1 -d "$work/tree" < "$probe" 2>"$work/patcherr"; then
    printf '  %-42s %sdoes not apply%s\n' "$name" "$RED" "$OFF"
    sed 's/^/      | /' "$work/patcherr" | head -10
    MISSED=$((MISSED + 1)); MISSED_NAMES+=("$name (stale patch)")
    # A patch can fail after applying some of its hunks, and a half-patched
    # copy would then be the baseline every probe after it runs against. Reset
    # rather than reverse: there is no telling how much went in.
    copy_tree
    continue
  fi

  output=$( (cd "$work/tree" && eval "$run") 2>&1 )
  status=$?
  # A reversal that fails leaves the mutation in the copy, and every probe
  # after it then runs against a tree that is not the repository. A full disk
  # did exactly that to a whole sweep once: patch writes failed mid-file,
  # nothing checked them, and the tail of the run reported nonsense.
  if ! patch -s -R -p1 -d "$work/tree" < "$probe"; then
    copy_tree
  fi

  if [ $status -eq 0 ]; then
    printf '  %-42s %sNOT CAUGHT%s\n' "$name" "$RED" "$OFF"
    MISSED=$((MISSED + 1)); MISSED_NAMES+=("$name")
  # Not a pipe: under pipefail, `printf | grep -q` is a race. grep -q exits at
  # the first match and closes the pipe, printf then dies of SIGPIPE when the
  # output is long, and a probe the suite caught reports as missed — seen
  # intermittently for real, one miss in three runs of an unchanged tree.
  elif ! grep -qF "$expect" <<< "$output"; then
    # The suite failed, but not for the reason the probe claims. A mutation that
    # only breaks the build is not evidence that any assertion holds.
    printf '  %-42s %scaught by something else%s\n' "$name" "$RED" "$OFF"
    printf '      %sexpected to see: %s%s\n' "$DIM" "$expect" "$OFF"
    printf '%s' "$output" | sed 's/^/      | /' | head -12
    MISSED=$((MISSED + 1)); MISSED_NAMES+=("$name (wrong failure)")
  else
    printf '  %-42s %scaught%s %s(%s)%s\n' "$name" "$GREEN" "$OFF" "$DIM" "$expect" "$OFF"
    CAUGHT=$((CAUGHT + 1))
  fi
done

printf '\n'
if [ $MISSED -eq 0 ]; then
  printf '%s%d probe(s) caught, 0 missed%s\n' "$GREEN" "$CAUGHT" "$OFF"
  exit 0
fi
printf '%s%d caught, %d MISSED%s\n' "$RED" "$CAUGHT" "$MISSED" "$OFF"
for n in "${MISSED_NAMES[@]}"; do printf '  %s- %s%s\n' "$RED" "$n" "$OFF"; done
exit 1
