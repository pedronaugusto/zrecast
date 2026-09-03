#!/usr/bin/env bash
#
# zrecast — cook the fixture on every target this host can reach, and compare.
#
# The suite already asserts that one target cooking twice gets the same bytes
# ("a cook is deterministic"). What it cannot see from inside one process is
# how the bytes compare across targets, so this script builds the test binary
# per target, runs it wherever it can run — natively, under Rosetta, in a
# container — and collects the three cook digests each run prints when
# ZRECAST_COOK_DIGESTS is set.
#
# The comparison is scoped to what is true. Targets sharing a C library must
# agree: with -ffp-contract=off pinned and no libm in the bake, architecture
# is not allowed to move a byte, and a musl or glibc pair that disagrees FAILS
# here. Targets with DIFFERENT C libraries are compared and reported, not
# gated: upstream sorts BV items, holes and diagonals with qsort, ties fall to
# the implementation, and that divergence is documented in UPSTREAM.md rather
# than patched over in the vendored tree.
#
# Three outcomes, never two. A target that ran, a target that FAILED, and a
# target this host could not run at all — the last counted and named in the
# summary rather than dropped, so the result can never read as more coverage
# than it was.
#
# Usage:
#   ci/determinism.sh           # every target; unrun targets are reported
#   ci/determinism.sh --strict  # and an unrun target is a failure
#   ci/determinism.sh musl      # only targets whose label matches
#
# ZIG overrides the zig invocation, as in ci/run.sh.

set -uo pipefail
cd "$(dirname "$0")/.."

ZIG="${ZIG:-zig}"

STRICT=0
if [ "${1:-}" = "--strict" ]; then STRICT=1; shift; fi
FILTER="${1:-}"

if [ -t 1 ]; then
  RED=$'\033[31m'; GREEN=$'\033[32m'; YELLOW=$'\033[33m'
  DIM=$'\033[2m'; BOLD=$'\033[1m'; OFF=$'\033[0m'
else RED=; GREEN=; YELLOW=; DIM=; BOLD=; OFF=; fi

work=$(mktemp -d); trap 'rm -rf "$work"' EXIT
mkdir -p "$work/digests"

RAN=0
FAILED=0
UNRUN=0
RAN_LABELS=()
RAN_GROUPS=()
FAILED_NAMES=()
UNRUN_NAMES=()

wanted() { case "$1" in *"$FILTER"*) return 0 ;; *) return 1 ;; esac; }

# macOS ships no coreutils `timeout`, and a docker pull against a registry this
# host cannot reach otherwise hangs the script with no output at all — which is
# the failure mode this script exists to avoid in the first place.
bounded() { # bounded <seconds> <command...>
  local seconds="$1"; shift
  "$@" &
  local pid=$!
  local waited=0
  while kill -0 "$pid" 2> /dev/null; do
    if [ "$waited" -ge "$seconds" ]; then
      kill -9 "$pid" 2> /dev/null
      wait "$pid" 2> /dev/null
      return 124
    fi
    sleep 1
    waited=$((waited + 1))
  done
  wait "$pid"
}

bad()   { printf '  %-34s %s%s%s\n' "$1" "$RED" "$2" "$OFF"
          FAILED=$((FAILED + 1)); FAILED_NAMES+=("$1 — $2"); }
# Not a verdict on the cook: this host could not put the binary in front of a
# machine that would run it. Counted and named, never folded into the pass.
unrun() { printf '  %-34s %sunrun%s %s(%s)%s\n' "$1" "$YELLOW" "$OFF" "$DIM" "$2" "$OFF"
          UNRUN=$((UNRUN + 1)); UNRUN_NAMES+=("$1 — $2"); }

# A run that produced its three digest lines. The digests land in a file named
# by the label, and the label joins its C-library group for the comparison.
collect() { # collect <label> <group> <output>
  local label="$1" group="$2" output="$3"
  local lines
  lines=$(printf '%s\n' "$output" | grep '^cook digest ' | sort)
  if [ "$(printf '%s\n' "$lines" | grep -c .)" -ne 3 ]; then
    bad "$label" "suite passed but printed no digests"
    return 1
  fi
  RAN=$((RAN + 1))
  RAN_LABELS+=("$label")
  RAN_GROUPS+=("$group")
  printf '%s\n' "$lines" > "$work/digests/$RAN"
  printf '  %-34s %sran%s %s(%s)%s\n' "$label" "$GREEN" "$OFF" "$DIM" "$group" "$OFF"
}

# Built here and run here: native, or close enough that the OS will execute it.
local_target() { # local_target <label> <group> <extra zig args...>
  local label="$1" group="$2"; shift 2
  wanted "$label" || return 0
  local prefix="$work/local-$RAN-$UNRUN-$FAILED"
  if ! out=$($ZIG build test-artifact -Doptimize=Debug --prefix "$prefix" "$@" 2>&1); then
    bad "$label" "FAILED to build"
    printf '%s' "$out" | sed 's/^/      | /' | head -20
    return 0
  fi
  local binary
  binary=$(find "$prefix/bin" -name 'zrecast-tests*' -type f | head -1)
  if [ -z "$binary" ]; then
    bad "$label" "test-artifact installed no binary"
    return 0
  fi
  if ! out=$(ZRECAST_COOK_DIGESTS=1 bounded "${ZRECAST_RUN_TIMEOUT:-600}" \
               "$binary" 2>&1); then
    case "$out" in
      # An x86-64 binary on an Apple-silicon host with no Rosetta installed.
      *"Bad CPU type"*|*"cannot execute binary"*)
        unrun "$label" "this host cannot execute the binary" ;;
      *) bad "$label" "suite FAILED"
         printf '%s' "$out" | sed 's/^/      | /' | head -20 ;;
    esac
    return 0
  fi
  collect "$label" "$group" "$out"
}

# Runs in a container: cross-compiled here, executed there. The image only has
# to be able to start the binary, so it carries no toolchain.
container_target() { # container_target <label> <group> <zig target> <image> <platform>
  local label="$1" group="$2" zig_target="$3" image="$4" platform="$5"
  wanted "$label" || return 0
  if ! command -v docker > /dev/null 2>&1; then
    unrun "$label" "no docker on this host"
    return 0
  fi
  if ! docker info > /dev/null 2>&1; then
    unrun "$label" "the docker daemon is not running"
    return 0
  fi

  # Fetched once, with a bound: an image that is not here and cannot be pulled
  # is a target this host cannot run, not a reason to wait forever.
  if ! docker image inspect "$image" > /dev/null 2>&1; then
    if ! bounded "${ZRECAST_PULL_TIMEOUT:-60}" \
         docker pull --platform "$platform" "$image" > /dev/null 2>&1; then
      unrun "$label" "$image is not local and could not be fetched"
      return 0
    fi
  fi

  local prefix="$work/$zig_target"
  if ! out=$($ZIG build test-artifact -Doptimize=Debug \
               -Dtarget="$zig_target" --prefix "$prefix" 2>&1); then
    bad "$label" "FAILED to build"
    printf '%s' "$out" | sed 's/^/      | /' | head -20
    return 0
  fi

  if ! out=$(bounded "${ZRECAST_RUN_TIMEOUT:-600}" \
               docker run --rm --platform "$platform" \
               -e ZRECAST_COOK_DIGESTS=1 \
               -v "$prefix:/zrecast:ro" "$image" \
               /zrecast/bin/zrecast-tests 2>&1); then
    # A foreign architecture without emulation cannot even start the binary,
    # and that is a gap in what was checked rather than a failure of the code.
    case "$out" in
      *"exec format error"*|*"cannot execute binary"*|*"no matching manifest"*)
        unrun "$label" "$platform cannot execute here" ;;
      *"failed to resolve"*|*"dial tcp"*|*"timeout"*|*"connection refused"*|\
      *"i/o timeout"*|*"TLS handshake"*)
        unrun "$label" "$image could not be fetched" ;;
      *) bad "$label" "suite FAILED"
         printf '%s' "$out" | sed 's/^/      | /' | head -20 ;;
    esac
    return 0
  fi
  collect "$label" "$group" "$out"
}

printf '%szrecast cook determinism%s  %s%s%s\n\n' \
  "$BOLD" "$OFF" "$DIM" "$($ZIG version)" "$OFF"

case "$(uname -sm)" in
  "Darwin arm64")
    local_target 'aarch64-macos (native)' macos
    # Rosetta 2 runs the x86-64 build on this host, which is the whole
    # cross-architecture question: AArch64 mandates FMA and the x86-64 baseline
    # has none, so an unpinned contraction mode shows up here first — and both
    # builds share one libSystem, so their digests are gated, not just shown.
    local_target 'x86_64-macos (rosetta)' macos -Dtarget=x86_64-macos
    ;;
  Linux*)
    # The host's own C library decides which comparison group the native run
    # joins; a musl distribution is a musl target, not a glibc one.
    if ldd --version 2>&1 | grep -qi musl; then host_group=musl
    else host_group=gnu; fi
    local_target "$(uname -sm) (native)" "$host_group"
    ;;
  *)
    local_target "$(uname -sm) (native)" ucrt
    ;;
esac

container_target 'aarch64-linux-musl (docker)' musl aarch64-linux-musl alpine:3.20 linux/arm64
container_target 'aarch64-linux-gnu (docker)'  gnu  aarch64-linux-gnu  debian:bookworm-slim linux/arm64
container_target 'x86_64-linux-musl (docker)'  musl x86_64-linux-musl  alpine:3.20 linux/amd64
container_target 'x86_64-linux-gnu (docker)'   gnu  x86_64-linux-gnu   debian:bookworm-slim linux/amd64

# Within a C-library group the bytes are gated: same qsort, pinned contraction,
# no libm — nothing left that is allowed to differ.
i=0
while [ $i -lt $RAN ]; do
  j=$((i + 1))
  while [ $j -lt $RAN ]; do
    if [ "${RAN_GROUPS[$i]}" = "${RAN_GROUPS[$j]}" ] &&
       ! cmp -s "$work/digests/$((i + 1))" "$work/digests/$((j + 1))"; then
      bad "${RAN_LABELS[$i]} vs ${RAN_LABELS[$j]}" \
        "DISAGREE within ${RAN_GROUPS[$i]}"
      diff "$work/digests/$((i + 1))" "$work/digests/$((j + 1))" |
        sed 's/^/      | /'
    fi
    j=$((j + 1))
  done
  i=$((i + 1))
done

# Across C libraries the bytes are compared and reported. A difference here is
# upstream's qsort tie order, documented in UPSTREAM.md — worth seeing, not a
# failure.
if [ $RAN -gt 1 ]; then
  distinct=$(cat "$work/digests"/* | sort -u | grep -c '^cook digest untiled')
  printf '\n'
  if [ "$distinct" -eq 1 ] &&
     [ "$(cat "$work/digests"/* | sort -u | grep -c .)" -eq 3 ]; then
    printf '%severy target reached produced identical bytes%s\n' "$GREEN" "$OFF"
  else
    printf '%sC libraries disagree at documented qsort tie sites (UPSTREAM.md):%s\n' \
      "$YELLOW" "$OFF"
    i=0
    while [ $i -lt $RAN ]; do
      printf '  %s%s%s\n' "$DIM" "${RAN_LABELS[$i]} (${RAN_GROUPS[$i]})" "$OFF"
      sed 's/^/    /' "$work/digests/$((i + 1))"
      i=$((i + 1))
    done
  fi
fi

printf '\n'
printf '%s%d ran%s' "$GREEN" "$RAN" "$OFF"
[ $FAILED -ne 0 ] && printf ', %s%d failed%s' "$RED" "$FAILED" "$OFF"
[ $UNRUN -ne 0 ] && printf ', %s%d unrun%s' "$YELLOW" "$UNRUN" "$OFF"
printf '\n'
[ $FAILED -ne 0 ] && for n in "${FAILED_NAMES[@]}"; do
  printf '  %s- %s%s\n' "$RED" "$n" "$OFF"
done
[ $UNRUN -ne 0 ] && for n in "${UNRUN_NAMES[@]}"; do
  printf '  %s- %s%s\n' "$YELLOW" "$n" "$OFF"
done

[ $FAILED -eq 0 ] || exit 1
if [ $UNRUN -ne 0 ] && [ $STRICT -eq 1 ]; then
  printf '\n%s--strict: an unrun target is a target this says nothing about%s\n' \
    "$RED" "$OFF" >&2
  exit 1
fi
exit 0
