#!/usr/bin/env bash
#
# zrecast — the CI matrix, run locally.
#
# This mirrors .github/workflows/ci.yml so a failure can be reproduced and fixed
# on your own machine instead of in a pull request. Install it as a pre-push
# hook with ci/install-hooks.sh to catch problems before they are pushed at all.
#
# The one difference from the hosted run: CI executes the suite on Linux, macOS
# and Windows, whereas this executes it on whichever host you are on and
# cross-compiles the rest.
#
# Three jobs stay out of this script and run on their own: ci/probe.sh
# rebuilds the suite once per probe, ci/determinism.sh needs docker, and
# ci/verify-vendor.sh needs network.
#
# Usage:
#   ci/run.sh                 # full matrix
#   ci/run.sh --quick         # native Debug only, for the inner loop
#   ci/run.sh --list          # print the step names without running them
#
# ZIG overrides the zig invocation (e.g. ZIG="path/to/zig").
#
# Exits non-zero if any step fails, after running every step — a single failure
# should not hide the others.

set -uo pipefail
cd "$(dirname "$0")/.."

ZIG="${ZIG:-zig}"

QUICK=0
LIST=0
case "${1:-}" in
  --quick) QUICK=1 ;;
  --list) LIST=1 ;;
esac

if [ -t 1 ]; then
  RED=$'\033[31m'; GREEN=$'\033[32m'; DIM=$'\033[2m'; BOLD=$'\033[1m'; OFF=$'\033[0m'
else
  RED=; GREEN=; DIM=; BOLD=; OFF=
fi

PASSED=0
FAILED=0
FAILED_NAMES=()

# run <name> <command...>
run() {
  local name="$1"; shift
  if [ $LIST -eq 1 ]; then
    printf '  %s\n' "$name"
    return 0
  fi
  printf '  %-46s' "$name"
  local start output status
  start=$(date +%s)
  output=$("$@" 2>&1)
  status=$?
  local elapsed=$(( $(date +%s) - start ))

  if [ $status -eq 0 ]; then
    printf '%sok%s %s(%ds)%s\n' "$GREEN" "$OFF" "$DIM" "$elapsed" "$OFF"
    PASSED=$((PASSED + 1))
  else
    printf '%sFAILED%s %s(%ds)%s\n' "$RED" "$OFF" "$DIM" "$elapsed" "$OFF"
    FAILED=$((FAILED + 1))
    FAILED_NAMES+=("$name")
    printf '%s' "$output" | sed 's/^/      | /' | head -40
  fi
}

section() { printf '\n%s%s%s\n' "$BOLD" "$1" "$OFF"; }

# skip <name> <reason> — a roster step this host cannot execute, said out loud
# rather than left off the list, so --list counts the same steps everywhere.
skip() { printf '  %-46s%sskipped (%s)%s\n' "$1" "$DIM" "$2" "$OFF"; }

if [ $LIST -eq 0 ]; then
  printf '%szrecast local CI%s  %s%s%s\n' "$BOLD" "$OFF" "$DIM" "$($ZIG version)" "$OFF"
fi

#-----------------------------------------------------------------------------
section 'Hygiene'
#-----------------------------------------------------------------------------

# Only our own Zig sources: libs/recastnavigation is vendored verbatim and must
# not be reformatted, or the next re-vendor becomes an unreadable diff.
run 'zig fmt (src, build.zig, examples, consumer)' \
  $ZIG fmt --check src build.zig examples tests/consumer

# Every zrecast header the umbrella pulls in has to be installed with the
# library, or a C consumer gets an umbrella header that does not resolve. The
# consumer test catches it too, but only after a full build.
run 'installed headers cover every include' bash -c '
  comm -23 \
    <(grep -hoE "#include \"zrecast_[a-z_]+\.h\"" ffi/zrecast.h 2>/dev/null |
      sed "s/#include \"//; s/\"//" | sort -u) \
    <(grep -oE "ffi/zrecast_[a-z_]+\.h" build.zig | sed "s|ffi/||" | sort -u) |
  grep . && { echo "not in build.zig'"'"'s installHeader list"; exit 1; }; exit 0'

# Comments stay documentation rather than becoming a diary. The register half
# of this is the substantive rule; the length caps are a backstop.
run 'comment standard' ci/check-comments.sh

# A script without its executable bit runs fine from `bash script.sh` locally
# and then fails as `./script.sh` in CI or a hook — the bit lives in the git
# index, which is what this checks.
run 'scripts are executable in the index' ci/check-executable.sh

# This script and ci.yml must exercise the same -D option sets, or "green
# locally" and "green hosted" quietly become different claims.
run 'run.sh mirrors ci.yml' ci/check-mirror.sh

#-----------------------------------------------------------------------------
section 'Tests — native'
#-----------------------------------------------------------------------------

# Every optimize mode with Zig's C undefined-behaviour sanitizer both on and
# off. On, because that is what would catch UB in the vendored C++ and in the
# FFI layer; off, because that is how a consumer will actually build, and the
# sanitizer changes code generation enough to be worth testing without.
run 'test Debug (UBSan on)' $ZIG build test -Doptimize=Debug -Dsanitize_c=true
run 'test Debug (UBSan off)' $ZIG build test -Doptimize=Debug -Dsanitize_c=false

if [ $QUICK -eq 0 ]; then
  for mode in ReleaseSafe ReleaseFast ReleaseSmall; do
    run "test $mode (UBSan on)" $ZIG build test -Doptimize="$mode" -Dsanitize_c=true
    run "test $mode (UBSan off)" $ZIG build test -Doptimize="$mode" -Dsanitize_c=false
  done

  # The C boundary on its own, with no Zig in the picture.
  run 'test-c (C ABI standalone)' $ZIG build test-c

  # On a Windows host the MSVC target is native, so the suite runs on it, the
  # same two arms CI's Windows leg has. Zig's default ABI on Windows is gnu,
  # and the MSVC branches in build.zig would otherwise never execute here.
  # Any other host lacks Microsoft's standard library and reports the two
  # steps as skipped.
  case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*) msvc_host=1 ;;
    *) msvc_host=0 ;;
  esac
  if [ $LIST -eq 1 ] || [ $msvc_host -eq 1 ]; then
    run 'test x86_64-windows-msvc' $ZIG build test -Dtarget=x86_64-windows-msvc
    run 'test-c x86_64-windows-msvc' $ZIG build test-c -Dtarget=x86_64-windows-msvc
  else
    skip 'test x86_64-windows-msvc' 'needs a Windows host'
    skip 'test-c x86_64-windows-msvc' 'needs a Windows host'
  fi
fi

#-----------------------------------------------------------------------------
if [ $QUICK -eq 0 ]; then
section 'Cross-compilation'
#-----------------------------------------------------------------------------

# Compile-only. These prove the sources and build graph are portable; the tests
# above are what prove behaviour, on this host. CI executes the suite on Linux,
# macOS and Windows as well.
for target in \
  x86_64-linux-gnu \
  aarch64-linux-gnu \
  x86_64-linux-musl \
  aarch64-linux-musl \
  x86_64-windows-gnu \
  aarch64-windows-gnu \
  x86_64-macos \
  aarch64-macos
do
  run "build $target" $ZIG build -Dtarget="$target"
done

# x86_64-windows-msvc is absent here because it needs the Microsoft standard
# library, which a non-Windows host does not have. CI builds it explicitly on
# the Windows runner — explicitly, because Zig's default ABI for a Windows host
# is gnu, so a plain `zig build` there would never reach the MSVC branches.

#-----------------------------------------------------------------------------
section 'Build configurations'
#-----------------------------------------------------------------------------

run 'shared library' $ZIG build -Dshared=true
run 'asserts off' $ZIG build -Denable_asserts=false
run 'ReleaseFast + asserts on' $ZIG build -Doptimize=ReleaseFast -Denable_asserts=true

#-----------------------------------------------------------------------------
section 'Consumer'
#-----------------------------------------------------------------------------

# A downstream package's view: the module through b.dependency and the
# installed C artifact through linkLibrary + installHeader. Nothing in src/
# exercises either resolution path, so the suite alone cannot catch a break
# here. The asserts-off arm exists because the consumer forwards that option;
# an option nothing forwards would be dead wiring.
run 'consumer (Zig + C via b.dependency)' \
  $ZIG build --build-file tests/consumer/build.zig run
run 'consumer, asserts off' \
  $ZIG build --build-file tests/consumer/build.zig run -Denable_asserts=false

#-----------------------------------------------------------------------------
section 'Coverage'
#-----------------------------------------------------------------------------

# Every public Recast/Detour name has a verdict, and every verdict that says
# "reachable" names a symbol the headers really declare. This step was red
# from the day it was written until the binding was complete — a coverage
# check that only runs once the answer is comfortable is not a check.
run 'coverage (every name has a verdict)' ci/check-coverage.sh

#-----------------------------------------------------------------------------
section 'Documentation'
#-----------------------------------------------------------------------------

# Every number the documents publish, regenerated and compared; a hand-written
# count needs a tools/doc_numbers.txt line saying why it cannot go stale.
run 'docs (generated blocks and numbers)' env ZIG="$ZIG" ci/check-docs.sh
fi

#-----------------------------------------------------------------------------
[ $LIST -eq 1 ] && exit 0

printf '\n'
if [ $FAILED -eq 0 ]; then
  printf '%s%d passed, 0 failed%s\n' "$GREEN" "$PASSED" "$OFF"
  exit 0
fi

printf '%s%d passed, %d FAILED%s\n' "$RED" "$PASSED" "$FAILED" "$OFF"
for name in "${FAILED_NAMES[@]}"; do
  printf '  %s- %s%s\n' "$RED" "$name" "$OFF"
done
exit 1
