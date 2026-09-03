#!/usr/bin/env bash
#
# zrecast — recompute every number the documentation claims.
#
# Every count README.md publishes comes from here and from nowhere else: the
# README carries a generated block that ci/check-docs.sh rebuilds from this
# script and refuses to let drift. A hand-written count is not allowed to
# exist, so there is nothing left that can quietly go stale.
#
# Usage:
#   ci/measurements.sh            # human-readable
#   ci/measurements.sh --kv       # KEY<TAB>VALUE<TAB>DESCRIPTION
#   ci/measurements.sh --markdown # the table README.md's generated block holds
#
# ZIG overrides the compiler used by the one measurement that has to build.
# Nothing here uses `bc`: it is absent from Git Bash, and an absent bc produced
# an EMPTY count rather than an error.

set -euo pipefail
cd "$(dirname "$0")/.."

MODE=${1:-}
ZIG=${ZIG:-zig}

#-----------------------------------------------------------------------------
# The measurements. Each is `emit KEY VALUE DESCRIPTION`, and the description
# is what the README prints, so a measurement is described once.
#-----------------------------------------------------------------------------

keys=()
values=()
descriptions=()
emit() {
  keys+=("$1")
  values+=("$2")
  descriptions+=("$3")
}

sum() { awk '{ total += $1 } END { print total + 0 }'; }

emit version "$(sed -n 's/^ *\.version = "\([^"]*\)".*/\1/p' build.zig.zon)" \
  'version (one home: `build.zig.zon`)'

# The C boundary and its Zig mirror. tests/c_smoke.c and the ABI checks hold
# the two surfaces together; these counts can only disagree through a bug in
# this script.
c_entry_points=$(grep -h '^ZRC_API' ffi/*.h | grep -c .)
emit c_entry_points "$c_entry_points" \
  'C entry points (`ZRC_API` in `ffi/*.h`)'

externs=$(grep -hc '^pub extern fn zrc' src/*.zig | sum)
emit zig_externs "$externs" 'Zig externs (`pub extern fn` in `src/`)'

# The coverage ledger: one verdict per public upstream name, kept complete by
# ci/check-coverage.sh, which cross-checks every count against the headers.
ledger() { cat tools/unbound_*.txt; }
emit upstream_names "$(ledger | grep -c .)" \
  'public Recast/Detour names, each carrying a verdict in `tools/`'
emit verdict_bound "$(ledger | awk -F'\t' '$3 == "BOUND"' | grep -c .)" \
  'of them reachable through the C boundary (`BOUND`)'
emit verdict_language "$(ledger | awk -F'\t' '$3 == "LANGUAGE"' | grep -c .)" \
  'C++-only surface a C boundary cannot carry (`LANGUAGE`), each with the reason'
emit verdict_zig "$(ledger | awk -F'\t' '$3 == "ZIG"' | grep -c .)" \
  'reimplemented on the Zig side (`ZIG`), each naming its mirror'

# What the build reports, not what a grep for `test` finds: a test behind a
# build option would be counted by the grep and never run.
#
# The output is captured before it is parsed, rather than piped straight into
# sed. Piped, a failing build sends its own diagnosis INTO the pipe, `set -e`
# ends this script with nothing on any stream, and ci/check-docs.sh reports
# "generator failed" followed by an empty stderr — a build error rendered as a
# documentation error, naming nothing to act on.
if ! build_log=$(${ZIG} build test --summary all 2>&1); then
  printf '%s\n' "$build_log" >&2
  printf '\nci/measurements.sh: `%s build test` failed, output above.\n' "$ZIG" >&2
  printf 'The test count is what the build reports, so no number here can\n' >&2
  printf 'be recomputed until it passes.\n' >&2
  exit 1
fi
test_count=$(printf '%s\n' "$build_log" |
  sed -n 's/.*run test zrecast-tests \([0-9][0-9]*\) pass.*/\1/p' | head -1)
emit zig_tests_run "$test_count" 'Zig tests `zig build test` executes'
emit c_smoke_assertions "$(grep -c '^ *CHECK(' tests/c_smoke.c)" \
  'assertions in the standalone C smoke test'

emit upstream_translation_units \
  "$(grep -cE '"libs/recastnavigation/[^"]*\.cpp"' build.zig)" \
  'vendored recastnavigation translation units `build.zig` compiles'
emit ffi_source_lines "$(cat ffi/*.h ffi/*.cpp | wc -l | tr -d ' ')" \
  'C boundary lines (`ffi/`)'
emit zig_source_lines "$(cat src/*.zig | wc -l | tr -d ' ')" \
  'Zig source lines (`src/`)'

# One probe per mutated invariant; ci/probe.sh refuses a probe the suite
# survives, so the count is of proofs, not of files.
emit mutation_probes "$(ls ci/probes/*.patch | grep -c .)" \
  'invariants `ci/probe.sh` mutates, each with the test that must notice'

# From ci/run.sh itself, which names every step it would run and runs none.
# Counting `run` lines instead is wrong: the cross-compilation loop is one
# line and several steps.
emit ci_checks "$(bash ci/run.sh --list | grep -c '^  ')" 'steps `ci/run.sh` runs'
emit ci_cross_targets \
  "$(sed -n '/^for target in/,/^do$/p' ci/run.sh | grep -cE '^ +[a-z0-9_]+-')" \
  'further targets `ci/run.sh` cross-compiles'

#-----------------------------------------------------------------------------
# Output
#-----------------------------------------------------------------------------

# A measurement that silently produces nothing is worse than a wrong one: it
# renders as an empty cell and reads as "not applicable". An empty value here
# is what a changed `zig build` summary format looks like from the outside.
for i in "${!keys[@]}"; do
  if [ -z "${values[$i]}" ]; then
    printf 'measurement %s produced no value; its source has changed shape\n' \
      "${keys[$i]}" >&2
    exit 1
  fi
done

if [ "$c_entry_points" != "$externs" ]; then
  printf '\nc_entry_points and zig_externs must match: tests/c_smoke.c and the\n' >&2
  printf 'coverage gate pair the two surfaces, so a difference is a bug in\n' >&2
  printf 'this script.\n' >&2
  exit 1
fi

if [ "$MODE" = "--kv" ]; then
  for i in "${!keys[@]}"; do
    printf '%s\t%s\t%s\n' "${keys[$i]}" "${values[$i]}" "${descriptions[$i]}"
  done
  exit 0
fi

if [ "$MODE" = "--markdown" ]; then
  printf '| | |
|---:|---|
'
  for i in "${!keys[@]}"; do
    printf '| **%s** | %s |
' "${values[$i]}" "${descriptions[$i]}"
  done
  exit 0
fi

for i in "${!keys[@]}"; do
  printf '%-26s %8s  %s\n' "${keys[$i]}" "${values[$i]}" "${descriptions[$i]}"
done
