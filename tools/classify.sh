#!/usr/bin/env bash
#
# zrecast — the verdict for every public Recast/Detour name that can be
# computed rather than asserted.
#
# The rule: an exclusion is legitimate only when upstream itself marks the
# name internal. Recast has exactly one phrase for that, "generally meant for
# internal use only", and tools/recast_internal.awk finds both places it is
# written. Everything else is a binding, a Zig facility, or a gap — being
# awkward to bind is not a reason.
#
# ci/check-coverage.sh recomputes this and rejects any INTERNAL line whose
# evidence does not match, so the record cannot claim an exclusion the headers
# do not support.
#
#   tools/classify.sh          SYMBOL<TAB>INTERNAL<TAB>evidence
#   tools/classify.sh --open   the public names it cannot justify excluding

set -uo pipefail
cd "$(dirname "$0")/.."

RECAST=libs/recastnavigation
t=$(mktemp -d); trap 'rm -rf "$t"' EXIT

tools/coverage.sh --names | cut -f2 | sort -u > "$t/names"

find "$RECAST/Recast/Include" "$RECAST/Detour/Include" \
     "$RECAST/DetourCrowd/Include" "$RECAST/DetourTileCache/Include" \
     -name '*.h' -print0 |
  xargs -0 awk -f tools/recast_internal.awk /dev/null 2>/dev/null |
  sort -u | awk -F'\t' '!seen[$1]++' > "$t/marked"

awk -F'\t' -v mode="${1:-}" '
  FILENAME ~ /marked$/ { ev[$1] = "upstream " $2 " (" $3 ")"; next }
  {
    # SYMBOL is KIND:OWNER::NAME with an optional /ARITY; the marker scanner
    # works in bare names, so compare on the bare name.
    s = $0
    n = s; sub(/^[a-z]+:[^:]*::/, "", n); sub(/\/[0-9-]+$/, "", n)
    if (n in ev) { if (mode != "--open") print s "\tINTERNAL\t" ev[n] }
    else if (mode == "--open") print s
  }
' "$t/marked" "$t/names"
