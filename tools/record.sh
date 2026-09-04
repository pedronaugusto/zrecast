#!/usr/bin/env bash
#
# zrecast — write tools/unbound_*.txt from its three inputs.
#
#   tools/coverage.sh --names   every public name upstream declares, by area.
#   tools/bindings.tsv          the names that are answered, and how.
#   tools/tranches.awk          which tranche closes each of the rest.
#
# The record used to be edited by hand, one row per name, each gap carrying its
# own sentence of prose. Two things went wrong with that and both are structural
# rather than careless: a name could carry one verdict in one file and a
# different one in another, and a single tranche of work falsified dozens of
# sentences at once while every one of them stayed in the tree looking current.
#
# So the record is generated. Binding a name is one line appended to
# tools/bindings.tsv; moving a whole family of gaps to another tranche is one
# rule edited in tools/tranches.awk. ci/check-coverage.sh runs --check, so a
# record edited by hand fails the build rather than surviving as a lie.
#
# Usage:
#   tools/record.sh            # write the files
#   tools/record.sh --check    # fail if what is checked in differs

set -euo pipefail
cd "$(dirname "$0")/.."

# The record is a checked-in file, so the order of its lines is part of it.
# sort collates by locale, and ':' '~' and letter case rank differently under a
# UTF-8 locale than under C, so an unpinned sort makes the same three inputs
# generate a different file on a different machine. C order is the recorded
# one. Exported, so tools/coverage.sh sorts the harvest the same way.
export LC_ALL=C

CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1

if [ -t 1 ]; then RED=$'\033[31m'; GREEN=$'\033[32m'; OFF=$'\033[0m'
else RED=; GREEN=; OFF=; fi

work=$(mktemp -d); trap 'rm -rf "$work"' EXIT
TAB=$'\t'

tools/coverage.sh --names | sort -u > "$work/names"

# The bindings are the only hand-maintained input, so they are the only one that
# can be malformed. A binding for a name upstream no longer declares is the
# re-vendor failure this catches: it would otherwise sit unreferenced forever.
awk -F'\t' -v OFS='\t' '
  FILENAME ~ /names$/ { known[$2] = 1; next }
  /^#/ || !NF { next }
  NF != 3 {
    printf "tools/bindings.tsv:%d: not SYMBOL<TAB>VERDICT<TAB>EVIDENCE\n", FNR > "/dev/stderr"
    bad = 1; next }
  $2 !~ /^(BOUND|EXTENSION|LANGUAGE|ZIG|INTERNAL)$/ {
    printf "tools/bindings.tsv:%d: %s is not a verdict a binding can carry\n", FNR, $2 > "/dev/stderr"
    bad = 1; next }
  !($1 in known) {
    printf "tools/bindings.tsv:%d: upstream declares no %s\n", FNR, $1 > "/dev/stderr"
    bad = 1; next }
  seen[$1]++ {
    printf "tools/bindings.tsv:%d: %s is bound twice\n", FNR, $1 > "/dev/stderr"
    bad = 1; next }
  { print }
  END { exit bad ? 1 : 0 }
' "$work/names" tools/bindings.tsv > "$work/bindings" || {
  printf '%stools/bindings.tsv is malformed%s\n' "$RED" "$OFF" >&2; exit 1; }

# A name with a binding takes it; a name without one is a gap, and its evidence
# is the tranche label the rules give it.
awk -f tools/tranches.awk "$work/names" > "$work/labelled"
awk -F'\t' -v OFS='\t' '
  FILENAME ~ /bindings$/ { verdict[$1] = $2; evidence[$1] = $3; next }
  $3 == "UNCLASSIFIED" {
    printf "%s: no rule in tools/tranches.awk claims it\n", $2 > "/dev/stderr"
    bad = 1; next }
  { print $1, $2, ($2 in verdict) ? verdict[$2] : "GAP",
              ($2 in verdict) ? evidence[$2] : $3 }
  END { exit bad ? 1 : 0 }
' "$work/bindings" "$work/labelled" > "$work/rows" || {
  printf '%stools/tranches.awk left a name unclassified%s\n' "$RED" "$OFF" >&2; exit 1; }

# One file per area, named after it, sorted by symbol so a diff of the record is
# a diff of what changed rather than of where things moved.
cut -f1 "$work/rows" | sort -u | while read -r area; do
  file="tools/unbound_$(printf '%s' "$area" | tr 'A-Z' 'a-z').txt"
  awk -F'\t' -v A="$area" '$1 == A' "$work/rows" | sort -t"$TAB" -k2,2 > "$work/out"
  if [ "$CHECK" -eq 1 ]; then
    if ! cmp -s "$work/out" "$file"; then
      printf '%s%s is not what tools/record.sh generates%s\n' "$RED" "$file" "$OFF" >&2
      diff "$file" "$work/out" | head -20 >&2 || true
      echo stale >> "$work/stale"
    fi
  else
    cp "$work/out" "$file"
  fi
done

if [ "$CHECK" -eq 1 ]; then
  if [ -s "$work/stale" ] 2>/dev/null; then
    printf '\nEdit tools/bindings.tsv or tools/tranches.awk and re-run tools/record.sh;\n' >&2
    printf 'the record itself is generated and hand edits to it are lost.\n' >&2
    exit 1
  fi
  printf '%sOK%s  the record matches its inputs\n' "$GREEN" "$OFF"
  exit 0
fi

awk -F'\t' '{ c[$3]++; n++ } END {
  printf "  %-30s %5d\n", "public Recast/Detour names", n
  for (v in c) printf "    %-28s %5d\n", tolower(v), c[v]
}' "$work/rows" | sort
