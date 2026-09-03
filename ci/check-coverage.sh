#!/usr/bin/env bash
#
# zrecast — every public Recast/Detour name has a verdict, and every verdict is
# checkable.
#
#   tools/coverage.sh --names   every public name upstream declares, by area.
#   tools/bindings.tsv          the names that are answered, and how.
#   tools/tranches.awk          which tranche closes each of the rest.
#   tools/unbound_*.txt         the record those three generate, checked in so
#                               a diff of it is reviewable.
#   tools/classify.sh           what upstream itself justifies excluding.
#
# INTERNAL is not taken on trust: it is recomputed here and rejected unless
# classify.sh proves it. GAP fails the build — which it will, loudly, until the
# binding is complete. That is the true state of the package and the script
# will not pretend otherwise.
#
# A GAP still has to say which tranche closes it. A gap nobody has scheduled is
# how a work list turns back into a wish, and the record is the work list.
#
# Usage:
#   ci/check-coverage.sh          # the gate
#   ci/check-coverage.sh --list   # and print the open gaps

set -uo pipefail
cd "$(dirname "$0")/.."

LIST=0
[ "${1:-}" = "--list" ] && LIST=1

if [ -t 1 ]; then RED=$'\033[31m'; GREEN=$'\033[32m'; DIM=$'\033[2m'; BOLD=$'\033[1m'; OFF=$'\033[0m'
else RED=; GREEN=; DIM=; BOLD=; OFF=; fi

work=$(mktemp -d); trap 'rm -rf "$work"' EXIT
fails=0
fail() { printf '%s%s%s\n' "$RED" "$1" "$OFF" >&2; fails=$((fails + 1)); }

TAB=$'\t'

# Every identifier ffi/*.h declares or mentions — types and enum constants as
# well as functions.
grep -hoE '\b(zrc|Zrc|ZRC_)[A-Za-z0-9_]*' ffi/*.h | sort -u > "$work/syms"

tools/coverage.sh --names | sort -u > "$work/names"
tools/classify.sh | sort -t"$TAB" -k1,1 > "$work/provable"

#-----------------------------------------------------------------------------
# Claimed directories. A directory of upstream that no area covers is a place a
# capability can hide where nothing here would ever look, so a re-vendor that
# adds one fails until tools/coverage.sh catches up.
#-----------------------------------------------------------------------------
find libs/recastnavigation -mindepth 1 -maxdepth 1 -type d |
  sed 's|libs/recastnavigation/||' | sort -u > "$work/dirs"
tools/coverage.sh --areas | sort -u > "$work/claimed"
comm -23 "$work/dirs" "$work/claimed" > "$work/unclaimed"
if [ -s "$work/unclaimed" ]; then
  sed 's/^/  /' "$work/unclaimed" >&2
  fail "$(grep -c . "$work/unclaimed") upstream directory(ies) no area claims"
fi

#-----------------------------------------------------------------------------
# Shape.
#-----------------------------------------------------------------------------
: > "$work/rows"; : > "$work/shape"
for f in tools/unbound_*.txt; do
  [ -e "$f" ] || continue
  awk -F'\t' -v F="$f" '
    /^#/ || !NF { next }
    NF != 4 { printf "%s:%d: not four tab-separated fields\n", F, FNR > "/dev/stderr"; next }
    $3 !~ /^(BOUND|EXTENSION|LANGUAGE|ZIG|INTERNAL|GAP)$/ {
      printf "%s:%d: unknown verdict %s\n", F, FNR, $3 > "/dev/stderr"; next }
    $3 != "GAP" && length($4) < 8 {
      printf "%s:%d: %s has no evidence\n", F, FNR, $2 > "/dev/stderr"; next }
    $3 == "GAP" && $4 !~ /^tranche [1-8][ab]? - .../ {
      printf "%s:%d: %s is a GAP not assigned to a tranche\n", F, FNR, $2 > "/dev/stderr"; next }
    { print $1 "\t" $2 "\t" $3 "\t" $4 }
  ' "$f" 2>>"$work/shape" >> "$work/rows"
done
if [ -s "$work/shape" ]; then
  cat "$work/shape" >&2
  fail "$(grep -c . "$work/shape") malformed line(s)"
fi

#-----------------------------------------------------------------------------
# The record is generated, so a hand edit to it is a lie waiting to be read.
# tools/record.sh writes it from the harvest, the bindings and the rules; this
# is where a divergence between the three and what is checked in fails.
#-----------------------------------------------------------------------------
if ! tools/record.sh --check > "$work/record" 2>&1; then
  cat "$work/record" >&2
  fail 'tools/unbound_*.txt is not what tools/record.sh generates'
fi

#-----------------------------------------------------------------------------
# Completeness, both directions.
#-----------------------------------------------------------------------------
cut -f1,2 "$work/rows" | sort -u > "$work/classified"
comm -23 "$work/names" "$work/classified" > "$work/missing"
if [ -s "$work/missing" ]; then
  head -40 "$work/missing" | sed 's/^/  /' >&2
  fail "$(grep -c . "$work/missing") area/name pair(s) with no verdict"
fi
comm -13 "$work/names" "$work/classified" > "$work/stale"
if [ -s "$work/stale" ]; then
  head -40 "$work/stale" | sed 's/^/  /' >&2
  fail "$(grep -c . "$work/stale") stale line(s) — upstream no longer declares these; delete them"
fi
if [ "$(cut -f1,2 "$work/rows" | sort | uniq -d | grep -c .)" -gt 0 ]; then
  cut -f1,2 "$work/rows" | sort | uniq -d | sed 's/^/  /' >&2
  fail 'duplicate area/name pair(s) in the record'
fi

#-----------------------------------------------------------------------------
# Evidence, one rule per verdict.
#-----------------------------------------------------------------------------
cat > "$work/facilities" <<'EOF'
@Vector
@shuffle
@splat
@bitCast
@ptrCast
@floatCast
@intCast
@enumFromInt
@intFromEnum
@as
@min
@max
@abs
@sqrt
@floor
@ceil
@cos
@sin
@memcpy
@memset
@byteSwap
@fieldParentPtr
std.math
std.mem
std.sort
std.ArrayList
std.PriorityQueue
std.heap
std.fmt
std.Thread
std.testing
EOF

awk -F'\t' '
  FILENAME ~ /syms$/       { sym[$0] = 1; next }
  FILENAME ~ /facilities$/ { fac[++nf] = $0; next }
  FILENAME ~ /provable$/   { prov[$1] = $3; next }
  { name = $2; verdict = $3; evidence = $4 }

  # BOUND and EXTENSION must name a zrecast symbol that ffi/*.h really declares.
  # A struct member is written Type.member and both halves are checked, the
  # member below.
  verdict == "BOUND" || verdict == "EXTENSION" {
    n = 0; s = evidence
    while (match(s, /(zrc|Zrc|ZRC_)[A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)?/)) {
      t = substr(s, RSTART, RLENGTH); s = substr(s, RSTART + RLENGTH)
      n++
      if (index(t, ".") > 0) { print "MEMBER\t" name "\t" t; continue }
      if (!(t in sym)) printf "  %s: %s is not in ffi/*.h\n", name, t > "/dev/stderr"
    }
    if (n == 0) printf "  %s: %s names no zrecast symbol\n", name, verdict > "/dev/stderr"
    next
  }

  # LANGUAGE names a Zig facility from a closed list, so it cannot become a
  # dumping ground for anything inconvenient.
  verdict == "LANGUAGE" {
    hit = 0; s = evidence
    while (match(s, /(@[A-Za-z]+|std\.[A-Za-z.]+)/)) {
      t = substr(s, RSTART, RLENGTH); s = substr(s, RSTART + RLENGTH)
      ok = 0
      for (i = 1; i <= nf; i++) if (index(t, fac[i]) == 1) { ok = 1; break }
      if (!ok) printf "  %s: %s is not a facility this file recognises\n", name, t > "/dev/stderr"
      else hit = 1
    }
    if (!hit && evidence !~ /Zig slices|defer|errdefer|discard/)
      printf "  %s: LANGUAGE names no facility\n", name > "/dev/stderr"
    next
  }

  # Only the leading src/FILE.zig:decl is the reference; whatever follows it is
  # prose, the same way BOUND evidence is prose with symbols in it. Passing the
  # whole line on would make the lookup below search for a sentence.
  verdict == "ZIG" {
    if (match(evidence, /^src\/([A-Za-z0-9_]+\/)?[A-Za-z0-9_]+\.zig:[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)?/))
      print "ZIG\t" name "\t" substr(evidence, RSTART, RLENGTH)
    else
      printf "  %s: ZIG evidence is not src/FILE.zig:decl\n", name > "/dev/stderr"
    next
  }

  verdict == "INTERNAL" {
    if (!(name in prov))
      printf "  %s: INTERNAL, but tools/classify.sh cannot justify it\n", name > "/dev/stderr"
    else if (evidence != prov[name])
      printf "  %s: INTERNAL evidence does not match what classify.sh computes\n", name > "/dev/stderr"
    next
  }
' "$work/syms" "$work/facilities" "$work/provable" "$work/rows" \
  2>"$work/evidence" > "$work/refs"
if [ -s "$work/evidence" ]; then
  head -40 "$work/evidence" >&2
  fail "$(grep -c . "$work/evidence") unusable piece(s) of evidence"
fi

# Type.member evidence: the member has to sit inside that type's own braces.
: > "$work/refmiss"
grep '^MEMBER' "$work/refs" 2>/dev/null | while IFS="$TAB" read -r _ name ref; do
  type=${ref%%.*}; member=${ref#*.}
  awk -v T="$type" -v M="$member" '
    $0 ~ ("(typedef (struct|enum|union) " T "|^} " T ";)") { inb = !inb ? 1 : inb }
    inb && $0 ~ ("(^|[^A-Za-z0-9_])" M "([^A-Za-z0-9_]|$)") { found = 1 }
    $0 ~ ("^} " T ";") { inb = 0 }
    END { exit found ? 0 : 1 }' ffi/*.h ||
    printf '  %s: ffi/*.h declares no %s inside %s\n' "$name" "$member" "$type" >> "$work/refmiss"
done
if [ -s "$work/refmiss" ]; then
  cat "$work/refmiss" >&2
  fail "$(grep -c . "$work/refmiss") member reference(s) pointing at nothing"
fi

# ZIG evidence: the declaration has to exist.
: > "$work/zigmiss"
grep '^ZIG' "$work/refs" 2>/dev/null | while IFS="$TAB" read -r _ name ref; do
  file=${ref%%:*}; decl=${ref#*:}
  [ -f "$file" ] || { printf '  %s: %s does not exist\n' "$name" "$file" >> "$work/zigmiss"; continue; }
  case "$decl" in
    *.*)
      type=${decl%%.*}; member=${decl#*.}
      awk -v T="$type" -v M="$member" '
        $0 ~ ("^pub const " T " = (extern |packed )?(struct|union|enum)") { inb = 1; depth = 0 }
        inb {
          depth += gsub(/{/, "{") - gsub(/}/, "}")
          if ($0 ~ ("^[[:space:]]*(pub )?(fn|const|inline fn|var) " M "[ (:=]")) found = 1
          if (depth <= 0 && seen) inb = 0
          seen = 1
        }
        END { exit found ? 0 : 1 }' "$file" ||
        printf '  %s: %s declares no %s inside %s\n' "$name" "$file" "$member" "$type" >> "$work/zigmiss"
      ;;
    *)
      grep -qE "^[[:space:]]*pub (fn|const|inline fn|var) $decl\b" "$file" ||
        printf '  %s: %s declares no %s\n' "$name" "$file" "$decl" >> "$work/zigmiss"
      ;;
  esac
done
if [ -s "$work/zigmiss" ]; then
  cat "$work/zigmiss" >&2
  fail "$(grep -c . "$work/zigmiss") ZIG line(s) pointing at nothing"
fi

#-----------------------------------------------------------------------------
# The Zig surface. An entry point declared in C and never wrapped is
# unreachable for a Zig host, and nothing above can see it. src/c.zig is the
# extern layer, and a declaration is not use; tests do not count either.
#-----------------------------------------------------------------------------
grep -ho 'zrc[A-Za-z0-9_]*(' ffi/*.h | grep -o 'zrc[A-Za-z0-9_]*' | sort -u > "$work/entrypoints"
# /dev/null is passed as an extra file so grep never falls back to reading
# stdin when find matches nothing, which is a hang rather than an error.
find src -name '*.zig' ! -name 'c.zig' ! -name '*_test.zig' -print0 |
  xargs -0 grep -ho 'zrc[A-Za-z0-9_]*' /dev/null 2>/dev/null | sort -u > "$work/wrapped"
: > "$work/exc_shape"
if [ -f tools/zig_surface_exceptions.txt ]; then
  awk -F'\t' '/^#/ || !NF { next }
    NF != 2 { printf "  %s: not NAME<TAB>reason\n", $1 > "/dev/stderr"; next }
    length($2) < 10 { printf "  %s: no reason given\n", $1 > "/dev/stderr"; next }
    { print $1 }' tools/zig_surface_exceptions.txt 2>"$work/exc_shape" | sort -u > "$work/excused"
else
  : > "$work/excused"
fi
if [ -s "$work/exc_shape" ]; then
  cat "$work/exc_shape" >&2
  fail "$(grep -c . "$work/exc_shape") malformed exception line(s)"
fi

comm -23 "$work/entrypoints" "$work/wrapped" > "$work/unwrapped"
comm -23 "$work/unwrapped" "$work/excused" > "$work/stranded"
if [ -s "$work/stranded" ]; then
  sed 's/^/  /' "$work/stranded" >&2
  fail "$(grep -c . "$work/stranded") entry point(s) with no Zig caller"
fi
comm -13 "$work/unwrapped" "$work/excused" > "$work/excess"
if [ -s "$work/excess" ]; then
  sed 's/^/  /' "$work/excess" >&2
  fail "$(grep -c . "$work/excess") excused entry point(s) that Zig does call, or that no longer exist"
fi

#-----------------------------------------------------------------------------
# Summary.
#-----------------------------------------------------------------------------
awk -F'\t' '$3 == "GAP"' "$work/rows" > "$work/open"
if [ "$LIST" -eq 1 ] && [ -s "$work/open" ]; then
  printf '%sgaps%s\n' "$BOLD" "$OFF"
  awk -F'\t' '{ printf "  %s\t%s\n", $1, $2 }' "$work/open" | sort >&2
  printf '\n'
fi

printf '%szrecast coverage%s\n' "$BOLD" "$OFF"
printf '  %-30s %5d\n' 'entry points exported' "$(grep -c . "$work/entrypoints")"
printf '  %-30s %5d\n' 'public Recast/Detour names' "$(grep -c . "$work/names")"
awk -F'\t' '{ c[$3]++ } END { for (v in c) printf "    %-28s %5d\n", tolower(v), c[v] }' "$work/rows" | sort

if [ -s "$work/open" ]; then
  fail "$(grep -c . "$work/open") gap(s) — run with --list to see them"
fi

#-----------------------------------------------------------------------------
# README's headline numbers, which live in the generated "By the numbers"
# table. ci/measurements.sh computes them from the ledger and check-docs keeps
# the table current; this recomputes both from the headers and the harvest, so
# the two derivations have to agree or one of the scripts is wrong.
#-----------------------------------------------------------------------------
entry_point_decls=$(grep -c '^ZRC_API' ffi/*.h | awk -F: '{ n += $2 } END { print n }')
public_names=$(grep -c . "$work/names")
readme_entry_points=$(grep -oE '\| \*\*[0-9]+\*\* \| C entry points' README.md |
                      grep -oE '[0-9]+' | sort -u)
readme_names=$(grep -oE '\| \*\*[0-9]+\*\* \| public Recast/Detour names' README.md |
               grep -oE '[0-9]+' | sort -u)
if [ "$readme_entry_points" != "$entry_point_decls" ] || [ "$readme_names" != "$public_names" ]; then
  printf '  README says %s entry points and %s public names; the tree has %s and %s\n' \
    "${readme_entry_points:-none}" "${readme_names:-none}" \
    "$entry_point_decls" "$public_names" >&2
  fail 'README headline numbers are stale'
fi

if [ "$fails" -ne 0 ]; then
  printf '\n%sFAIL%s  %d problem(s)\n' "$RED" "$OFF" "$fails" >&2
  exit 1
fi
printf '\n%sOK%s  every public Recast/Detour name is accounted for\n' "$GREEN" "$OFF"
