#!/usr/bin/env bash
#
# zrecast — every public name Recast and Detour declare, and where each one
# stands.
#
# The previous version of this script answered "how much of Recast is bound"
# with "7 of 421 names — 1%". Both halves were wrong. It matched an identifier
# followed by "(", so it saw functions and nothing else: dtNavMeshCreateParams'
# 31 must-populate fields, dtStatus's twelve `static const unsigned int` detail
# bits and the 25 constants in the six unprefixed enums were all invisible. It
# then decided a name was bound if some exported symbol *contained* it, so a
# name like `init` matched anything.
#
# What replaced it:
#
#   tools/upstream_names.awk  harvests functions, data members, enum constants,
#                             static constants and type names, keeps overloads
#                             apart by arity, and prints what it cannot parse.
#   tools/unbound_*.txt       gives every public name a verdict and evidence.
#   ci/check-coverage.sh      is the gate: no name without a verdict, no
#                             verdict without evidence that resolves.
#
# There is deliberately no "bound by name resemblance" heuristic left. A name
# counts as bound when a line in the record says so and the evidence checks
# out, and not before.
#
# Usage:
#   tools/coverage.sh                  # summary per area
#   tools/coverage.sh --names          # AREA<TAB>SYMBOL, every public name
#   tools/coverage.sh --areas          # the upstream directories claimed here
#   tools/coverage.sh --audit          # lines the harvester did not understand
#   tools/coverage.sh --collapsed      # symbols covering >1 declaration
#   tools/coverage.sh DetourCrowd      # the names in one area

set -uo pipefail
cd "$(dirname "$0")/.."

RECAST=libs/recastnavigation
MODE="${1:-}"

if [ -t 1 ]; then B=$'\033[1m'; D=$'\033[2m'; R=$'\033[31m'; G=$'\033[32m'; Y=$'\033[33m'; O=$'\033[0m'
else B=; D=; R=; G=; Y=; O=; fi

# Areas. DetourNavMeshQuery is pulled out of Detour rather than folded in:
# dtNavMeshQuery is the pathfinding engine, large enough to want its own line
# rather than being buried next to DetourAlloc.
#
# DebugUtils is claimed and deliberately empty: every one of its functions
# takes a duDebugDraw* renderer callback, which is a drawing interface rather
# than a navigation capability, and this ABI does not host one. It is named
# here so ci/check-coverage.sh's directory guard sees it as accounted for
# rather than as a directory nobody looked at.
areas() { printf '%s\n' Recast Detour DetourNavMeshQuery DetourCrowd DetourTileCache; }

area_files() {
  case "$1" in
    Recast)             printf '%s\n' "$RECAST"/Recast/Include/*.h ;;
    Detour)             for f in "$RECAST"/Detour/Include/*.h; do
                          [ "${f##*/}" = DetourNavMeshQuery.h ] || printf '%s\n' "$f"
                        done ;;
    DetourNavMeshQuery) printf '%s\n' "$RECAST"/Detour/Include/DetourNavMeshQuery.h ;;
    DetourCrowd)        printf '%s\n' "$RECAST"/DetourCrowd/Include/*.h ;;
    DetourTileCache)    printf '%s\n' "$RECAST"/DetourTileCache/Include/*.h ;;
  esac
}

# KIND:OWNER::NAME, with /ARITY on a function so overloads stay apart, and the
# parameter types after that when an arity is not enough — dtSwapEndian's five
# one-argument forms, rcRasterizeTriangles' two eight-argument ones, and every
# member that differs from its twin only in constness. The types are added only
# where they are needed, so the 1,000-odd names that never collided keep the
# spelling they already have.
symbols_of() {
  # shellcheck disable=SC2046
  awk -f tools/upstream_names.awk $(area_files "$1") 2>/dev/null |
    awk -F'\t' '$5 == "public" {
      s = $1 ":" $2 "::" $3
      if ($1 == "func") s = s "/" $4
      key[NR] = s; disc[NR] = $7
      if (!((s SUBSEP $7) in seen)) { seen[s SUBSEP $7] = 1; forms[s]++ }
      n = NR
    }
    END {
      for (i = 1; i <= n; i++)
        if (i in key) print key[i] (forms[key[i]] > 1 ? disc[i] : "")
    }' | sort -u
}

case "$MODE" in
  --areas)
    printf 'Recast\nDetour\nDetourCrowd\nDetourTileCache\nDebugUtils\n'
    exit 0 ;;

  --names)
    for a in $(areas); do symbols_of "$a" | sed "s|^|$a\t|"; done
    exit 0 ;;

  --audit)
    for a in $(areas); do
      # shellcheck disable=SC2046
      awk -v AUDIT=1 -f tools/upstream_names.awk $(area_files "$a") 2>/dev/null |
        grep '^SKIP' | cut -f6-
    done
    exit 0 ;;

  --collapsed)
    # Two declarations reduced to one symbol, after arity and parameter types
    # have both been applied. What is left is a name upstream really does
    # declare twice — a typedef with an #ifdef branch each way, or a member
    # declared virtual in one configuration and not in the other — so one
    # symbol is the right answer and this is the list that says so.
    for a in $(areas); do
      # shellcheck disable=SC2046
      awk -f tools/upstream_names.awk $(area_files "$a") 2>/dev/null |
        awk -F'\t' -v A="$a" '$5 == "public" {
          s = $1 ":" $2 "::" $3; if ($1 == "func") s = s "/" $4 $7
          n[s]++; where[s] = where[s] " " $6
        } END { for (s in n) if (n[s] > 1) printf "%s\t%s\t%d\t%s\n", A, s, n[s], where[s] }'
    done | sort
    exit 0 ;;
esac

#-----------------------------------------------------------------------------
# Summary. The only completeness number here is the one the record supports:
# names with a verdict, by verdict. A name nobody has judged is counted as
# unclassified and named as such, never folded into a percentage.
#-----------------------------------------------------------------------------
work=$(mktemp -d); trap 'rm -rf "$work"' EXIT
cat tools/unbound_*.txt 2>/dev/null | grep -v '^#' | grep . > "$work/record" || true

printf '%szrecast coverage of Recast and Detour%s\n\n' "$B" "$O"
printf '  %-22s %6s %6s %6s %6s %6s %6s\n' area names bound ext lang internal GAP

tb=0; tn=0; te=0; tl=0; ti=0; tg=0; tu=0
for a in $(areas); do
  symbols_of "$a" > "$work/names"
  n=$(grep -c . "$work/names")
  awk -F'\t' -v A="$a" '$1 == A { print $2 "\t" $3 }' "$work/record" | sort -u > "$work/verdicts"

  b=$(awk -F'\t' '$2 == "BOUND"' "$work/verdicts" | wc -l | tr -d ' ')
  e=$(awk -F'\t' '$2 == "EXTENSION"' "$work/verdicts" | wc -l | tr -d ' ')
  l=$(awk -F'\t' '$2 == "LANGUAGE" || $2 == "ZIG"' "$work/verdicts" | wc -l | tr -d ' ')
  i=$(awk -F'\t' '$2 == "INTERNAL"' "$work/verdicts" | wc -l | tr -d ' ')
  g=$(awk -F'\t' '$2 == "GAP"' "$work/verdicts" | wc -l | tr -d ' ')
  u=$(( n - b - e - l - i - g ))

  colour=$Y; [ "$g" -eq 0 ] && [ "$u" -eq 0 ] && colour=$G
  printf '  %-22s %6d %6d %6d %6d %6d %s%6d%s\n' "$a" "$n" "$b" "$e" "$l" "$i" "$colour" "$g" "$O"
  tn=$((tn+n)); tb=$((tb+b)); te=$((te+e)); tl=$((tl+l)); ti=$((ti+i)); tg=$((tg+g)); tu=$((tu+u))

  if [ -n "$MODE" ] && printf '%s' "$a" | grep -qi "$MODE"; then
    join -t"$(printf '\t')" -a1 -e GAP -o '0,2.2' \
      <(sort "$work/names") <(sort -t"$(printf '\t')" -k1,1 "$work/verdicts") 2>/dev/null |
      sed 's/^/      /'
  fi
done

printf '  %-22s %6d %6d %6d %6d %6d %6d\n' TOTAL "$tn" "$tb" "$te" "$tl" "$ti" "$tg"
if [ "$tu" -gt 0 ]; then
  printf '\n  %s%d name(s) with no verdict at all — run ci/check-coverage.sh%s\n' "$R" "$tu" "$O"
fi

#-----------------------------------------------------------------------------
# What this measurement does not see. Stated every run, because the last
# version of this script was believed to be complete for months.
#-----------------------------------------------------------------------------
collapsed=$("$0" --collapsed | wc -l | tr -d ' ')
audit=$("$0" --audit | wc -l | tr -d ' ')
printf '\n  %sblind spots%s\n' "$B" "$O"
printf '    %spreprocessor macros are not names here (rcLikely, the assert macros)%s\n' "$D" "$O"
printf '    %soperator overloads carry no name a C ABI can spell%s\n' "$D" "$O"
printf '    %s%s symbol(s) cover more than one declaration — tools/coverage.sh --collapsed%s\n' "$D" "$collapsed" "$O"
printf '    %s%s line(s) the harvester did not parse — tools/coverage.sh --audit%s\n' "$D" "$audit" "$O"
printf '    %sDebugUtils is out of scope: every entry point there takes a renderer callback%s\n' "$D" "$O"
