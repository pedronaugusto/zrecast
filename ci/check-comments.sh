#!/usr/bin/env bash
#
# Refuses comments that have stopped being documentation.
#
# Two rules, and they are not the same rule. The register check is the
# substantive one: a comment that talks about the people who wrote it
# ("we", "it used to", "which is why") is a diary entry, and a package whose
# history is as public as its README does not want one.
#
# The length cap is a backstop, not the standard, and it is set per kind of
# comment rather than as one number. A `///` block above a declaration in the
# public header IS the reference documentation for that entry point, and
# zrcNavMeshValidate's contract needs every line it has. A `//` block inside an
# implementation is commentary, but the ones here name upstream defects by
# file and line and are the reason a reader does not have to rediscover them.
# The numbers are therefore set from what documentation of that kind actually
# needs in this package; they catch a comment that has turned into an essay,
# not a thorough one. Sibling packages use one tighter number because their
# headers are thinner, and the difference is deliberate.
#
#   ci/check-comments.sh

set -uo pipefail
cd "$(dirname "$0")/.."

if [ -t 1 ]; then RED=$'\033[31m'; GREEN=$'\033[32m'; OFF=$'\033[0m'
else RED=; GREEN=; OFF=; fi

MAX_DOC=40      # /// reference documentation above one declaration
MAX_CODE=30     # // commentary inside an implementation
MAX_BANNER=42   # a file header, or a //===---===// section banner
work=$(mktemp -d); trap 'rm -rf "$work"' EXIT

for f in ffi/*.h ffi/*.cpp src/*.zig tests/*.c tests/*.h; do
  [ -f "$f" ] || continue
  awk -v F="$f" -v MDOC="$MAX_DOC" -v MCODE="$MAX_CODE" -v MBAN="$MAX_BANNER" '
    BEGIN { run = 0; start = 0; first = 1 }
    /^[[:space:]]*(\/\/|\/\/\/|\/\/!)/ {
      if (run == 0) { start = NR; banner = 0; doc = ($0 ~ /^[[:space:]]*(\/\/\/|\/\/!)/) }
      if ($0 ~ /^\/\/===/) banner = 1
      run++; next
    }
    {
      if (run > 0) {
        cap = ((first && start <= 3) || banner) ? MBAN : (doc ? MDOC : MCODE)
        if (run > cap) printf "%s:%d: %d comment lines in one block (max %d)\n", F, start, run, cap
        first = 0; run = 0
      }
    }
    END { if (run > MCODE) printf "%s:%d: %d trailing comment lines\n", F, start, run }
  ' "$f" >> "$work/long"

  # Narrative register. These read as a person talking, not as documentation.
  grep -nEi '^[[:space:]]*(//|///|//!).*(\bwe\b|\bour\b|\bus\b|\bnote that\b|\bworth (stating|noting|saying)\b|\bused to be\b|\bthey used to\b|\bit used to\b|\bthe reason (is|it)\b|\bwhich is why\b|\bthat is why\b|\bturns out\b|\bin practice this\b|\bdo not be\b|\byou might (think|expect)\b|\bit is tempting\b)' "$f" |
    sed "s|^|$f:|" >> "$work/voice"
done

fails=0
if [ -s "$work/long" ]; then
  sort -t: -k1,1 "$work/long" | head -40 | sed 's/^/  /' >&2
  printf '%s%d over-long comment block(s)%s\n' "$RED" "$(grep -c . "$work/long")" "$OFF" >&2
  fails=$((fails + 1))
fi
if [ -s "$work/voice" ]; then
  head -30 "$work/voice" | sed 's/^/  /' >&2
  printf '%s%d narrative comment line(s)%s\n' "$RED" "$(grep -c . "$work/voice")" "$OFF" >&2
  fails=$((fails + 1))
fi

[ "$fails" -ne 0 ] && exit 1
printf '%sOK%s  comments stay documentation\n' "$GREEN" "$OFF"
