# Names Recast and Detour mark as internal, with the marker's FILE:LINE.
#
# Recast has one such phrase, "generally meant for internal use only", and it
# appears in two positions:
#
#   group  a doxygen @name banner — DetourNavMesh.h:522 opens "Encoding and
#          Decoding" with it, and it holds until the closing "/// @}".
#   decl   an @note on the comment immediately above one declaration.
#
# Nothing else in the vendored headers claims to be internal, so this is the
# whole of what upstream justifies excluding. A name that is merely awkward to
# bind is not internal, and gets a GAP until it is bound.
#
# Emits: NAME<TAB>FILE:LINE<TAB>group|decl

FNR == 1 { group = 0; pending = 0; incomment = 0 }

/generally meant for internal use only/ {
  if ($0 ~ /These functions are/) { group = 1; mark = FILENAME ":" FNR }
  else                           { pending = 1; pendmark = FILENAME ":" FNR }
  next
}
group && /^[[:space:]]*\/\/\/[[:space:]]*@}/ { group = 0; next }
/^[[:space:]]*\/\/\// { next }

{
  line = $0
  if (incomment) { p = index(line, "*/"); if (p == 0) next
                   line = substr(line, p + 2); incomment = 0 }
  while ((a = index(line, "/*")) > 0) {
    rest = substr(line, a + 2); b = index(rest, "*/")
    if (b == 0) { line = substr(line, 1, a - 1); incomment = 1; break }
    line = substr(line, 1, a - 1) substr(rest, b + 2)
  }
  sub(/\/\/.*/, "", line)
  $0 = line
}

/^[[:space:]]*#/ { next }
!NF { next }

match($0, /[A-Za-z_][A-Za-z0-9_]*[[:space:]]*\(/) {
  pre = (RSTART > 1) ? substr($0, RSTART - 1, 1) : ""
  if (pre == "." || pre == ">" || pre == ":") { pending = 0; next }
  n = substr($0, RSTART, RLENGTH - 1); gsub(/[[:space:]()]/, "", n)
  if (n ~ /^(if|for|while|switch|return|sizeof|do|else|operator|new|delete)$/) { pending = 0; next }
  if (pending)    print n "\t" pendmark "\tdecl"
  else if (group) print n "\t" mark "\tgroup"
  pending = 0
}
