# Every name Recast and Detour declare in a header, by kind.
#
# The old measurement answered "how much of the library is bound" by looking
# for an identifier followed by "(", which is a function and nothing else.
# Recast's surface is not mostly functions: dtNavMeshCreateParams has 31 data
# members that must be populated before a navmesh can exist, dtStatus's twelve
# detail bits are `static const unsigned int` rather than an enum, and six
# enums carry no rc/dt prefix at all. None of that was counted, so "N of 421
# names bound" was a true sentence about the wrong set.
#
# Emits one line per declaration, tab-separated:
#
#   KIND  OWNER  NAME  SIG  ACCESS  FILE:LINE
#
#   KIND    func | field | enum | const | type
#   OWNER   the enclosing class, struct or enum, or "-" at file scope
#   SIG     for a function, its parameter count; "-" otherwise. Overloads are
#           separate capabilities — dtNavMesh::init takes either a single tile
#           or a tile grid — and collapsing them hides one of the two.
#   ACCESS  public | nonpublic. Every enclosing scope must be public: a public
#           member of a privately nested type is unreachable anyway.
#
# Physical lines are joined while parentheses are unbalanced, so a declaration
# spanning four lines is read once, in full, and its continuation lines are
# never mistaken for declarations of their own.
#
# Run with -v AUDIT=1 to print every harvestable line that produced no name.
# That list is the honest statement of what this does not understand, and it
# should contain nothing but braces, operators, and out-of-line definitions of
# members already declared in their class.
#
# What it still cannot see, stated so no one mistakes silence for coverage:
#
#   - preprocessor macros (rcLikely, RC_SIZE_MAX, the assert macros)
#   - operator overloads, which have no name a C ABI can carry
#   - const/non-const overload pairs, which collapse to one row because they
#     are one capability across a C boundary
#   - anything declared only in a .cpp, which no host can reach either
#
# Written for POSIX awk: the CI runners are Linux and the development host is
# macOS, and neither gets a dialect the other lacks.

function scope_is_public(   i) {
  for (i = 1; i <= top; i++)
    if (skind[i] == "class" && !sacc[i]) return 0
  return 1
}

function last_ident(s,   n) {
  n = ""
  while (match(s, /[A-Za-z_][A-Za-z0-9_]*/)) {
    n = substr(s, RSTART, RLENGTH); s = substr(s, RSTART + RLENGTH)
  }
  return n
}

function first_ident(s) {
  if (match(s, /[A-Za-z_][A-Za-z0-9_]*/)) return substr(s, RSTART, RLENGTH)
  return ""
}

function is_keyword(n) {
  return n ~ /^(if|for|while|switch|return|sizeof|alignof|static_assert|do|else|catch|new|delete|defined|throw|goto|case|break|continue|template|typename|operator)$/
}

# A type keyword at the head of a declaration is not the thing being declared.
function is_typeword(n) {
  return n ~ /^(void|bool|char|short|int|long|float|double|signed|unsigned|const|volatile|static|inline|virtual|explicit|extern|struct|class|enum|union|typedef|using|friend|public|private|protected|size_t|auto|register|mutable)$/
}

# Parameters between the first balanced pair of parentheses. Commas inside
# nested parentheses and inside template arguments belong to a parameter, not
# between two, so both nestings are tracked.
function arity(s,   i, c, pd, ad, n, seen) {
  i = index(s, "(")
  if (i == 0) return "-"
  pd = 0; ad = 0; n = 0; seen = 0
  for (; i <= length(s); i++) {
    c = substr(s, i, 1)
    if (c == "(") { pd++; continue }
    if (c == ")") { pd--; if (pd == 0) break; continue }
    if (c == "<") { ad++; continue }
    if (c == ">") { if (ad > 0) ad--; continue }
    if (c == "," && pd == 1 && ad == 0) n++
    if (c ~ /[A-Za-z0-9_]/ && pd == 1) seen = 1
  }
  if (!seen) return 0
  return n + 1
}

# One parameter, reduced to its type: default argument dropped, array extent
# emptied, parameter name removed, and whitespace around * and & closed up. The
# name is the last identifier when something precedes it and it is not itself a
# type word, which keeps `unsigned char` whole while stripping the `v` from
# `float* v`.
function paramtype(p,   n) {
  gsub(/^[[:space:]]+|[[:space:]]+$/, "", p)
  sub(/=.*$/, "", p)
  gsub(/\[[^]]*\]/, "[]", p)
  gsub(/[[:space:]]+/, " ", p)
  gsub(/^[[:space:]]+|[[:space:]]+$/, "", p)
  if (match(p, /[A-Za-z_][A-Za-z0-9_]*$/)) {
    n = substr(p, RSTART)
    if (!is_typeword(n) && RSTART > 1) p = substr(p, 1, RSTART - 1)
  }
  gsub(/[[:space:]]*\*[[:space:]]*/, "*", p)
  gsub(/[[:space:]]*&[[:space:]]*/, "\\&", p)
  gsub(/^[[:space:]]+|[[:space:]]+$/, "", p)
  return p
}

# The parameter types of the first balanced parenthesis group, as
# "(type,type)" with "const" appended for a const member function.
#
# An arity alone cannot separate every overload: dtSwapEndian has five forms
# that all take one argument, rcRasterizeTriangles has two that take eight and
# differ in their fourth parameter, and eleven member functions differ only in
# constness. Each of those folded into a single row, which is a declaration
# nobody would ever be asked to account for. This is what tells them apart, and
# it is stable across upstream reformatting in a way an ordinal is not.
function paramtypes(s,   i, c, pd, ad, cur, out, tail) {
  i = index(s, "(")
  if (i == 0) return ""
  pd = 0; ad = 0; cur = ""; out = ""
  for (; i <= length(s); i++) {
    c = substr(s, i, 1)
    if (c == "(") { pd++; if (pd == 1) continue }
    else if (c == ")") { pd--; if (pd == 0) break }
    else if (c == "<") ad++
    else if (c == ">") { if (ad > 0) ad-- }
    else if (c == "," && pd == 1 && ad == 0) {
      out = out (out == "" ? "" : ",") paramtype(cur); cur = ""; continue
    }
    cur = cur c
  }
  if (cur ~ /[A-Za-z0-9_]/) out = out (out == "" ? "" : ",") paramtype(cur)
  tail = substr(s, i + 1)
  return "(" out ")" (tail ~ /^[[:space:]]*const([^A-Za-z0-9_]|$)/ ? "const" : "")
}

function emit(kind, name, sig, disc,   owner) {
  if (name == "" || is_keyword(name) || is_typeword(name)) return
  emitted = 1
  owner = (top > 0 && skind[top] != "block") ? sname[top] : "-"
  print kind "\t" owner "\t" name "\t" sig "\t" \
        (scope_is_public() ? "public" : "nonpublic") "\t" FILENAME ":" startfnr \
        "\t" disc
}

# Enumerators and multi-declarator members both arrive as a comma-separated
# list, and taking one match per line is how the old tool lost `float cs, ch;`.
function emit_list(kind, decl,   n, i, parts, p, nm) {
  gsub(/\[[^]]*\]/, "", decl)          # array bounds are not names
  n = split(decl, parts, ",")
  for (i = 1; i <= n; i++) {
    p = parts[i]
    sub(/=.*$/, "", p)                 # initialiser
    sub(/:.*$/, "", p)                 # bitfield width
    nm = (kind == "enum") ? first_ident(p) : last_ident(p)
    emit(kind, nm, "-")
  }
}

# Comment removal is stateful across lines, so it is a function: joined
# continuation lines have to run through the same machine.
function strip(l,   p, a, b, rest) {
  if (incomment) {
    p = index(l, "*/")
    if (p == 0) return ""
    l = substr(l, p + 2); incomment = 0
  }
  while ((a = index(l, "/*")) > 0) {
    rest = substr(l, a + 2); b = index(rest, "*/")
    if (b == 0) { l = substr(l, 1, a - 1); incomment = 1; break }
    l = substr(l, 1, a - 1) substr(rest, b + 2)
  }
  sub(/\/\/.*/, "", l)
  # String and character literals can carry braces and parentheses;
  # DT_NAVMESH_MAGIC is spelled 'D'<<24 | 'N'<<16 | ... and must not unbalance
  # the depth count.
  gsub(/"[^"]*"/, "0", l)
  gsub(/'[^']*'/, "0", l)
  return l
}

function unbalanced(s,   i, c, pd) {
  pd = 0
  for (i = 1; i <= length(s); i++) {
    c = substr(s, i, 1)
    if (c == "(") pd++
    else if (c == ")") pd--
  }
  return pd > 0
}

FNR == 1 {
  depth = 0; top = 0
  incomment = 0; incpp = 0; pendkind = ""; pendname = ""; pendacc = 1
}

{
  line = strip($0)

  # A preprocessor directive, and every line its trailing backslash continues.
  # rcAssert's body is a braced block, and letting it into the depth count
  # would move every declaration after it into a scope that does not exist.
  if (incpp) { if (line !~ /\\[[:space:]]*$/) incpp = 0; next }
  if (line ~ /^[[:space:]]*#/) { if (line ~ /\\[[:space:]]*$/) incpp = 1; next }

  sub(/^[[:space:]]*template[[:space:]]*<[^>]*>[[:space:]]*/, "", line)
  if (line ~ /^[[:space:]]*$/) next

  startfnr = FNR
  while (unbalanced(line) && (getline nextline) > 0) {
    nextline = strip(nextline)
    if (nextline ~ /^[[:space:]]*#/) continue
    line = line " " nextline
  }

  kind = (top > 0) ? skind[top] : "file"
  harvest = 1

  if (line ~ /^[[:space:]]*(public|private|protected)[[:space:]]*:/) {
    if (top > 0 && skind[top] == "class")
      sacc[top] = (line ~ /^[[:space:]]*public[[:space:]]*:/) ? 1 : 0
    harvest = 0
  }

  if (harvest && (kind == "file" || kind == "class" || kind == "enum")) {
    emitted = 0
    harvest_line(line, kind)
    # Rule one of this work: the tool says what it did not understand. Every
    # harvestable line that yielded no name is printed under AUDIT, so a
    # declaration form nobody anticipated shows up as a line to read rather
    # than as silence.
    if (AUDIT && !emitted) print "SKIP\t-\t-\t-\t-\t" FILENAME ":" startfnr "\t" line
  }

  update_depth(line)
}

function harvest_line(l, kind,   tok, nm, kw, decl, rest, pre, brace) {

  # typedef — a name a host writes even though it declares no storage. The
  # name of a function typedef sits inside the first parenthesised group
  # (`typedef void (dtFreeFunc)(void* ptr);`), not at the end of the line,
  # where the last identifier is a parameter.
  if (l ~ /^[[:space:]]*typedef[[:space:]]/) {
    decl = l; sub(/;.*$/, "", decl); gsub(/\[[^]]*\]/, "", decl)
    if (match(decl, /\([^()]*\)/))
      emit("type", last_ident(substr(decl, RSTART + 1, RLENGTH - 2)), "-")
    else
      emit("type", last_ident(decl), "-")
    return
  }

  # class / struct / union. The keyword also introduces an elaborated type
  # specifier in an ordinary declaration — `struct dtTileCacheAlloc* getAlloc()`
  # returns one — so what follows the name decides. A body, a base clause or
  # end of line is a definition; a `;` is a forward declaration, whose type is
  # emitted where it is defined; anything else is a use, and falls through to
  # the function and field rules below.
  if (match(l, /(^|[^A-Za-z0-9_])(class|struct|union)[[:space:]]+[A-Za-z_][A-Za-z0-9_]*/)) {
    tok  = substr(l, RSTART, RLENGTH)
    rest = substr(l, RSTART + RLENGTH); sub(/^[[:space:]]*/, "", rest)
    if (rest == "" || rest ~ /^[{:]/ || rest ~ /^final([[:space:]{:]|$)/) {
      kw = (tok ~ /(^|[^A-Za-z0-9_])class[[:space:]]/) ? "class" : "struct"
      nm = last_ident(tok)
      emit("type", nm, "-")
      pendkind = "class"; pendname = nm; pendacc = (kw == "class") ? 0 : 1
      return
    }
    if (rest ~ /^;/) return
  }

  # An anonymous aggregate declares its members in the enclosing type, so its
  # scope carries the enclosing owner and access rather than starting a new
  # one. dtTileCacheObstacle's union of cylinder/box/orientedBox is the case.
  if (l ~ /^[[:space:]]*(union|struct|class)[[:space:]]*({[[:space:]]*)?$/) {
    pendkind = "class"
    pendname = (top > 0 && skind[top] != "block") ? sname[top] : "-"
    pendacc  = (l ~ /^[[:space:]]*class/) ? 0 : 1
    return
  }

  if (match(l, /(^|[^A-Za-z0-9_])enum[[:space:]]+[A-Za-z_][A-Za-z0-9_]*/)) {
    tok  = substr(l, RSTART, RLENGTH)
    rest = substr(l, RSTART + RLENGTH); sub(/^[[:space:]]*/, "", rest)
    if (rest == "" || rest ~ /^[{:]/) {
      nm = last_ident(tok)
      emit("type", nm, "-")
      pendkind = "enum"; pendname = nm; pendacc = 1
      return
    }
    if (rest ~ /^;/) return
  }

  if (kind == "enum") {
    decl = l; sub(/}.*$/, "", decl); sub(/{/, "", decl)
    emit_list("enum", decl); return
  }

  # A constant: `static const unsigned int DT_PARTIAL_RESULT = 1 << 6;` at file
  # scope and `static const int MAX_LOCAL_SEGS = 8;` inside a class read the
  # same way. The initialiser may call a function, so this runs before the
  # function rule.
  if (l ~ /(^|[^A-Za-z0-9_])static[[:space:]]+const[[:space:]]/ ||
      (kind == "file" && l ~ /^[[:space:]]*const[[:space:]].*=/)) {
    decl = l; sub(/;.*$/, "", decl); sub(/=.*$/, "", decl)
    gsub(/\[[^]]*\]/, "", decl)
    emit("const", last_ident(decl), "-")
    return
  }

  # An operator has no name a C ABI can carry, and its declaration would
  # otherwise be read as whatever call its inline body makes first.
  if (l ~ /(^|[^A-Za-z0-9_])operator([^A-Za-z0-9_]|$)/) return

  # A function is the first identifier followed by "(" — first, because a
  # constructor's member-initialiser list is a run of identifiers followed by
  # "(" too, and only the leftmost is the function being declared. A "{"
  # earlier on the line means the match is inside a body, not a declaration.
  if (match(l, /[A-Za-z_][A-Za-z0-9_]*[[:space:]]*\(/)) {
    brace = index(substr(l, 1, RSTART), "{")
    pre = (RSTART > 1) ? substr(l, RSTART - 1, 1) : ""
    nm  = substr(l, RSTART, RLENGTH - 1); gsub(/[[:space:]()]/, "", nm)
    if (brace == 0 && pre != "." && pre != ">" && pre != ":" &&
        !is_keyword(nm) && !is_typeword(nm)) {
      emit("func", (pre == "~") ? "~" nm : nm, arity(substr(l, RSTART)),
           paramtypes(substr(l, RSTART)))
      return
    }
    # Not a declaration; a declarator list never contains "(" in these headers.
    return
  }

  # A data member. Only inside a type: at file scope a bare declaration would
  # be a variable definition, and Recast has none outside `static const`.
  if (kind == "class" && l ~ /;[[:space:]]*$/ &&
      l !~ /^[[:space:]]*(using|friend|return|}|\{)/) {
    decl = l; sub(/;.*$/, "", decl)
    emit_list("field", decl)
  }
}

# Every brace opens a scope, so a function body is a scope too and nothing
# inside one is mistaken for a declaration.
function update_depth(l,   i, c) {
  for (i = 1; i <= length(l); i++) {
    c = substr(l, i, 1)
    if (c == "{") {
      depth++; top++
      sdepth[top] = depth
      skind[top]  = (pendkind != "") ? pendkind : "block"
      sname[top]  = (pendkind != "") ? pendname : "-"
      sacc[top]   = (pendkind != "") ? pendacc : 1
      pendkind = ""; pendname = ""
    }
    else if (c == "}") {
      if (top > 0 && sdepth[top] == depth) top--
      if (depth > 0) depth--
    }
  }
}
