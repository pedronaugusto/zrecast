# Coverage — how the record is measured

`zig build test` says the code works. It says nothing about how much of Recast
and Detour a host can get at. That question is answered by a generated record,
one line per public upstream name, and by a gate that re-derives the name list
from the vendored headers so a name cannot go missing quietly.

## The verdicts

Every public name in `Recast/`, `Detour/`, `DetourCrowd/`, `DetourTileCache/`
and `DebugUtils/` carries exactly one:

- **`BOUND`** — reachable through the C boundary. The evidence names the `zrc`
  symbol that carries it, and the gate checks that symbol is declared in
  `ffi/zrecast.h`.
- **`LANGUAGE`** — C++-only surface a C boundary cannot carry: `rcVectorBase`,
  `rcIntArray`, `rcTempVector`, `rcPermVector`, `rcScopedDelete`,
  `rcScopedTimer`, `dtNodeQueue`, the `dtMath*` wrappers over libm, the
  `dtSwapEndian` overloads, `rcSqrt`, `RC_PI`. The evidence says what a C or
  Zig host does instead.
- **`ZIG`** — reimplemented on the Zig side: the `dtV`/`rcV` vector families
  and the scalar helpers around them. The evidence names the mirror in
  `src/vec.zig`, and that file's bit-identity test against linkable shims over
  upstream's own `inline` definitions is what makes a mirror true.
- **`UNDEFINED`** — a header declares it and no vendored source defines it, so
  there is no symbol for any host to call. The evidence names the header and
  line, and the gate rechecks both halves against the tree: a re-vendor that
  supplies the definition fails until the name is bound. One name carries it,
  `duDebugDrawHeightfieldLayersRegions`, which would also have nothing to draw
  — `rcHeightfieldLayer` carries no region ids.

## The commands

```sh
tools/coverage.sh              # the summary, per area
tools/coverage.sh --names      # every public name upstream declares
tools/coverage.sh --audit      # every line the harvester did not understand
tools/coverage.sh --collapsed  # symbols covering more than one declaration
tools/record.sh                # rewrite the record from its three inputs
ci/check-coverage.sh --list    # the gate, and the open gaps
```

`tools/unbound_*.txt` is generated. `tools/record.sh` writes it from the
harvest, from `tools/bindings.tsv` (the names that are answered — the only half
kept by hand) and from `tools/tranches.awk` (which tranche closes each of the
rest), so a name cannot carry one verdict in one place and another somewhere
else, and a gap's reason is a rule that is re-read rather than a sentence that
rots. `ci/check-coverage.sh` refuses any verdict it cannot check: a `BOUND`
line has to name a symbol `ffi/zrecast.h` really declares, and a gap has to say
which tranche of work closes it.

The gate was red from the day it was written until the binding was complete,
and it has been part of `ci/run.sh` and of CI throughout. A coverage check that
only runs once the answer is comfortable is not a check.

## What the harvester counts, and what it used to count

The question used to be answered by a script that matched an identifier
followed by `(`. It counted functions and nothing else, so it missed
`dtNavMeshCreateParams`' 31 must-populate fields, the 12 `dtStatus` detail bits
that are `static const unsigned int` rather than an enum, and the 24 constants
in the six enums whose names carry no `rc` or `dt` prefix. Its answer, "7 of
421 names", was a true sentence about the wrong set.

What replaced it enumerates functions, data members, enum constants, `static
const` values and type names; keeps overloads apart by arity and — where an
arity is not enough, as for `dtSwapEndian`'s five one-argument forms — by
parameter type; and prints what it could not parse.

`DetourNavMeshQuery` is pulled out of `Detour` as its own area rather than
folded in: `dtNavMeshQuery` is the pathfinding engine, large enough to want its
own line rather than being buried next to `DetourAlloc`. `DebugUtils` is one
area covering its four headers.

Its blind spots are printed with the summary: preprocessor macros are not names
here (`rcLikely`, the assert macros), and operator overloads carry no name a C
ABI can spell.

## The documented numbers

Not one number in README.md or UPSTREAM.md is typed in by hand.
`ci/measurements.sh` recomputes them from the tree, `ci/check-docs.sh`
regenerates the block and fails the build if what is committed differs, and the
same gate refuses any other hand-written number in those documents unless
`tools/doc_numbers.txt` says why it cannot go stale. Adding a claim means
adding its measurement.

A count is a count. Matching entry-point counts prove presence, not
correctness — the behavioural tests, the C smoke test and the mutation probes
hold that, and `ci/check-coverage.sh` holds each verdict one name at a time.
Source lines measure volume, not surface. The gate itself has three blind
spots: a number spelled as a word, a number inside `code` — where it is an
identifier or a citation rather than a claim — and a sentence that is wrong
without any number in it.
