# Contributing

Issues and pull requests are welcome. [BINDING.md](../BINDING.md) is the
walkthrough for adding surface; these are the three rules that fail a build.

- **`libs/recastnavigation` is vendored and must not be edited, at all.**
  `ci/verify-vendor.sh` diffs it against a fresh clone of the pinned commit, so
  any local edit fails the build. If upstream needs fixing, fix it upstream; if
  zrecast needs to work around upstream, do it in `ffi/` and record the reason
  in [UPSTREAM.md](../UPSTREAM.md).
- **Run `ci/run.sh` before pushing** — or `ci/install-hooks.sh` once, and it
  runs itself. It is the same matrix CI runs, and every step of it must be
  green.
- **A new binding also adds a line to `tools/bindings.tsv`.** Every public
  Recast and Detour name is either answered there or is a gap, and the evidence
  has to name the symbol in `ffi/zrecast.h` that a host now calls. Then run
  `tools/record.sh`, which rewrites `tools/unbound_*.txt` from that file, the
  upstream harvest and the tranche rules — the record is generated, and
  `ci/check-coverage.sh` fails if what is checked in differs. The gate checks
  that the symbol exists; it cannot check that the evidence is apt, so say what
  the mapping is when it is not obvious.

New source files are added to the explicit lists in `build.zig` deliberately;
there are no globs, so nothing starts compiling by accident.

A new invariant gets a probe in `ci/probes/` naming the test that must fail
when it is broken; `ci/probe.sh` refuses a probe the suite survives. A new C
entry point gets `CHECK`s in `tests/c_smoke.c`. If any number the README states
moved, `ci/check-docs.sh --write` regenerates the blocks.
