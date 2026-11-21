---
applyTo: "**/*.rs"
---

# Rust code

- Every module (`//!`), type, field, variant, function, constant and macro has a doc
  comment, private and `pub(crate)` items included. Docs state intent, invariants, units and
  error/panic conditions (`# Errors`, `# Panics`). They do not restate the name.
- No `mod.rs`: modules with children are `foo.rs` plus `foo/`. Integration tests live under
  `tests/it/`, with shared strategies and oracles in `tests/it/support.rs`. Property tests
  take their case count from `support::cases`, so `PROPTEST_CASES` can raise it.
- No separator comments (`// -----`, `// =====`). If a file needs sections, split it into
  submodules.
- Leave a blank line after the closing `}` of a block statement when another statement
  follows, and between logical phases of a function. No lumped code.
- One level of abstraction per function. Search and solver loops are short drivers that
  call documented step functions. If a block needs a "what" comment, extract it into a
  function named after that comment.
- `clippy::pedantic` clean. Suppression only with a scoped
  `#[expect(lint, reason = "...")]`. Never `#[allow]`, and never crate-level suppression.
- No `unsafe`: the crate root has `#![forbid(unsafe_code)]`.
- The crate is `no_std` with `alloc`: no `std` imports, no `HashMap` (use `BTreeMap` or
  sorted vectors). Results must be deterministic: no iteration order that depends on
  hashing or addresses.
- Geometry is exact: coordinates are `i32` within `MAX_COORD`, predicates are evaluated in
  `i64` or `i128` without overflow, and costs saturate or return `Error::CostOverflow`.
  No floating point in routing decisions. Reject changes that weaken this.
- Routes never enter an obstacle's open interior; `check` validates that independently.
  Tests and the fuzz targets (`route_scene`, `sleeve_route`) check every route they get
  with `check::orthogonal` or `check::polyline`.
- Optimality claims are tested against brute force (`tests/it/support.rs`). A change that
  makes a search inexact must say so in its docs and in `docs/design.md`.
- Public API changes go with a matching update to the README "Scope and status" list and
  `docs/design.md`.
- Mermaid diagrams in doc comments must match the code they describe. Update them when the
  structure changes.
