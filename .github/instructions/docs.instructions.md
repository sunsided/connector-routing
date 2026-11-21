---
applyTo: "**/*.md,docs/**"
---

# Documentation

- README files are rendered on crates.io and docs.rs. Every link to a repo file and every
  image uses an absolute URL (`https://github.com/...`, `https://raw.githubusercontent.com/...`).
  Only in-page anchors may be relative.
- Mermaid diagrams (```` ```mermaid ````) must be valid (`task lint:mermaid` renders them)
  and must match the code. Flag diagrams that the change makes stale. Keep each diagram under
  about 15 nodes. The crate root and the module docs of `scene`, `global`, `nudge` and
  `pipeline` carry diagrams.
- Code blocks are tagged with their language. The README's Rust example is doctested
  through the private `#[cfg(doctest)]` module in `src/lib.rs`; keep it compiling and in step
  with the crate-level example.
- `docs/design.md` records the models, the deviations from the papers that testing found,
  and the paper table. Update it when a router, objective or nudging model is added or
  changes shape, and keep the README "Scope" list in step.
- Keep badges, the License section and the MSRV statement consistent with `Cargo.toml`.
