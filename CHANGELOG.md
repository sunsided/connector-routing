# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.1](https://github.com/sunsided/connector-routing/compare/v0.1.0...v0.1.1) - 2026-11-22

### Changed

- Link the crate to its page on the planning and search site

## [0.1.0](https://github.com/sunsided/connector-routing/releases/tag/v0.1.0) - 2026-11-22

### Added

- Add dormant release automation with release-plz
- Add CONTRIBUTING.md
- Add git hooks and Copilot review instructions
- Add README badges, MSRV, contributing and license sections
- Add a property test for crossing reduction
- Add fuzz targets for orthogonal and sleeve routing
- Add REUSE licensing metadata
- Add cargo-deny, Dependabot and typos configuration
- Add the Codecov configuration
- Add the Taskfile and CI workflows
- Add the 2309.01671 channel graph and full nudging
- Add VPSC projection and water-filling to nudging
- Add the poly-line angle penalty and hidden-region pruning
- Add the hyperedge local improvement (segment shifting)
- Add Pareto fronts over length, bends and crossings
- Add orthogonal hyperedge routing (Wybrow et al. 2012)
- Add benches, speed up crossing reduction, harden any-angle routing
- Add incremental poly-line routing (Wybrow et al. 2005)
- Add sleeve routing over an exact constrained Delaunay triangulation
- Add the 2309.01671 drawing pipeline with port assignment
- Add path ordering, crossing reduction and exact nudging
- Add PathFinder global routing and change reports
- Add single-connector routing: OVG A* and the 1-bend graph
- Add exact geometry, the routing graph and the incremental OVG
- Add the design document

### Changed

- List the crossing points of finished routes for drawing hops
- Show any-angle routing next to orthogonal routing in the README
- Document that full nudging leaves Drawing::ports unmoved
- Set the version to 0.1.0 for the first release
- Show a routed drawing in the README
- Update criterion requirement
- Depend on the released multiobjective-search and drop the sibling checkout
- Depend on heuristic-search 0.2
- Depend on the released heuristic-search 0.1
- Describe the released scope and audience in the README
- Raise the MSRV to 1.86 and check every feature on it
- List every fuzz target in the review instructions
- Fuzz the pipeline, hyperedge trees and poly-line routing
- Check the MSRV with default features only
- Extract the single-connector benchmarks into a helper
- Split the pipeline, the interleaved Steiner search and full nudging
- Extend the lint table, clippy.toml and rustfmt.toml
- Check the no_std build on a bare-metal target
- Render mermaid diagrams in the rustdoc output
- Let PROPTEST_CASES override the property test case counts
- Invoke the sibling checkout action by its local path
- Let sleeve routes leave slits between touching obstacles
- Recompute overuse after crossing reduction; test touching obstacles
- Order shared edges by pseudo directions; add forced-order arcs
- Document the crate, its status and the open questions
- Format the crossing-reduction rounds
- Reserve the crate name with a 0.0.0 placeholder

### Fixed

- Fix poly-line reroutes for endpoints covered by a later obstacle
