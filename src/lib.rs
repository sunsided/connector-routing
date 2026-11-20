//! # Connector routing
//!
//! Global connector routing: many orthogonal connectors at once with bend, length and crossing costs, negotiated congestion and rip-up and reroute.
//!
//! ## Planned scope
//!
//! - Orthogonal visibility graphs over rectangular obstacles
//! - Single-connector A* with bend, length and crossing costs
//! - Global routing of many connectors: `PathFinder` negotiated congestion, rip-up and reroute
//! - Conflict-based multi-net routing with channel capacities
//! - Nudging and ordering of parallel segments, port constraints
//! - Multi-objective routing (length, bends, crossings) via `multiobjective-search`
//!
//! ## Status
//!
//! Pre-release. Version 0.0.0 reserves the name; the API is not started yet and will change.
//!
//! Repository: <https://github.com/sunsided/connector-routing>

#![no_std]

/// A connector to route between two ports.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Connector<P> {
    /// The port the connector starts at.
    pub source: P,
    /// The port the connector ends at.
    pub target: P,
}
