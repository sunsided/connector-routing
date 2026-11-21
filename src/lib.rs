//! # Connector routing
//!
//! Global connector routing: many orthogonal connectors at once with bend, length and
//! crossing costs, negotiated congestion and rip-up and reroute.
//!
//! Repository: <https://github.com/sunsided/connector-routing>

#![no_std]

extern crate alloc;

pub mod check;
pub mod crossing;
mod error;
mod geom;
pub mod global;
mod graph;
pub mod incremental;
mod nets;
pub mod nudge;
pub mod pipeline;
mod route;
mod scene;
mod search;

pub use error::Error;
pub use geom::{Axis, DirSet, Direction, MAX_COORD, Point, Rect, Side};
pub use graph::{RoutingGraph, VertexId};
pub use nets::Net;
pub use route::{Route, simplify};
pub use scene::{ObstacleId, Port, PortId, Scene};
pub use search::{
    EdgeCosts, Endpoints, Objective, PlainCosts, Search, Weights, cost_of, endpoints, min_bends,
    route, route_vertices,
};

/// A connector to route between two ports.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Connector<P> {
    /// The port the connector starts at.
    pub source: P,
    /// The port the connector ends at.
    pub target: P,
}
