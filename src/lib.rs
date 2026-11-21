//! # Connector routing
//!
//! Global connector routing: route many connectors at once around rectangular obstacles
//! (diagram nodes, schematic parts, cabinets in a cable plan), minimising length, bends,
//! crossings and overlaps, and place the final segments apart from each other.
//!
//! All geometry is exact integer arithmetic ([`Point`] coordinates within
//! [`MAX_COORD`]); there are no floating-point robustness issues and results are
//! reproducible. The crate is `no_std` and needs only `alloc`.
//!
//! ## Orthogonal routing
//!
//! ```mermaid
//! flowchart LR
//!     S[Scene: obstacles, ports, OVG] --> A[route: one connector]
//!     S --> G[global::route_all: PathFinder]
//!     G --> C[crossing::reduce_crossings]
//!     C --> N[nudge::nudge]
//!     P[pipeline::draw] --> S
//! ```
//!
//! - [`Scene`]: obstacles and [`Port`]s with their orthogonal visibility graph
//!   (Wybrow, Marriott and Stuckey 2009), updated incrementally as obstacles and ports
//!   change. [`incremental`] reports which committed routes a change affects.
//! - [`route`]: the optimal route of one connector under an [`Objective`] (length and
//!   bends, exact lexicographic orders available), by A* over `(vertex, heading)`; the
//!   1-bend visibility graph of Marriott et al. (2014) is available as [`Search::OneBend`].
//! - [`global::route_all`]: many connectors with `PathFinder` negotiated congestion over
//!   channel capacities, rip-up and reroute, and crossing penalties.
//! - [`crossing`]: required crossings between routes and crossing reduction.
//! - [`nudge`]: ordering of shared segments and exact separation by difference
//!   constraints (no LP); the constraints are exported for layout engines.
//! - [`pipeline::draw`]: the pipeline of arXiv 2309.01671 from boxes and edges to a
//!   drawing, port assignment included.
//! - [`hyperedge`]: nets with more than two ports as rectilinear Steiner trees
//!   (Wybrow et al. 2012).
//! - [`check`]: independent validation of routes.
//!
//! ## Any-angle routing
//!
//! - [`sleeve`]: sleeve routing through an exact constrained Delaunay triangulation
//!   with the funnel algorithm (arXiv 2605.17498).
//! - [`polyline`]: incremental shortest poly-line routing over the visibility graph
//!   (Wybrow et al. 2005).
//!
//! ## Features
//!
//! - `multi-objective`: `pareto::front`, the Pareto front of one connector over
//!   length, bends and crossings (NAMOA*dr from `multiobjective-search`).
//!
//! ## Example
//!
//! ```
//! use connector_routing::{Connector, DirSet, Point, Port, Rect, Scene, check, global, nudge};
//!
//! let mut scene = Scene::new();
//! let boxes = [Rect::new(0, 0, 20, 10), Rect::new(60, 0, 80, 10), Rect::new(30, -30, 50, 40)];
//! for b in boxes {
//!     scene.add_obstacle(b)?;
//! }
//!
//! // Two connectors from the left box to the right box, around the box in between.
//! let a = scene.add_port(Port::new(Point::new(20, 3), DirSet::RIGHT))?;
//! let b = scene.add_port(Port::new(Point::new(60, 3), DirSet::LEFT))?;
//! let c = scene.add_port(Port::new(Point::new(20, 7), DirSet::RIGHT))?;
//! let d = scene.add_port(Port::new(Point::new(60, 7), DirSet::LEFT))?;
//! let connectors = [Connector { source: a, target: b }, Connector { source: c, target: d }];
//!
//! let routing = global::route_all(&scene, &connectors, &global::Config::default())?;
//! let nudged = nudge::nudge(scene.graph(), &boxes, &routing.routes, &nudge::Config::default());
//! for route in &nudged.routes {
//!     assert!(check::orthogonal(route, &boxes).is_ok());
//! }
//!
//! assert_eq!(nudged.overlaps, 0);
//! # Ok::<(), connector_routing::Error>(())
//! ```
//!
//! See `docs/design.md` for the models, the deviations from the papers found by
//! testing, and the paper table.
//!
//! ## Status
//!
//! Pre-release; the API will change.
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
pub mod hyperedge;
pub mod incremental;
mod nets;
pub mod nudge;
#[cfg(feature = "multi-objective")]
pub mod pareto;
pub mod pipeline;
pub mod polyline;
mod route;
mod scene;
mod search;
pub mod sleeve;

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
