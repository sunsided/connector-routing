//! The orthogonal drawing pipeline of Hegemann and Wolff (arXiv 2309.01671) for fixed
//! boxes: port assignment, routing, crossing reduction, path ordering and nudging.
//!
//! ```mermaid
//! flowchart LR
//!     B[boxes and edges] --> P[port assignment]
//!     P --> G[orthogonal visibility graph]
//!     G --> R[negotiated routing]
//!     R --> C[crossing reduction]
//!     C --> N[ordering and nudging]
//!     N --> D[drawing]
//! ```
//!
//! The paper's own routing graph (channel representatives) is not implemented; the
//! orthogonal visibility graph contains every route it could offer. Its full nudging
//! mode, which moves and grows boxes, belongs to a layout engine; the exported
//! [`nudge::constraints`] are the interface for that.

mod ports;

use alloc::vec::Vec;

use crate::Connector;
use crate::crossing;
use crate::error::Error;
use crate::geom::{Point, Rect};
use crate::global;
use crate::nudge;
use crate::scene::{Port, Scene};

pub use ports::assign_ports;

/// Parameters of the pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Global routing parameters.
    pub routing: global::Config,
    /// Nudging parameters.
    pub nudging: nudge::Config,
    /// The maximum number of crossing-reduction rounds; zero disables it.
    pub crossing_rounds: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            routing: global::Config::default(),
            nudging: nudge::Config::default(),
            crossing_rounds: 4,
        }
    }
}

/// A finished drawing.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Drawing {
    /// The ports of each edge, `(source, target)`, in input order.
    pub ports: Vec<(Port, Port)>,
    /// One polyline per edge, in input order, from its source port to its target port.
    pub routes: Vec<Vec<Point>>,
    /// The number of crossings between different edges.
    pub crossings: usize,
    /// The total channel overuse left after negotiation (zero when every channel is
    /// within capacity).
    pub overuse: u64,
    /// The smallest separation nudging achieved, if any segments needed separating.
    pub separation: Option<u32>,
}

/// Draws `edges` (index pairs into `boxes`; multi-edges and self-loops allowed) between
/// fixed, non-overlapping `boxes`.
///
/// # Errors
///
/// [`Error::CoordinateOutOfRange`] for boxes beyond [`MAX_COORD`](crate::MAX_COORD),
/// [`Error::NoRoute`] if an edge cannot be routed, and the errors of
/// [`global::route_all`].
///
/// # Panics
///
/// Panics if an edge refers to a box index out of range.
///
/// # Examples
///
/// ```
/// use connector_routing::{Rect, pipeline};
///
/// let boxes = [Rect::new(0, 0, 20, 10), Rect::new(60, 40, 80, 50), Rect::new(60, -40, 80, -30)];
/// let edges = [(0, 1), (0, 2), (1, 2)];
/// let drawing = pipeline::draw(&boxes, &edges, &pipeline::Config::default())?;
/// assert_eq!(drawing.routes.len(), 3);
/// assert_eq!(drawing.crossings, 0);
/// # Ok::<(), connector_routing::Error>(())
/// ```
pub fn draw(boxes: &[Rect], edges: &[(usize, usize)], config: &Config) -> Result<Drawing, Error> {
    let ports = assign_ports(boxes, edges);
    let scene = Scene::build(
        boxes.iter().copied(),
        ports.iter().flat_map(|&(a, b)| [a, b]),
    )?;
    // Ports get ids in input order, two per edge.
    let ids: Vec<_> = scene.ports().map(|(id, _)| id).collect();
    let connectors: Vec<Connector<_>> = ids
        .chunks(2)
        .map(|pair| Connector {
            source: pair[0],
            target: pair[1],
        })
        .collect();

    let mut routing = global::route_all(&scene, &connectors, &config.routing)?;
    if config.crossing_rounds > 0 {
        let weights = config.routing.objective.weights(scene.graph())?;
        crossing::reduce_crossings(
            scene.graph(),
            &mut routing.routes,
            weights,
            config.crossing_rounds,
        );
    }

    // Crossing reduction does not check capacities, so measure the overuse again.
    let overuse = global::overuse(&scene, &routing.routes, config.routing.separation);
    let nudged = nudge::nudge(scene.graph(), boxes, &routing.routes, &config.nudging);
    let mut crossings = 0;
    for (i, a) in nudged.routes.iter().enumerate() {
        for b in &nudged.routes[i + 1..] {
            crossings += crate::check::orthogonal_crossings(a, b);
        }
    }

    Ok(Drawing {
        ports,
        routes: nudged.routes,
        crossings,
        overuse,
        separation: nudged.separation,
    })
}
