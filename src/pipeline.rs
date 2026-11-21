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
//! The routing graph is the orthogonal visibility graph, or the paper's sparser channel
//! graph ([`GraphKind::Channels`]).

mod channels;
mod ports;

use alloc::vec::Vec;

use crate::Connector;
use crate::crossing;
use crate::error::Error;
use crate::geom::{Point, Rect};
use crate::global;
use crate::graph::RoutingGraph;
use crate::nudge;
use crate::scene::{Port, PortId, Scene};
use crate::search::Endpoints;

pub use channels::channel_graph;
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
    /// The routing graph to route on.
    pub graph: GraphKind,
    /// Use full nudging ([`nudge::full`]): boxes move too and the drawing is compacted.
    /// Falls back to constrained nudging if the separation cannot be met.
    pub full_nudging: bool,
}

/// The routing graph of the pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum GraphKind {
    /// The orthogonal visibility graph: every route the boxes allow.
    #[default]
    Visibility,
    /// The channel graph of arXiv 2309.01671 ([`channel_graph`]): sparser, one line per
    /// channel. If some edge cannot be routed on it, the drawing falls back to the
    /// visibility graph.
    Channels,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            routing: global::Config::default(),
            nudging: nudge::Config::default(),
            crossing_rounds: 4,
            graph: GraphKind::Visibility,
            full_nudging: false,
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
    /// The routing graph the drawing was routed on (the channel graph may fall back).
    pub graph: GraphKind,
    /// The boxes: the input, or moved by full nudging.
    pub boxes: Vec<Rect>,
    /// Whether full nudging was applied (it may fall back to constrained nudging).
    pub full: bool,
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
    let (channels, on_channels) = (config.graph == GraphKind::Channels)
        .then(|| channel_routing(&scene, boxes, &ports, config))
        .flatten()
        .unzip();
    let (graph, mut routing, kind) = match (&channels, on_channels) {
        (Some(graph), Some(routing)) => (graph, routing, GraphKind::Channels),
        _ => (
            scene.graph(),
            global::route_all(&scene, &connectors(&scene), &config.routing)?,
            GraphKind::Visibility,
        ),
    };
    if config.crossing_rounds > 0 {
        let weights = config.routing.objective.weights(graph)?;
        crossing::reduce_crossings(graph, &mut routing.routes, weights, config.crossing_rounds);
    }

    // Crossing reduction does not check capacities, so measure the overuse again.
    let overuse = global::overuse_on(graph, boxes, &routing.routes, config.routing.separation);
    let nudged = nudge::nudge(graph, boxes, &routing.routes, &config.nudging);
    let full = config
        .full_nudging
        .then(|| nudge::full(graph, boxes, &routing.routes, &config.nudging))
        .flatten();
    let (routes, out_boxes) = match &full {
        Some(f) => (f.routes.clone(), f.boxes.clone()),
        None => (nudged.routes.clone(), boxes.to_vec()),
    };

    Ok(Drawing {
        ports,
        crossings: crossings(&routes),
        routes,
        overuse,
        separation: nudged.separation,
        graph: kind,
        boxes: out_boxes,
        full: full.is_some(),
    })
}

/// One connector per edge, between the two ports [`Scene::build`] added for it (ports get
/// ids in input order, two per edge).
fn connectors(scene: &Scene) -> Vec<Connector<PortId>> {
    let ids: Vec<_> = scene.ports().map(|(id, _)| id).collect();

    ids.chunks(2)
        .map(|pair| Connector {
            source: pair[0],
            target: pair[1],
        })
        .collect()
}

/// Routes every edge on the channel graph of arXiv 2309.01671, in a frame grown around the
/// boxes by a margin that leaves room for nudging.
///
/// Returns the graph and its routing, or `None` if a port is not a channel-graph vertex or
/// routing on the channels fails; the caller then falls back to the visibility graph.
fn channel_routing(
    scene: &Scene,
    boxes: &[Rect],
    ports: &[(Port, Port)],
    config: &Config,
) -> Option<(RoutingGraph, global::Routing)> {
    let margin = i32::try_from(4 * config.nudging.separation + 8).unwrap_or(64);
    let frame = boxes
        .iter()
        .copied()
        .reduce(|a, b| a.union(&b))
        .map_or(Rect::new(-margin, -margin, margin, margin), |r| {
            r.buffer(margin)
        });
    let graph = channel_graph(
        boxes,
        &scene.ports().map(|(_, p)| p).collect::<Vec<_>>(),
        frame,
    );
    let ends: Option<Vec<Endpoints>> = ports
        .iter()
        .map(|&(a, b)| {
            Some(Endpoints {
                source: graph.vertex_at(a.point)?,
                source_directions: a.directions,
                target: graph.vertex_at(b.point)?,
                target_directions: b.directions,
            })
        })
        .collect();
    let routing = global::route_all_on(&graph, boxes, &ends?, &config.routing).ok()?;

    Some((graph, routing))
}

/// The number of crossings between different routes of a drawing.
fn crossings(routes: &[Vec<Point>]) -> usize {
    routes
        .iter()
        .enumerate()
        .flat_map(|(i, a)| routes[i + 1..].iter().map(move |b| (a, b)))
        .map(|(a, b)| crate::check::orthogonal_crossings(a, b))
        .sum()
}
