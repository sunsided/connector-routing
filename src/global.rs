//! Global routing of many connectors with `PathFinder` negotiated congestion
//! (`McMurchie` and Ebeling 1995), rip-up and reroute.
//!
//! Every connector is routed with the single-connector A* over the orthogonal
//! visibility graph, with edge costs raised where channels are over capacity:
//!
//! ```text
//! cost(edge) = sum over its resources r of width_r * length_weight
//!              * (1 + history_r) * (1 + present * overuse_r)
//! ```
//!
//! where `overuse_r` is how far `r` would be over capacity with this connector added.
//! After each iteration `history_r` grows by the overuse and `present` doubles, so
//! connectors negotiate: the ones with alternatives move away from contested channels.
//! A crossing penalty is charged for passing straight through a vertex that another
//! connector passes straight through perpendicularly (a certain crossing).
//!
//! ```mermaid
//! flowchart LR
//!     O[order connectors] --> R[route all]
//!     R --> C{overuse?}
//!     C -- no --> D[done]
//!     C -- yes --> H[raise history and present cost]
//!     H --> U[rip up and reroute connectors on overused resources]
//!     U --> C
//! ```
//!
//! Not checked against the original paper (unavailable); see `docs/design.md`.

mod resources;

use alloc::vec::Vec;

use crate::Connector;
use crate::error::Error;
use crate::geom::Direction;
use crate::graph::VertexId;
use crate::route::Route;
use crate::scene::{PortId, Scene};
use crate::search::{EdgeCosts, Endpoints, Objective, Weights, endpoints, route_vertices};

pub(crate) use resources::Resources;

/// Parameters of negotiated congestion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// What each connector minimises before congestion is added.
    pub objective: Objective,
    /// The track pitch: the minimum distance nudging should keep between parallel
    /// segments. A channel of height `h` holds `h / separation + 1` tracks.
    pub separation: u32,
    /// The cost of a certain crossing with another connector.
    pub crossing_penalty: u64,
    /// The maximum number of routing iterations, the first included.
    pub max_iterations: u32,
    /// How much a resource's history cost grows per unit of overuse per iteration.
    pub history_increment: u64,
    /// The present-congestion factor of the first iteration; doubled every iteration.
    pub initial_present: u64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            objective: Objective::Weighted {
                length: 1,
                bend: 10,
            },
            separation: 4,
            crossing_penalty: 20,
            max_iterations: 40,
            history_increment: 1,
            initial_present: 1,
        }
    }
}

/// The result of [`route_all`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Routing {
    /// One route per connector, in input order.
    pub routes: Vec<Route>,
    /// The number of iterations run.
    pub iterations: u32,
    /// The total overuse left: the sum over resources of tracks used beyond capacity.
    /// Zero means every channel is within capacity.
    pub overuse: u64,
    /// The number of certain crossings left (straight passes through a common vertex).
    pub crossings: u64,
}

/// The mutable congestion state of one run.
struct Congestion<'a> {
    /// The resources and capacities.
    resources: &'a Resources,
    /// Tracks in use per resource.
    occupancy: Vec<u32>,
    /// History cost per resource.
    history: Vec<u64>,
    /// The present-congestion factor.
    present: u64,
    /// Per vertex slot, the number of connectors passing straight through horizontally
    /// and vertically.
    straight: Vec<[u32; 2]>,
    /// The cost of one certain crossing.
    crossing_penalty: u64,
}

impl EdgeCosts for Congestion<'_> {
    fn edge(&self, from: VertexId, dir: Direction, to: VertexId, length_cost: u64) -> u64 {
        let uses = self.resources.of_edge(from, dir, to);
        let total_width: u64 = uses.iter().map(|&(_, w)| u64::from(w)).sum();
        let unit = length_cost.checked_div(total_width).unwrap_or(0);
        uses.iter().fold(0_u64, |sum, &(r, width)| {
            let r = r as usize;
            let over =
                u64::from((self.occupancy[r] + 1).saturating_sub(self.resources.capacity[r]));
            let factor =
                (1 + self.history[r]).saturating_mul(1 + self.present.saturating_mul(over));
            sum.saturating_add(unit.saturating_mul(u64::from(width)).saturating_mul(factor))
        })
    }

    fn pass(&self, at: VertexId, entry: Option<Direction>, exit: Direction) -> u64 {
        if entry != Some(exit) {
            return 0;
        }

        let perpendicular = match exit.axis() {
            crate::geom::Axis::X => self.straight[at.index()][1],
            crate::geom::Axis::Y => self.straight[at.index()][0],
        };

        self.crossing_penalty
            .saturating_mul(u64::from(perpendicular))
    }
}

impl Congestion<'_> {
    /// Adds (`sign = true`) or removes the occupancy and straight passes of `route`.
    fn apply(&mut self, graph: &crate::graph::RoutingGraph, route: &Route, add: bool) {
        for pair in route.vertices.windows(2) {
            let dir =
                Direction::between(graph.point(pair[0]), graph.point(pair[1])).expect("aligned");
            for &(r, _) in self.resources.of_edge(pair[0], dir, pair[1]) {
                let slot = &mut self.occupancy[r as usize];
                *slot = if add { *slot + 1 } else { *slot - 1 };
            }
        }

        for triple in route.vertices.windows(3) {
            let a = graph.point(triple[0]);
            let b = graph.point(triple[1]);
            let c = graph.point(triple[2]);
            if Direction::between(a, b) == Direction::between(b, c) {
                let axis = usize::from(a.y != b.y);
                let slot = &mut self.straight[triple[1].index()][axis];
                *slot = if add { *slot + 1 } else { *slot - 1 };
            }
        }
    }

    /// The tracks used beyond capacity, summed over resources.
    fn overuse(&self) -> u64 {
        self.occupancy
            .iter()
            .zip(&self.resources.capacity)
            .map(|(&used, &cap)| u64::from(used.saturating_sub(cap)))
            .sum()
    }

    /// Whether `route` uses an overused resource.
    fn uses_overused(&self, graph: &crate::graph::RoutingGraph, route: &Route) -> bool {
        route.vertices.windows(2).any(|pair| {
            let dir =
                Direction::between(graph.point(pair[0]), graph.point(pair[1])).expect("aligned");
            self.resources
                .of_edge(pair[0], dir, pair[1])
                .iter()
                .any(|&(r, _)| self.occupancy[r as usize] > self.resources.capacity[r as usize])
        })
    }

    /// Raises history costs by the current overuse and doubles the present factor.
    fn negotiate(&mut self, increment: u64) {
        for (r, h) in self.history.iter_mut().enumerate() {
            let over = self.occupancy[r].saturating_sub(self.resources.capacity[r]);
            *h = h.saturating_add(increment.saturating_mul(u64::from(over)));
        }

        self.present = self.present.saturating_mul(2);
    }

    /// The number of certain crossings: per vertex, horizontal times vertical straight
    /// passes.
    fn crossings(&self) -> u64 {
        self.straight
            .iter()
            .map(|&[h, v]| u64::from(h) * u64::from(v))
            .sum()
    }
}

/// Routes all connectors together with negotiated congestion.
///
/// The connector order is deterministic: by decreasing Manhattan distance between the
/// ports, ties by input index. The first iteration routes every connector; later
/// iterations rip up and reroute those on an overused resource, until nothing is
/// overused or [`Config::max_iterations`] is reached. Remaining overuse is reported, not
/// an error.
///
/// # Errors
///
/// [`Error::UnknownPort`] or [`Error::PortBlocked`] for a bad port,
/// [`Error::CostOverflow`] if the objective's weights overflow, [`Error::NoRoute`] if a
/// connector's ports are not connected.
///
/// # Examples
///
/// ```
/// use connector_routing::{Connector, DirSet, Point, Port, Rect, Scene, global};
///
/// let mut scene = Scene::new();
/// scene.add_obstacle(Rect::new(0, 0, 10, 10))?;
/// let a = scene.add_port(Port::new(Point::new(-10, 2), DirSet::RIGHT))?;
/// let b = scene.add_port(Port::new(Point::new(20, 2), DirSet::LEFT))?;
/// let c = scene.add_port(Port::new(Point::new(-10, 8), DirSet::RIGHT))?;
/// let d = scene.add_port(Port::new(Point::new(20, 8), DirSet::LEFT))?;
/// let connectors = [Connector { source: a, target: b }, Connector { source: c, target: d }];
///
/// let routing = global::route_all(&scene, &connectors, &global::Config::default())?;
/// assert_eq!(routing.routes.len(), 2);
/// assert_eq!(routing.overuse, 0);
/// # Ok::<(), connector_routing::Error>(())
/// ```
pub fn route_all(
    scene: &Scene,
    connectors: &[Connector<PortId>],
    config: &Config,
) -> Result<Routing, Error> {
    let graph = scene.graph();
    let weights = config.objective.weights(graph)?;
    let ends: Vec<Endpoints> = connectors
        .iter()
        .map(|c| endpoints(scene, c.source, c.target))
        .collect::<Result<_, _>>()?;
    let order = net_order(graph, &ends);
    let resources = Resources::new(graph, &scene.rects(), config.separation);
    let mut congestion = Congestion {
        resources: &resources,
        occupancy: alloc::vec![0; resources.len()],
        history: alloc::vec![0; resources.len()],
        present: config.initial_present,
        straight: alloc::vec![[0, 0]; graph.capacity()],
        crossing_penalty: config.crossing_penalty,
    };

    let mut routes: Vec<Option<Route>> = alloc::vec![None; connectors.len()];
    let mut iterations = 0;
    while iterations < config.max_iterations.max(1) {
        iterations += 1;
        let reroute: Vec<usize> = order
            .iter()
            .copied()
            .filter(|&i| {
                routes[i]
                    .as_ref()
                    .is_none_or(|r| congestion.uses_overused(graph, r))
            })
            .collect();
        if reroute.is_empty() {
            break;
        }

        for i in reroute {
            reroute_one(graph, &mut congestion, &mut routes[i], ends[i], weights, i)?;
        }

        if congestion.overuse() == 0 {
            break;
        }

        congestion.negotiate(config.history_increment);
    }

    Ok(Routing {
        // The first iteration routes every connector, so none is missing.
        routes: routes.into_iter().flatten().collect(),
        iterations,
        overuse: congestion.overuse(),
        crossings: congestion.crossings(),
    })
}

/// The total channel overuse of `routes` in `scene` for tracks of pitch `separation`:
/// the sum over resources of tracks used beyond capacity (see [`route_all`]).
#[must_use]
pub fn overuse(scene: &Scene, routes: &[Route], separation: u32) -> u64 {
    let graph = scene.graph();
    let resources = Resources::new(graph, &scene.rects(), separation);
    let mut congestion = Congestion {
        resources: &resources,
        occupancy: alloc::vec![0; resources.len()],
        history: alloc::vec![0; resources.len()],
        present: 0,
        straight: alloc::vec![[0, 0]; graph.capacity()],
        crossing_penalty: 0,
    };
    for route in routes {
        congestion.apply(graph, route, true);
    }

    congestion.overuse()
}

/// Rips up `slot`'s route (if any) and routes the connector again under the current
/// congestion.
fn reroute_one(
    graph: &crate::graph::RoutingGraph,
    congestion: &mut Congestion<'_>,
    slot: &mut Option<Route>,
    ends: Endpoints,
    weights: Weights,
    index: usize,
) -> Result<(), Error> {
    if let Some(old) = slot.take() {
        congestion.apply(graph, &old, false);
    }

    let route = route_vertices(graph, ends, weights, &*congestion)
        .ok_or(Error::NoRoute { connector: index })?;
    congestion.apply(graph, &route, true);
    *slot = Some(route);

    Ok(())
}

/// The routing order: by decreasing Manhattan distance between the endpoints, ties by
/// index.
fn net_order(graph: &crate::graph::RoutingGraph, ends: &[Endpoints]) -> Vec<usize> {
    let mut order: Vec<usize> = (0..ends.len()).collect();
    order.sort_by_key(|&i| {
        let span = graph
            .point(ends[i].source)
            .manhattan(graph.point(ends[i].target));
        (core::cmp::Reverse(span), i)
    });

    order
}
