//! Pareto-optimal routes for one connector over length, bends and crossings (feature
//! `multi-objective`), via NAMOA*dr from `multiobjective-search`.
//!
//! The search runs over the same `(vertex, heading)` state graph as the single-objective
//! A*. Each move costs `[length, bend, crossings]`, where a crossing is a straight pass
//! through a vertex that one of the `others` routes passes straight through in the
//! perpendicular direction. The heuristic is `[Manhattan distance, exact free-space
//! bends, 0]`, consistent per objective.

use alloc::vec::Vec;

use multiobjective_search::{Graph, heuristic_fn, namoa_star_dr};

use crate::error::Error;
use crate::geom::{Axis, Direction};
use crate::graph::{RoutingGraph, VertexId};
use crate::route::Route;
use crate::scene::{PortId, Scene};
use crate::search::{Endpoints, endpoints, min_bends};

/// One Pareto-optimal route.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParetoRoute {
    /// The route (its `cost` is the length).
    pub route: Route,
    /// The cost vector `[length, bends, crossings]`.
    pub cost: [u64; 3],
}

/// A search state: vertex slot and heading index, 4 before the first move.
type State = (u32, u8);

/// The state graph with crossing counts.
struct Front<'a> {
    /// The routing graph.
    graph: &'a RoutingGraph,
    /// The connector's endpoints.
    ends: Endpoints,
    /// Per vertex slot, the straight passes of other routes: horizontal, vertical.
    straight: Vec<[u64; 2]>,
}

impl Graph for Front<'_> {
    type State = State;
    type Weight = [u64; 3];

    fn successors(&self, state: &State, out: &mut Vec<(State, [u64; 3])>) {
        let v = VertexId(state.0);
        let heading = (state.1 != 4).then(|| Direction::from_index(usize::from(state.1)));
        let options: Vec<Direction> = match heading {
            Some(h) => alloc::vec![h, h.right(), h.left()],
            None => self.ends.source_directions.iter().collect(),
        };
        for dir in options {
            let Some(w) = self.graph.neighbor(v, dir) else {
                continue;
            };

            let length = self.graph.point(v).manhattan(self.graph.point(w));
            let bend = u64::from(heading.is_some_and(|h| h != dir));
            let crossings = if heading == Some(dir) {
                match dir.axis() {
                    Axis::X => self.straight[v.index()][1],
                    Axis::Y => self.straight[v.index()][0],
                }
            } else {
                0
            };
            let index = u8::try_from(dir.index()).expect("four directions");
            out.push(((w.0, index), [length, bend, crossings]));
        }
    }
}

/// The straight passes of `routes` per vertex slot, horizontal and vertical.
fn straight_passes(graph: &RoutingGraph, routes: &[Route]) -> Vec<[u64; 2]> {
    let mut straight = alloc::vec![[0, 0]; graph.capacity()];
    for route in routes {
        for t in route.vertices.windows(3) {
            let (a, b, c) = (graph.point(t[0]), graph.point(t[1]), graph.point(t[2]));
            if Direction::between(a, b) == Direction::between(b, c) {
                straight[t[1].index()][usize::from(a.y != b.y)] += 1;
            }
        }
    }

    straight
}

/// Every Pareto-optimal route between two ports over `[length, bends, crossings]`
/// against the fixed `others`, sorted by length.
///
/// # Errors
///
/// [`Error::UnknownPort`] or [`Error::PortBlocked`] for a bad port.
///
/// # Examples
///
/// ```
/// use connector_routing::{Point, Port, Rect, Scene, pareto};
///
/// let mut scene = Scene::new();
/// scene.add_obstacle(Rect::new(10, 0, 20, 10))?;
/// let a = scene.add_port(Port::free(Point::new(0, 0)))?;
/// let b = scene.add_port(Port::free(Point::new(30, 10)))?;
/// let front = pareto::front(&scene, a, b, &[])?;
/// assert!(!front.is_empty());
/// assert!(front.windows(2).all(|w| w[0].cost[0] < w[1].cost[0]));
/// # Ok::<(), connector_routing::Error>(())
/// ```
pub fn front(
    scene: &Scene,
    source: PortId,
    target: PortId,
    others: &[Route],
) -> Result<Vec<ParetoRoute>, Error> {
    let graph = scene.graph();
    let ends = endpoints(scene, source, target)?;
    let search = Front {
        graph,
        ends,
        straight: straight_passes(graph, others),
    };
    let target_point = graph.point(ends.target);
    let arrivals = ends.target_directions.reversed();
    let heuristic = heuristic_fn(|s: &State| {
        let here = graph.point(VertexId(s.0));
        let bends = if s.1 == 4 {
            ends.source_directions
                .iter()
                .filter_map(|d| min_bends(here, d, target_point, arrivals))
                .min()
                .unwrap_or(0)
        } else {
            min_bends(
                here,
                Direction::from_index(usize::from(s.1)),
                target_point,
                arrivals,
            )
            .unwrap_or(0)
        };
        Some([here.manhattan(target_point), u64::from(bends), 0])
    });
    let target = ends.target.0;
    let is_goal = |s: &State| {
        s.0 == target && (s.1 == 4 || arrivals.contains(Direction::from_index(usize::from(s.1))))
    };
    let output = namoa_star_dr(&search, heuristic, (ends.source.0, 4), is_goal);

    Ok(output
        .solutions
        .into_iter()
        .map(|solution| {
            let vertices = solution.path.iter().map(|s| VertexId(s.0)).collect();
            ParetoRoute {
                route: Route::from_vertices(graph, vertices, solution.cost[0]),
                cost: solution.cost,
            }
        })
        .collect())
}
