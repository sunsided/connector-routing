//! Single-connector routing: A* over `(vertex, heading)` in a routing graph (Wybrow,
//! Marriott and Stuckey 2009, section 4).
//!
//! The state is a graph vertex and the direction of the last move (or "not moved yet"
//! at the source). Moves never reverse. A move costs its length times the length
//! weight, plus the bend weight when it changes the heading, plus whatever extra cost
//! the caller charges ([`EdgeCosts`]). The heuristic is the Manhattan distance times
//! the length weight plus the exact free-space bend estimate ([`min_bends`]) times the
//! bend weight, which is consistent.

mod bends;
pub(crate) mod one_bend;

use alloc::vec::Vec;

use heuristic_search::{Limits, Successors, a_star, heuristic_fn};

use crate::error::Error;
use crate::geom::{DirSet, Direction};
use crate::graph::{RoutingGraph, VertexId};
use crate::route::Route;
use crate::scene::{PortId, Scene};

pub use bends::min_bends;

/// What a route minimises.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Objective {
    /// The shortest route, and among those one with the fewest bends (exact).
    #[default]
    ShortestThenFewestBends,
    /// The route with the fewest bends, and among those the shortest (exact).
    FewestBendsThenShortest,
    /// `length * length_weight + bends * bend_weight`.
    Weighted {
        /// The cost of one unit of length.
        length: u64,
        /// The cost of one bend.
        bend: u64,
    },
}

/// The length and bend weights of an [`Objective`] on a given graph.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Weights {
    /// The cost of one unit of length.
    pub length: u64,
    /// The cost of one bend.
    pub bend: u64,
}

impl Objective {
    /// The weights that realise this objective exactly on `graph`.
    ///
    /// The lexicographic objectives weight the secondary term by one and the primary
    /// term by one more than a bound on the secondary term of any optimal route: the
    /// number of vertices for bends (a shortest route never revisits a vertex), twice
    /// the total edge length plus one for length (a route visits every
    /// `(vertex, heading)` state at most once).
    ///
    /// # Errors
    ///
    /// [`Error::CostOverflow`] if the bound does not fit into `u64`.
    pub fn weights(self, graph: &RoutingGraph) -> Result<Weights, Error> {
        match self {
            Self::Weighted { length, bend } => Ok(Weights { length, bend }),
            Self::ShortestThenFewestBends => Ok(Weights {
                length: u64::try_from(graph.len()).map_err(|_| Error::CostOverflow)? + 1,
                bend: 1,
            }),
            Self::FewestBendsThenShortest => {
                let total = graph.edges().try_fold(0_u64, |sum, (a, b)| {
                    sum.checked_add(graph.point(a).manhattan(graph.point(b)))
                });
                let bound = total
                    .and_then(|t| t.checked_mul(2))
                    .and_then(|t| t.checked_add(1))
                    .ok_or(Error::CostOverflow)?;

                Ok(Weights {
                    length: 1,
                    bend: bound,
                })
            }
        }
    }
}

/// Extra costs a caller charges on top of length and bends, such as congestion.
///
/// The extra costs must be non-negative (they are `u64`), which keeps the free-space
/// heuristic admissible.
pub trait EdgeCosts {
    /// The cost of moving from `from` in direction `dir` to its neighbour `to`, which
    /// replaces `weights.length * length` for that edge. Must be at least
    /// `length_weight * length` for the heuristic to stay admissible.
    fn edge(&self, from: VertexId, dir: Direction, to: VertexId, length_cost: u64) -> u64 {
        let _ = (from, dir, to);
        length_cost
    }

    /// An extra cost for passing through `at`, entering with heading `entry` and leaving
    /// with heading `exit`. `entry` is `None` at the source.
    fn pass(&self, at: VertexId, entry: Option<Direction>, exit: Direction) -> u64 {
        let _ = (at, entry, exit);
        0
    }
}

/// No extra cost: plain length and bends.
#[derive(Debug, Clone, Copy, Default)]
pub struct PlainCosts;

impl EdgeCosts for PlainCosts {}

/// A routing request between two graph vertices.
#[derive(Debug, Clone, Copy)]
pub struct Endpoints {
    /// The source vertex.
    pub source: VertexId,
    /// The directions the route may leave the source in.
    pub source_directions: DirSet,
    /// The target vertex.
    pub target: VertexId,
    /// The directions the target port may be left in; the route arrives moving opposite
    /// to one of them.
    pub target_directions: DirSet,
}

/// A search state: a vertex slot and the heading index, or [`NOT_MOVED`].
type State = (u32, u8);

/// The heading of the start state, before the first move.
const NOT_MOVED: u8 = 4;

/// The `(vertex, heading)` state graph of a routing graph.
struct StateGraph<'a, E> {
    /// The routing graph.
    graph: &'a RoutingGraph,
    /// The source vertex and its allowed directions.
    endpoints: Endpoints,
    /// The objective weights.
    weights: Weights,
    /// Extra costs.
    costs: &'a E,
}

impl<E: EdgeCosts> Successors for StateGraph<'_, E> {
    type State = State;
    type Action = Direction;
    type Cost = u64;

    fn successors(&self, state: &State, mut emit: impl FnMut(Direction, State, u64)) {
        let v = VertexId(state.0);
        let entry = (state.1 != NOT_MOVED).then(|| Direction::from_index(usize::from(state.1)));
        // Straight first, then right, then left, as in the paper.
        let options: [Option<Direction>; 4] = if let Some(h) = entry {
            [Some(h), Some(h.right()), Some(h.left()), None]
        } else {
            let mut options = [None; 4];
            for (slot, d) in options
                .iter_mut()
                .zip(self.endpoints.source_directions.iter())
            {
                *slot = Some(d);
            }

            options
        };

        let here = self.graph.point(v);
        for dir in options.into_iter().flatten() {
            let Some(w) = self.graph.neighbor(v, dir) else {
                continue;
            };

            let length = here.manhattan(self.graph.point(w));
            let length_cost = self.weights.length.saturating_mul(length);
            let bend = if entry.is_some_and(|h| h != dir) {
                self.weights.bend
            } else {
                0
            };
            let cost = self
                .costs
                .edge(v, dir, w, length_cost)
                .saturating_add(bend)
                .saturating_add(self.costs.pass(v, entry, dir));
            let index = u8::try_from(dir.index()).expect("four directions");
            emit(dir, (w.0, index), cost);
        }
    }
}

/// The admissible, consistent estimate of the remaining cost from `state`.
fn estimate(
    graph: &RoutingGraph,
    endpoints: &Endpoints,
    weights: Weights,
    state: State,
) -> Option<u64> {
    let here = graph.point(VertexId(state.0));
    let target = graph.point(endpoints.target);
    let arrivals = endpoints.target_directions.reversed();
    let bends = if state.1 == NOT_MOVED {
        if state.0 == endpoints.target.0 {
            0
        } else {
            endpoints
                .source_directions
                .iter()
                .filter_map(|d| min_bends(here, d, target, arrivals))
                .min()?
        }
    } else {
        min_bends(
            here,
            Direction::from_index(usize::from(state.1)),
            target,
            arrivals,
        )?
    };

    Some(
        weights
            .length
            .saturating_mul(here.manhattan(target))
            .saturating_add(weights.bend.saturating_mul(u64::from(bends))),
    )
}

/// The cheapest route between two vertices of `graph`, or `None` if there is none.
///
/// A route between equal vertices is the single vertex, whatever the directions.
#[must_use]
pub fn route_vertices<E: EdgeCosts>(
    graph: &RoutingGraph,
    endpoints: Endpoints,
    weights: Weights,
    costs: &E,
) -> Option<Route> {
    let states = StateGraph {
        graph,
        endpoints,
        weights,
        costs,
    };
    let arrivals = endpoints.target_directions.reversed();
    let target = endpoints.target.0;
    let goal = move |s: &State| {
        s.0 == target
            && (s.1 == NOT_MOVED || arrivals.contains(Direction::from_index(usize::from(s.1))))
    };
    let heuristic = heuristic_fn(|s: &State| estimate(graph, &endpoints, weights, *s));
    let result = a_star(
        &states,
        (endpoints.source.0, NOT_MOVED),
        &goal,
        heuristic,
        Limits::NONE,
    );
    let solution = result.into_solution()?;
    let vertices: Vec<VertexId> = solution.states.iter().map(|s| VertexId(s.0)).collect();

    Some(Route::from_vertices(graph, vertices, solution.cost))
}

/// The endpoints of a connector between two ports of `scene`.
///
/// # Errors
///
/// [`Error::UnknownPort`] if a port does not exist, [`Error::PortBlocked`] if it lies
/// inside an obstacle.
pub fn endpoints(scene: &Scene, source: PortId, target: PortId) -> Result<Endpoints, Error> {
    let vertex = |id: PortId| -> Result<(VertexId, DirSet), Error> {
        let port = scene.port(id).ok_or(Error::UnknownPort)?;
        let v = scene
            .port_vertex(id)
            .ok_or(Error::PortBlocked(port.point))?;

        Ok((v, port.directions))
    };
    let (source, source_directions) = vertex(source)?;
    let (target, target_directions) = vertex(target)?;

    Ok(Endpoints {
        source,
        source_directions,
        target,
        target_directions,
    })
}

/// Which single-connector search to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Search {
    /// A* over the orthogonal visibility graph (Wybrow et al. 2009). Optimal on the
    /// graph, which with the routing frame and port stubs matched brute force on the
    /// unit grid for inputs spaced at least a few units apart (see `docs/design.md`).
    #[default]
    Ovg,
    /// A* over the lazily generated 1-bend visibility graph (Marriott et al. 2014),
    /// exploring only obstacle-hugging routes, with [`Search::Ovg`] as the fallback when
    /// it finds nothing.
    ///
    /// Not exact in general: on a slit between touching boxes it returns length 38 with
    /// 4 bends where the optimum is 20 with 6. Routes are always valid and never cheaper
    /// than the optimum. In this lazy form it is also slower than [`Search::Ovg`].
    OneBend,
}

/// The optimal route for one connector between two ports of `scene`.
///
/// # Errors
///
/// [`Error::UnknownPort`] or [`Error::PortBlocked`] for a bad port,
/// [`Error::CostOverflow`] if the objective's weights overflow, and
/// [`Error::NoRoute`] (with connector index 0) if the ports are not connected.
///
/// # Examples
///
/// ```
/// use connector_routing::{DirSet, Objective, Point, Port, Rect, Scene, Search, route};
///
/// let mut scene = Scene::new();
/// scene.add_obstacle(Rect::new(0, 0, 10, 10))?;
/// let a = scene.add_port(Port::new(Point::new(-5, 5), DirSet::RIGHT))?;
/// let b = scene.add_port(Port::new(Point::new(15, 5), DirSet::LEFT))?;
///
/// let r = route(&scene, a, b, Objective::default(), Search::default())?;
/// assert_eq!(r.length, 30); // around the box: 5 + 5 + 10 + 5 + 5
/// assert_eq!(r.bends, 4);
/// # Ok::<(), connector_routing::Error>(())
/// ```
pub fn route(
    scene: &Scene,
    source: PortId,
    target: PortId,
    objective: Objective,
    search: Search,
) -> Result<Route, Error> {
    let endpoints = endpoints(scene, source, target)?;
    let weights = objective.weights(scene.graph())?;
    let found = match search {
        Search::Ovg => route_vertices(scene.graph(), endpoints, weights, &PlainCosts),
        Search::OneBend => one_bend::route(scene, endpoints, weights)
            .or_else(|| route_vertices(scene.graph(), endpoints, weights, &PlainCosts)),
    };

    found.ok_or(Error::NoRoute { connector: 0 })
}

/// The cost of a route of `length` and `bends` under `weights`, saturating.
#[must_use]
pub fn cost_of(weights: Weights, length: u64, bends: u32) -> u64 {
    weights
        .length
        .saturating_mul(length)
        .saturating_add(weights.bend.saturating_mul(u64::from(bends)))
}
