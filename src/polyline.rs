//! Incremental poly-line connector routing (Wybrow, Marriott and Stuckey 2005; Wybrow's
//! thesis 2008, chapter 5): shortest any-angle routes over the visibility graph of the
//! obstacle corners, kept up to date as obstacles come and go.
//!
//! The visibility graph joins every pair of nodes (obstacle corners and connector
//! endpoints) whose straight segment enters no obstacle interior. Every pair that is not
//! visible is kept in the *invisibility graph*, associated with one obstacle that blocks
//! it, so removing an obstacle retests only the pairs it blocked. Routes are A* searches
//! with the Euclidean lower bound. The cost of a route is
//! `length + segment_penalty * bends + crossing_penalty * crossings` (the thesis'
//! formula without the angle term, which needs floating point).
//!
//! Lengths are fixed-point (`2^8` units per coordinate unit) and rounded up per segment,
//! with the heuristic rounded down, so A* stays admissible and consistent; the geometry is
//! exact.

use alloc::collections::{BTreeMap, BTreeSet};
use alloc::vec::Vec;

use heuristic_search::{Limits, Successors, a_star, heuristic_fn};

use crate::error::Error;
use crate::geom::{Point, Rect};
use crate::scene::ObstacleId;
use crate::sleeve::separated;

/// Fixed-point scale of lengths: costs are in `1 / 2^SHIFT` coordinate units.
const SHIFT: u32 = 8;

/// The fixed-point length of a segment, rounded up.
fn length_up(a: Point, b: Point) -> u64 {
    let squared = a.distance_squared(b) << (2 * SHIFT);
    let root = squared.isqrt();
    let root = if root * root == squared {
        root
    } else {
        root + 1
    };

    u64::try_from(root).unwrap_or(u64::MAX)
}

/// The fixed-point length of a segment, rounded down.
fn length_down(a: Point, b: Point) -> u64 {
    u64::try_from((a.distance_squared(b) << (2 * SHIFT)).isqrt()).unwrap_or(u64::MAX)
}

/// Parameters of poly-line routing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Config {
    /// The cost of a bend, in coordinate units.
    pub segment_penalty: u64,
    /// The cost of crossing a segment of another committed route, in coordinate units.
    pub crossing_penalty: u64,
}

/// A connector managed by a [`PolylineRouter`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ConnectorId(u32);

/// One managed connector.
#[derive(Debug, Clone)]
struct Managed {
    /// The source point.
    source: Point,
    /// The target point.
    target: Point,
    /// The committed route, if any.
    route: Option<Vec<Point>>,
    /// The fixed-point cost of the committed route.
    cost: u64,
}

/// What an obstacle change did to the managed connectors.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Change {
    /// Connectors whose route now enters an obstacle; they must be rerouted.
    pub invalidated: Vec<ConnectorId>,
    /// Connectors whose route may now have a cheaper alternative.
    pub may_improve: Vec<ConnectorId>,
}

/// Obstacles, connectors and the incrementally maintained visibility graph.
///
/// # Examples
///
/// ```
/// use connector_routing::{Point, Rect, check, polyline::{Config, PolylineRouter}};
///
/// let mut router = PolylineRouter::new(Config::default());
/// let block = router.add_obstacle(Rect::new(10, -10, 20, 10))?.0;
/// let c = router.add_connector(Point::new(0, 0), Point::new(30, 0))?;
/// let route = router.route(c).expect("a route").to_vec();
/// assert_eq!(route.len(), 4); // over two corners of the box
///
/// // Removing the box reports the connector as improvable; rerouting straightens it.
/// let change = router.remove_obstacle(block)?;
/// assert_eq!(change.may_improve, [c]);
/// router.reroute(c);
/// assert_eq!(router.route(c).unwrap(), [Point::new(0, 0), Point::new(30, 0)]);
/// # Ok::<(), connector_routing::Error>(())
/// ```
#[derive(Debug, Clone, Default)]
pub struct PolylineRouter {
    /// The routing parameters.
    config: Config,
    /// The obstacles by slot; `None` for removed ones.
    obstacles: Vec<Option<Rect>>,
    /// How many corners and endpoints sit at each node.
    nodes: BTreeMap<Point, u32>,
    /// The visibility graph: the visible nodes of each node.
    visible: BTreeMap<Point, BTreeSet<Point>>,
    /// The invisibility graph: the node pairs each obstacle blocks.
    blocked_by: BTreeMap<u32, BTreeSet<(Point, Point)>>,
    /// The managed connectors by slot.
    connectors: Vec<Option<Managed>>,
}

impl PolylineRouter {
    /// Creates an empty router.
    #[must_use]
    pub fn new(config: Config) -> Self {
        Self {
            config,
            ..Self::default()
        }
    }

    /// The live obstacles.
    fn rects(&self) -> impl Iterator<Item = (u32, Rect)> + '_ {
        self.obstacles
            .iter()
            .enumerate()
            .filter_map(|(i, r)| r.map(|r| (u32::try_from(i).expect("few obstacles"), r)))
    }

    /// The first obstacle that blocks the segment `a b`, if any.
    fn blocker(&self, a: Point, b: Point) -> Option<u32> {
        self.rects()
            .find(|(_, r)| !separated(a, b, r))
            .map(|(i, _)| i)
    }

    /// Whether `p` lies in some obstacle interior.
    fn inside(&self, p: Point) -> bool {
        self.rects().any(|(_, r)| r.interior_contains(p))
    }

    /// Adds `p` as a node (or counts one more use), connecting it to every node.
    fn add_node(&mut self, p: Point) {
        let count = self.nodes.entry(p).or_default();
        *count += 1;
        if *count > 1 {
            return;
        }

        let others: Vec<Point> = self.nodes.keys().copied().filter(|&q| q != p).collect();
        for q in others {
            self.classify(p, q);
        }
    }

    /// Puts the pair `p q` into the visibility or the invisibility graph.
    fn classify(&mut self, p: Point, q: Point) {
        let key = (p.min(q), p.max(q));
        if let Some(o) = self.blocker(p, q) {
            self.blocked_by.entry(o).or_default().insert(key);
        } else {
            self.visible.entry(p).or_default().insert(q);
            self.visible.entry(q).or_default().insert(p);
        }
    }

    /// Removes one use of node `p`, dropping it and its edges when unused.
    fn remove_node(&mut self, p: Point) {
        let Some(count) = self.nodes.get_mut(&p) else {
            return;
        };
        *count -= 1;
        if *count > 0 {
            return;
        }

        self.nodes.remove(&p);
        for q in self.visible.remove(&p).unwrap_or_default() {
            if let Some(set) = self.visible.get_mut(&q) {
                set.remove(&p);
            }
        }

        for set in self.blocked_by.values_mut() {
            set.retain(|&(a, b)| a != p && b != p);
        }
    }

    /// Adds an obstacle: moves the visibility edges it crosses to the invisibility
    /// graph, adds its corners, and reports the connectors whose routes it invalidates.
    ///
    /// # Errors
    ///
    /// [`Error::CoordinateOutOfRange`] beyond [`MAX_COORD`](crate::MAX_COORD).
    ///
    /// # Panics
    ///
    /// Panics after `2^32` obstacles.
    pub fn add_obstacle(&mut self, rect: Rect) -> Result<(ObstacleId, Change), Error> {
        for p in [rect.min, rect.max] {
            if !p.is_in_range() {
                return Err(Error::CoordinateOutOfRange(p));
            }
        }

        let id = u32::try_from(self.obstacles.len()).expect("few obstacles");
        self.obstacles.push(Some(rect));
        let crossing: Vec<(Point, Point)> = self
            .visible
            .iter()
            .flat_map(|(&p, set)| set.iter().filter(move |&&q| p < q).map(move |&q| (p, q)))
            .filter(|&(p, q)| !separated(p, q, &rect))
            .collect();
        for &(p, q) in &crossing {
            self.visible.get_mut(&p).expect("edge").remove(&q);
            self.visible.get_mut(&q).expect("edge").remove(&p);
            self.blocked_by.entry(id).or_default().insert((p, q));
        }

        for corner in rect.corners() {
            self.add_node(corner);
        }

        // Nodes now inside the obstacle stay nodes; their edges all cross it.
        let invalidated = self
            .connector_ids()
            .filter(|&c| {
                self.route(c)
                    .is_some_and(|r| r.windows(2).any(|w| !separated(w[0], w[1], &rect)))
            })
            .collect();

        Ok((
            ObstacleId(id),
            Change {
                invalidated,
                may_improve: Vec::new(),
            },
        ))
    }

    /// Removes an obstacle: drops its corners, retests the pairs it blocked, and reports
    /// the connectors that may now have a cheaper route (their lower bound through the
    /// freed area is below their cost) or a route at all.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownObstacle`] if `id` names no obstacle.
    pub fn remove_obstacle(&mut self, id: ObstacleId) -> Result<Change, Error> {
        let rect = self
            .obstacles
            .get_mut(id.index())
            .and_then(Option::take)
            .ok_or(Error::UnknownObstacle)?;
        for corner in rect.corners() {
            self.remove_node(corner);
        }

        for (p, q) in self.blocked_by.remove(&id.0).unwrap_or_default() {
            if self.nodes.contains_key(&p) && self.nodes.contains_key(&q) {
                self.classify(p, q);
            }
        }

        // A connector without a route may have become routable.
        let may_improve = self
            .connector_ids()
            .filter(|&c| {
                let m = self.managed(c);
                m.route.is_none() || lower_bound_through(m.source, m.target, &rect) < m.cost
            })
            .collect();

        Ok(Change {
            invalidated: Vec::new(),
            may_improve,
        })
    }

    /// Adds a connector between two points and routes it.
    ///
    /// # Errors
    ///
    /// [`Error::CoordinateOutOfRange`] beyond [`MAX_COORD`](crate::MAX_COORD),
    /// [`Error::PortBlocked`] for an endpoint inside an obstacle.
    ///
    /// # Panics
    ///
    /// Panics after `2^32` connectors.
    pub fn add_connector(&mut self, source: Point, target: Point) -> Result<ConnectorId, Error> {
        for p in [source, target] {
            if !p.is_in_range() {
                return Err(Error::CoordinateOutOfRange(p));
            }

            if self.inside(p) {
                return Err(Error::PortBlocked(p));
            }
        }

        self.add_node(source);
        self.add_node(target);
        let id = ConnectorId(u32::try_from(self.connectors.len()).expect("few connectors"));
        self.connectors.push(Some(Managed {
            source,
            target,
            route: None,
            cost: u64::MAX,
        }));
        self.reroute(id);

        Ok(id)
    }

    /// Removes a connector.
    pub fn remove_connector(&mut self, id: ConnectorId) {
        if let Some(m) = self
            .connectors
            .get_mut(id.0 as usize)
            .and_then(Option::take)
        {
            self.remove_node(m.source);
            self.remove_node(m.target);
        }
    }

    /// The committed route of a connector, if it has one.
    #[must_use]
    pub fn route(&self, id: ConnectorId) -> Option<&[Point]> {
        self.connectors
            .get(id.0 as usize)?
            .as_ref()?
            .route
            .as_deref()
    }

    /// Recomputes and commits the cheapest route of a connector; returns whether one
    /// exists.
    pub fn reroute(&mut self, id: ConnectorId) -> bool {
        let Some(m) = self.connectors.get(id.0 as usize).and_then(Option::as_ref) else {
            return false;
        };
        let (source, target) = (m.source, m.target);
        let found = self.search(id, source, target);
        let routed = found.is_some();
        if let Some(slot) = self.connectors[id.0 as usize].as_mut() {
            (slot.route, slot.cost) = match found {
                Some((route, cost)) => (Some(route), cost),
                None => (None, u64::MAX),
            };
        }

        routed
    }

    /// The visibility graph's edges, each once as `(a, b)` with `a < b`, sorted.
    #[must_use]
    pub fn visibility_edges(&self) -> Vec<(Point, Point)> {
        self.visible
            .iter()
            .flat_map(|(&p, set)| set.iter().filter(move |&&q| p < q).map(move |&q| (p, q)))
            .collect()
    }

    /// The fixed-point cost (`2^8` per unit) of a connector's committed route.
    #[must_use]
    pub fn cost(&self, id: ConnectorId) -> Option<u64> {
        let m = self.connectors.get(id.0 as usize)?.as_ref()?;

        m.route.as_ref().map(|_| m.cost)
    }

    /// The live connector ids.
    fn connector_ids(&self) -> impl Iterator<Item = ConnectorId> + '_ {
        self.connectors
            .iter()
            .enumerate()
            .filter(|(_, c)| c.is_some())
            .map(|(i, _)| ConnectorId(u32::try_from(i).expect("few connectors")))
    }

    /// The managed connector `id`.
    fn managed(&self, id: ConnectorId) -> &Managed {
        self.connectors[id.0 as usize]
            .as_ref()
            .expect("live connector")
    }

    /// A* over the visibility graph from `source` to `target`, with crossing costs
    /// against the other committed routes.
    fn search(&self, id: ConnectorId, source: Point, target: Point) -> Option<(Vec<Point>, u64)> {
        let others: Vec<(Point, Point)> = self
            .connector_ids()
            .filter(|&c| c != id)
            .filter_map(|c| self.route(c))
            .flat_map(|r| r.windows(2).map(|w| (w[0], w[1])).collect::<Vec<_>>())
            .collect();
        let graph = Visibility {
            router: self,
            source,
            others,
        };
        let heuristic = heuristic_fn(|p: &Point| Some(length_down(*p, target)));
        let result = a_star(
            &graph,
            source,
            &|p: &Point| *p == target,
            heuristic,
            Limits::NONE,
        );
        let solution = result.into_solution()?;

        Some((solution.states, solution.cost))
    }
}

/// The visibility graph of a router for one search.
struct Visibility<'a> {
    /// The router.
    router: &'a PolylineRouter,
    /// The search's source; leaving it is not a bend.
    source: Point,
    /// Segments of the other committed routes, for crossing costs.
    others: Vec<(Point, Point)>,
}

impl Successors for Visibility<'_> {
    type State = Point;
    type Action = ();
    type Cost = u64;

    fn successors(&self, state: &Point, mut emit: impl FnMut((), Point, u64)) {
        let Some(set) = self.router.visible.get(state) else {
            return;
        };
        let scale = 1_u64 << SHIFT;
        let bend = if *state == self.source {
            0
        } else {
            self.router.config.segment_penalty.saturating_mul(scale)
        };
        // Nodes inside obstacles have no visible pairs, so every successor is free.
        for &next in set {
            let crossings = self
                .others
                .iter()
                .filter(|&&(a, b)| properly_cross(*state, next, a, b))
                .count();
            let crossing = self
                .router
                .config
                .crossing_penalty
                .saturating_mul(scale)
                .saturating_mul(u64::try_from(crossings).unwrap_or(u64::MAX));
            emit(
                (),
                next,
                length_up(*state, next)
                    .saturating_add(bend)
                    .saturating_add(crossing),
            );
        }
    }
}

/// Whether segments `a b` and `p q` cross at a point interior to both.
fn properly_cross(a: Point, b: Point, p: Point, q: Point) -> bool {
    use crate::sleeve::cdt::orient;
    let d1 = orient(a, b, p).signum();
    let d2 = orient(a, b, q).signum();
    let d3 = orient(p, q, a).signum();
    let d4 = orient(p, q, b).signum();

    d1 * d2 < 0 && d3 * d4 < 0
}

/// A fixed-point lower bound, rounded down, on the length of any path from `u` to `v`
/// through `region`: the larger of the straight distance and the distances of `u` and
/// `v` to the region (a weaker but simpler form of the exact bound of Wybrow et al.
/// 2005, section 2.3).
fn lower_bound_through(u: Point, v: Point, region: &Rect) -> u64 {
    let clamp = |p: Point| {
        Point::new(
            p.x.clamp(region.min.x, region.max.x),
            p.y.clamp(region.min.y, region.max.y),
        )
    };
    let via = length_down(u, clamp(u)).saturating_add(length_down(v, clamp(v)));

    via.max(length_down(u, v)).saturating_sub(2)
}

#[cfg(test)]
mod tests {
    //! Unit tests for the fixed-point lengths.

    use super::*;

    /// Rounding up and down bracket the exact length.
    #[test]
    fn lengths_bracket() {
        let (a, b) = (Point::new(0, 0), Point::new(3, 4));
        assert_eq!(length_up(a, b), 5 << SHIFT);
        assert_eq!(length_down(a, b), 5 << SHIFT);
        let c = Point::new(1, 1);
        assert_eq!(length_up(a, c), length_down(a, c) + 1);
    }
}
