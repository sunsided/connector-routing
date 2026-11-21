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
use crate::sleeve::cdt::orient;
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// The cost of a bend, in coordinate units (the thesis' `alpha`).
    pub segment_penalty: u64,
    /// The angle penalty `beta` of the thesis: a bend that turns by `theta` costs
    /// `beta * a * ln(a + 1) / 10` coordinate units, with `a = 10 theta / pi`.
    pub angle_penalty: u64,
    /// The cost of crossing a segment of another committed route, in coordinate units
    /// (the thesis' `gamma`).
    pub crossing_penalty: u64,
    /// The hidden-region pruning of Wybrow et al. (2005, section 2): at an obstacle
    /// corner, a route only continues around the obstacle. Exact for shortest paths;
    /// applied only when the segment and angle penalties are zero (with them, a route
    /// may turn the other way to save a bend).
    pub hidden_region_pruning: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            segment_penalty: 0,
            angle_penalty: 0,
            crossing_penalty: 0,
            hidden_region_pruning: true,
        }
    }
}

/// `cos((k + 1/2) pi / 100)` scaled by `2^15`, for `k = 0..100`: the boundaries between
/// the 101 angle classes `a = k / 10`.
const COS_BOUNDS: [i64; 100] = [
    32764, 32732, 32667, 32570, 32441, 32280, 32087, 31863, 31607, 31319, 31001, 30653, 30274,
    29865, 29427, 28959, 28463, 27939, 27388, 26809, 26204, 25573, 24917, 24236, 23532, 22804,
    22053, 21281, 20488, 19675, 18842, 17990, 17121, 16235, 15333, 14416, 13485, 12540, 11583,
    10614, 9635, 8647, 7650, 6645, 5634, 4617, 3596, 2571, 1544, 515, -515, -1544, -2571, -3596,
    -4617, -5634, -6645, -7650, -8647, -9635, -10614, -11583, -12540, -13485, -14416, -15333,
    -16235, -17121, -17990, -18842, -19675, -20488, -21281, -22053, -22804, -23532, -24236, -24917,
    -25573, -26204, -26809, -27388, -27939, -28463, -28959, -29427, -29865, -30274, -30653, -31001,
    -31319, -31607, -31863, -32087, -32280, -32441, -32570, -32667, -32732, -32764,
];

/// `a ln(a + 1) / 10` scaled by `2^16`, for `a = k / 10`, `k = 0..=100`.
const ANGLE_COST: [u64; 101] = [
    0, 62, 239, 516, 882, 1329, 1848, 2434, 3082, 3786, 4543, 5349, 6201, 7096, 8032, 9008, 10019,
    11066, 12146, 13258, 14400, 15571, 16770, 17996, 19248, 20525, 21826, 23151, 24497, 25866,
    27256, 28666, 30096, 31545, 33013, 34500, 36004, 37526, 39064, 40619, 42190, 43777, 45380,
    46997, 48629, 50275, 51935, 53610, 55297, 56998, 58712, 60439, 62178, 63930, 65693, 67469,
    69256, 71054, 72864, 74685, 76516, 78359, 80211, 82075, 83948, 85832, 87725, 89628, 91541,
    93463, 95395, 97336, 99286, 101_244, 103_212, 105_189, 107_174, 109_167, 111_169, 113_179,
    115_198, 117_224, 119_259, 121_301, 123_352, 125_410, 127_475, 129_548, 131_629, 133_717,
    135_812, 137_914, 140_024, 142_141, 144_264, 146_395, 148_532, 150_676, 152_827, 154_984,
    157_148,
];

/// The angle class `k` (`a = k / 10`, `0..=100`) of the turn at `b` from `a` to `c`,
/// from the cosine of the turning angle (integer square root, then the class table).
#[expect(
    clippy::many_single_char_names,
    reason = "the usual names of a turn and its two vectors"
)]
fn angle_class(a: Point, b: Point, c: Point) -> usize {
    let u = (
        i128::from(b.x) - i128::from(a.x),
        i128::from(b.y) - i128::from(a.y),
    );
    let v = (
        i128::from(c.x) - i128::from(b.x),
        i128::from(c.y) - i128::from(b.y),
    );
    let dot = u.0 * v.0 + u.1 * v.1;
    let norms = u128::try_from(u.0 * u.0 + u.1 * u.1).unwrap_or(0)
        * u128::try_from(v.0 * v.0 + v.1 * v.1).unwrap_or(0);
    // Scale the norms up as far as `u128` allows, so the root keeps its precision for
    // short segments.
    let k = (norms.leading_zeros() / 2).saturating_sub(1).min(30);
    let denominator = i128::try_from((norms << (2 * k)).isqrt())
        .unwrap_or(i128::MAX)
        .max(1);
    let cos = i64::try_from((dot << (15 + k)) / denominator).unwrap_or(0);

    // The first boundary the cosine falls below is the class.
    COS_BOUNDS
        .iter()
        .position(|&bound| cos > bound)
        .unwrap_or(100)
}

/// The fixed-point angle penalty of the turn at `b` from `a` to `c`.
fn angle_cost(beta: u64, a: Point, b: Point, c: Point) -> u64 {
    beta.saturating_mul(ANGLE_COST[angle_class(a, b, c)]) >> (16 - SHIFT)
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
        // An obstacle added after the connector may cover an endpoint; then no route is
        // valid, not even the single point of a connector from a point to itself.
        if self.inside(source) || self.inside(target) {
            return None;
        }

        let others: Vec<(Point, Point)> = self
            .connector_ids()
            .filter(|&c| c != id)
            .filter_map(|c| self.route(c))
            .flat_map(|r| r.windows(2).map(|w| (w[0], w[1])).collect::<Vec<_>>())
            .collect();
        // Exact only for pure shortest paths: with bend costs, a route may take a turn
        // the pruning forbids to save a bend (found by the property test).
        let prune = self.config.hidden_region_pruning
            && self.config.angle_penalty == 0
            && self.config.segment_penalty == 0;
        let graph = Visibility {
            router: self,
            source,
            target,
            others,
            corners: if prune {
                self.single_corners()
            } else {
                BTreeMap::new()
            },
        };
        let heuristic = heuristic_fn(|s: &(Point, Point)| Some(length_down(s.0, target)));
        let result = a_star(
            &graph,
            (source, source),
            &|s: &(Point, Point)| s.0 == target,
            heuristic,
            Limits::NONE,
        );
        let solution = result.into_solution()?;

        Some((solution.states.iter().map(|s| s.0).collect(), solution.cost))
    }

    /// The obstacle corners that belong to exactly one obstacle, with twice its centre.
    fn single_corners(&self) -> BTreeMap<Point, Option<(i64, i64)>> {
        let mut corners: BTreeMap<Point, Option<(i64, i64)>> = BTreeMap::new();
        for (_, r) in self.rects() {
            let centre = (
                i64::from(r.min.x) + i64::from(r.max.x),
                i64::from(r.min.y) + i64::from(r.max.y),
            );
            for c in r.corners() {
                corners
                    .entry(c)
                    .and_modify(|slot| *slot = None)
                    .or_insert(Some(centre));
            }
        }

        corners
    }
}

/// The visibility graph of a router for one search. States are `(node, previous
/// node)`, the previous node equal to the node at the source, so bends and their angles
/// can be charged.
struct Visibility<'a> {
    /// The router.
    router: &'a PolylineRouter,
    /// The search's source; leaving it is not a bend.
    source: Point,
    /// The search's target, never pruned.
    target: Point,
    /// Segments of the other committed routes, for crossing costs.
    others: Vec<(Point, Point)>,
    /// For hidden-region pruning: obstacle corners of exactly one obstacle, with twice
    /// its centre (`None` for corners shared by several). Empty when pruning is off.
    corners: BTreeMap<Point, Option<(i64, i64)>>,
}

impl Visibility<'_> {
    /// Whether a route at obstacle corner `at`, coming from `prev`, may continue to
    /// `next`: it must go straight on or turn towards the obstacle, around it. Any other
    /// turn could be cut short, so no shortest route takes it.
    fn keeps(&self, prev: Point, at: Point, next: Point) -> bool {
        let Some(Some((cx, cy))) = self.corners.get(&at) else {
            return true;
        };
        if at == prev || at == self.source || at == self.target {
            return true;
        }

        let turn = orient(prev, at, next);
        if turn == 0 {
            return straight_on(prev, at, next);
        }

        // Twice the cross product of the incoming direction with the centre.
        let (dx, dy) = (
            i128::from(at.x) - i128::from(prev.x),
            i128::from(at.y) - i128::from(prev.y),
        );
        let side = dx * (i128::from(*cy) - 2 * i128::from(prev.y))
            - dy * (i128::from(*cx) - 2 * i128::from(prev.x));

        side.signum() == turn.signum()
    }
}

impl Successors for Visibility<'_> {
    type State = (Point, Point);
    type Action = ();
    type Cost = u64;

    fn successors(&self, state: &(Point, Point), mut emit: impl FnMut((), (Point, Point), u64)) {
        let (here, prev) = *state;
        let Some(set) = self.router.visible.get(&here) else {
            return;
        };
        let scale = 1_u64 << SHIFT;
        let config = self.router.config;
        // Nodes inside obstacles have no visible pairs, so every successor is free.
        for &next in set {
            if next == prev || !self.keeps(prev, here, next) {
                continue;
            }

            let turning = prev != here && !straight_on(prev, here, next);
            let bend = if turning {
                config
                    .segment_penalty
                    .saturating_mul(scale)
                    .saturating_add(angle_cost(config.angle_penalty, prev, here, next))
            } else {
                0
            };

            let crossings = self
                .others
                .iter()
                .filter(|&&(a, b)| properly_cross(here, next, a, b))
                .count();
            let crossing = self
                .router
                .config
                .crossing_penalty
                .saturating_mul(scale)
                .saturating_mul(u64::try_from(crossings).unwrap_or(u64::MAX));
            emit(
                (),
                (next, here),
                length_up(here, next)
                    .saturating_add(bend)
                    .saturating_add(crossing),
            );
        }
    }
}

/// Whether `c` continues the direction from `a` to `b` without turning.
fn straight_on(a: Point, b: Point, c: Point) -> bool {
    let forward = (i128::from(b.x) - i128::from(a.x)) * (i128::from(c.x) - i128::from(b.x))
        + (i128::from(b.y) - i128::from(a.y)) * (i128::from(c.y) - i128::from(b.y));

    orient(a, b, c) == 0 && forward > 0
}

/// Whether segments `a b` and `p q` cross at a point interior to both.
fn properly_cross(a: Point, b: Point, p: Point, q: Point) -> bool {
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

    /// Angle classes: straight 0, a right angle 50, a reversal 100, 45 degrees 25.
    #[test]
    fn angle_classes() {
        let o = Point::new(0, 0);
        let p = Point::new(10, 0);
        assert_eq!(angle_class(o, p, Point::new(20, 0)), 0);
        assert_eq!(angle_class(o, p, Point::new(10, 7)), 50);
        assert_eq!(angle_class(o, p, Point::new(3, 0)), 100);
        assert_eq!(angle_class(o, p, Point::new(15, 5)), 25);
        assert_eq!(angle_class(o, Point::new(10, 1), Point::new(20, 1)), 3);
    }

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
