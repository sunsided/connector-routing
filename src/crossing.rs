//! Crossings between routes in the routing graph, and crossing reduction
//! (arXiv 2309.01671; crossing detection on shared paths as in Dwyer et al. 2006).
//!
//! Two routes that share graph edges or vertices may or may not have to cross,
//! depending on how they enter and leave what they share. [`required_crossings`] counts
//! the crossings any drawing of the two paths must have: one per shared stretch whose
//! two ends put the routes on different sides, and one per shared vertex they pass
//! straight through in perpendicular directions. Ordering and nudging realise exactly
//! these.
//!
//! [`points`] and [`iter_points`] list where finished routes cross, for drawing hops
//! ("overpasses") at the crossings.

use alloc::collections::{BTreeMap, BTreeSet};
use alloc::vec::Vec;
use core::cmp::Ordering;

use crate::check::crossing_point;
use crate::geom::{Direction, Point};
use crate::graph::{RoutingGraph, VertexId};
use crate::route::Route;
use crate::search::{Weights, cost_of};

/// The rank of a turn from `heading` into `next`: 0 left, 1 straight, 2 right.
fn turn_rank(heading: Direction, next: Direction) -> u8 {
    if next == heading.left() {
        0
    } else if next == heading {
        1
    } else {
        2
    }
}

/// The direction from vertex `a` to its neighbour `b`.
fn dir(graph: &RoutingGraph, a: VertexId, b: VertexId) -> Direction {
    Direction::between(graph.point(a), graph.point(b)).expect("consecutive vertices are aligned")
}

/// A path with lookup structures: the first position of every vertex and its edge set.
struct Indexed<'a> {
    /// The vertices.
    path: &'a [VertexId],
    /// The first position of each vertex.
    position: BTreeMap<VertexId, usize>,
    /// Every edge as an ordered pair.
    edges: BTreeSet<(VertexId, VertexId)>,
}

impl<'a> Indexed<'a> {
    /// Indexes `path`.
    fn new(path: &'a [VertexId]) -> Self {
        let mut position = BTreeMap::new();
        for (i, &v) in path.iter().enumerate() {
            position.entry(v).or_insert(i);
        }

        let edges = path
            .windows(2)
            .map(|e| (e[0].min(e[1]), e[0].max(e[1])))
            .collect();

        Self {
            path,
            position,
            edges,
        }
    }

    /// The first position of `v`.
    fn position(&self, v: VertexId) -> Option<usize> {
        self.position.get(&v).copied()
    }

    /// Whether the path uses the edge between `u` and `w`.
    fn has_edge(&self, u: VertexId, w: VertexId) -> bool {
        self.edges.contains(&(u.min(w), u.max(w)))
    }
}

/// The neighbour of `path[at]` in `path` other than `other`, if any.
fn beyond(path: &[VertexId], at: usize, other: VertexId) -> Option<VertexId> {
    let before = at.checked_sub(1).map(|i| path[i]);
    let after = path.get(at + 1).copied();
    if before == Some(other) { after } else { before }
}

/// The side of `a` relative to `b` where they leave a shared stretch at `end`, coming
/// from `prev` (both paths contain the edge `prev end`): `Less` if `a` turns further
/// left. `None` if either path ends there.
fn side_at(
    graph: &RoutingGraph,
    a: &Indexed<'_>,
    b: &Indexed<'_>,
    prev: VertexId,
    end: VertexId,
) -> Option<Ordering> {
    let heading = dir(graph, prev, end);
    let next_a = beyond(a.path, a.position(end)?, prev)?;
    let next_b = beyond(b.path, b.position(end)?, prev)?;

    Some(
        turn_rank(heading, dir(graph, end, next_a))
            .cmp(&turn_rank(heading, dir(graph, end, next_b))),
    )
}

/// The number of crossings any drawing of paths `a` and `b` must have.
///
/// # Examples
///
/// ```
/// use connector_routing::{Point, Scene, Port, crossing};
///
/// // Two free ports per route on a plus shape: the routes must cross once.
/// let ports = [(0, 5), (10, 5), (5, 0), (5, 10)].map(|(x, y)| Port::free(Point::new(x, y)));
/// let scene = Scene::build([], ports)?;
/// let g = scene.graph();
/// let line = |a: (i32, i32), b: (i32, i32)| -> Vec<_> {
///     let (mut v, end) = (g.vertex_at(Point::new(a.0, a.1)).unwrap(), Point::new(b.0, b.1));
///     let dir = connector_routing::Direction::between(Point::new(a.0, a.1), end).unwrap();
///     let mut path = vec![v];
///     while g.point(v) != end {
///         v = g.neighbor(v, dir).unwrap();
///         path.push(v);
///     }
///     path
/// };
/// assert_eq!(crossing::required_crossings(g, &line((0, 5), (10, 5)), &line((5, 0), (5, 10))), 1);
/// # Ok::<(), connector_routing::Error>(())
/// ```
#[must_use]
pub fn required_crossings(graph: &RoutingGraph, a: &[VertexId], b: &[VertexId]) -> u32 {
    indexed_crossings(graph, &Indexed::new(a), &Indexed::new(b))
}

/// [`required_crossings`] on indexed paths.
fn indexed_crossings(graph: &RoutingGraph, ia: &Indexed<'_>, ib: &Indexed<'_>) -> u32 {
    let a = ia.path;
    let mut count = 0;
    let mut k = 0;
    while k + 1 < a.len() {
        if !ib.has_edge(a[k], a[k + 1]) {
            count += u32::from(k > 0 && straight_crossing(graph, a, ib, k));
            k += 1;
            continue;
        }

        // A shared stretch from a[k] to a[end].
        let mut end = k + 1;
        while end + 1 < a.len() && ib.has_edge(a[end], a[end + 1]) {
            end += 1;
        }

        let at_end = side_at(graph, ia, ib, a[end - 1], a[end]);
        let at_start = side_at(graph, ia, ib, a[k + 1], a[k]).map(Ordering::reverse);
        if let (Some(s), Some(e)) = (at_start, at_end) {
            count += u32::from(s != e);
        }

        k = end;
    }

    count
}

/// Whether `a` passes straight through `a[k]` while `b` passes straight through it in
/// the perpendicular direction, sharing no edge there.
fn straight_crossing(graph: &RoutingGraph, a: &[VertexId], ib: &Indexed<'_>, k: usize) -> bool {
    let b = ib.path;
    let Some(j) = ib.position(a[k]) else {
        return false;
    };
    if j == 0 || j + 1 >= b.len() || k + 1 >= a.len() {
        return false;
    }

    let da = (dir(graph, a[k - 1], a[k]), dir(graph, a[k], a[k + 1]));
    let db = (dir(graph, b[j - 1], b[j]), dir(graph, b[j], b[j + 1]));

    da.0 == da.1 && db.0 == db.1 && da.0.axis() != db.0.axis()
}

/// The total number of required crossings over all pairs of routes.
#[must_use]
pub fn total_crossings(graph: &RoutingGraph, routes: &[Route]) -> u32 {
    let indexed: Vec<Indexed<'_>> = routes.iter().map(|r| Indexed::new(&r.vertices)).collect();
    sharing_pairs(routes)
        .into_iter()
        .map(|(i, j)| indexed_crossings(graph, &indexed[i], &indexed[j]))
        .sum()
}

/// The pairs `(i, j)`, `i < j`, of routes that share a vertex; other pairs cannot cross.
fn sharing_pairs(routes: &[Route]) -> BTreeSet<(usize, usize)> {
    let mut users: BTreeMap<VertexId, Vec<usize>> = BTreeMap::new();
    for (i, r) in routes.iter().enumerate() {
        for &v in &r.vertices {
            let list = users.entry(v).or_default();
            if list.last() != Some(&i) {
                list.push(i);
            }
        }
    }

    let mut pairs = BTreeSet::new();
    for list in users.values() {
        for (k, &i) in list.iter().enumerate() {
            for &j in &list[k + 1..] {
                pairs.insert((i.min(j), i.max(j)));
            }
        }
    }

    pairs
}

/// Reduces crossings: whenever two routes must cross more than once, the section of one
/// between their first and last shared vertex is replaced by the corresponding section
/// of the other, if the result is a simple path and strictly lowers that route's total
/// crossing count. Repeats until nothing improves or `max_rounds` is reached.
///
/// Returns the number of replacements. Replaced routes are rebuilt with their cost under
/// `weights`; they may be longer.
pub fn reduce_crossings(
    graph: &RoutingGraph,
    routes: &mut [Route],
    weights: Weights,
    max_rounds: u32,
) -> u32 {
    let mut replaced = 0;
    for _ in 0..max_rounds {
        let changes = round(graph, routes);
        if changes.is_empty() {
            break;
        }

        for (i, vertices) in changes {
            let mut route = Route::from_vertices(graph, vertices, 0);
            route.cost = cost_of(weights, route.length, route.bends);
            routes[i] = route;
            replaced += 1;
        }
    }

    replaced
}

/// One round of crossing reduction over all pairs of routes sharing a vertex. A route
/// changes at most once per round; pairs with a changed route wait for the next round.
fn round(graph: &RoutingGraph, routes: &[Route]) -> Vec<(usize, Vec<VertexId>)> {
    let indexed: Vec<Indexed<'_>> = routes.iter().map(|r| Indexed::new(&r.vertices)).collect();
    let mut users: BTreeMap<VertexId, Vec<usize>> = BTreeMap::new();
    for (i, r) in routes.iter().enumerate() {
        for &v in &r.vertices {
            let list = users.entry(v).or_default();
            if list.last() != Some(&i) {
                list.push(i);
            }
        }
    }

    let mut dirty = BTreeSet::new();
    let mut changes = Vec::new();
    for (i, j) in sharing_pairs(routes) {
        for (x, y) in [(i, j), (j, i)] {
            if dirty.contains(&x) || dirty.contains(&y) {
                continue;
            }

            if let Some(candidate) = replacement(graph, &indexed, &users, x, y) {
                dirty.insert(x);
                changes.push((x, candidate));
            }
        }
    }

    changes
}

/// The new vertices of route `i` with its section between the first and last vertex
/// shared with route `j` replaced by `j`'s section, if they cross more than once and
/// that lowers `i`'s total crossings while keeping it a simple path.
fn replacement(
    graph: &RoutingGraph,
    indexed: &[Indexed<'_>],
    users: &BTreeMap<VertexId, Vec<usize>>,
    i: usize,
    j: usize,
) -> Option<Vec<VertexId>> {
    let (ia, ib) = (&indexed[i], &indexed[j]);
    if indexed_crossings(graph, ia, ib) < 2 {
        return None;
    }

    let (a, b) = (ia.path, ib.path);
    let shared: Vec<usize> = (0..a.len())
        .filter(|&k| ib.position(a[k]).is_some())
        .collect();
    let (&first, &last) = (shared.first()?, shared.last()?);
    let (pf, pl) = (ib.position(a[first])?, ib.position(a[last])?);
    let mut section: Vec<VertexId> = if pf <= pl {
        b[pf..=pl].to_vec()
    } else {
        b[pl..=pf].iter().rev().copied().collect()
    };
    let mut candidate = a[..first].to_vec();
    candidate.append(&mut section);
    candidate.extend_from_slice(&a[last + 1..]);
    if !is_simple(&candidate) {
        return None;
    }

    // Only routes sharing a vertex with either version can cross it.
    let mut others = BTreeSet::new();
    for v in a.iter().chain(&candidate) {
        others.extend(
            users
                .get(v)
                .into_iter()
                .flatten()
                .copied()
                .filter(|&k| k != i),
        );
    }

    let ic = Indexed::new(&candidate);
    let before: u32 = others
        .iter()
        .map(|&k| indexed_crossings(graph, ia, &indexed[k]))
        .sum();
    let after: u32 = others
        .iter()
        .map(|&k| indexed_crossings(graph, &ic, &indexed[k]))
        .sum();

    (after < before).then_some(candidate)
}

/// Whether no vertex repeats.
fn is_simple(path: &[VertexId]) -> bool {
    let mut sorted = path.to_vec();
    sorted.sort_unstable();

    sorted.windows(2).all(|w| w[0] != w[1])
}

/// A proper crossing between two finished orthogonal routes, as [`iter_points`] reports
/// it.
///
/// Each side is `(route, segment)`: the index of the route, and the index `k` of its
/// segment from point `k` to point `k + 1`. A renderer draws a hop on one of the two
/// segments at [`at`](Self::at).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub struct Crossing {
    /// Where the two segments cross.
    pub at: Point,
    /// The route and segment index of the horizontal segment.
    pub horizontal: (usize, usize),
    /// The route and segment index of the vertical segment.
    pub vertical: (usize, usize),
}

/// The proper crossings between different routes of a drawing, such as
/// [`Drawing::routes`](crate::pipeline::Drawing::routes) or nudged routes; see
/// [`iter_points`].
#[must_use]
pub fn points(routes: &[Vec<Point>]) -> Vec<Crossing> {
    iter_points(routes).collect()
}

/// Iterates the proper crossings between different routes of a drawing, without
/// collecting them; [`points`] collects them.
///
/// The rules are those of [`check::orthogonal_crossings`](crate::check::orthogonal_crossings):
/// a horizontal and a vertical segment cross where their relative interiors meet.
/// Touching endpoints, collinear overlaps and segments that are not axis-parallel are
/// not crossings, and a route crossing itself is not reported. Crossings come in order
/// of route pair `(i, j)` with `i < j`, then of the segments of `i`, then of `j`.
///
/// # Examples
///
/// ```
/// use connector_routing::{Point, crossing};
///
/// let routes = [
///     vec![Point::new(5, 0), Point::new(5, 10)],
///     vec![Point::new(0, 5), Point::new(10, 5), Point::new(10, 20)],
/// ];
/// let mut crossings = crossing::iter_points(&routes);
/// let c = crossings.next().unwrap();
/// assert_eq!(c.at, Point::new(5, 5));
/// assert_eq!((c.horizontal, c.vertical), ((1, 0), (0, 0)));
/// assert_eq!(crossings.next(), None);
/// ```
pub fn iter_points(routes: &[Vec<Point>]) -> impl Iterator<Item = Crossing> {
    routes.iter().enumerate().flat_map(move |(i, a)| {
        routes
            .iter()
            .enumerate()
            .skip(i + 1)
            .flat_map(move |(j, b)| {
                a.windows(2).enumerate().flat_map(move |(sa, pa)| {
                    b.windows(2).enumerate().filter_map(move |(sb, pb)| {
                        let crossing = |h, v, at| Crossing {
                            at,
                            horizontal: h,
                            vertical: v,
                        };
                        crossing_point(pa[0], pa[1], pb[0], pb[1])
                            .map(|at| crossing((i, sa), (j, sb), at))
                            .or_else(|| {
                                crossing_point(pb[0], pb[1], pa[0], pa[1])
                                    .map(|at| crossing((j, sb), (i, sa), at))
                            })
                    })
                })
            })
    })
}
