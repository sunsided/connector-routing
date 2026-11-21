//! Crossings between routes in the routing graph, and crossing reduction
//! (arXiv 2309.01671; crossing detection on shared paths as in Dwyer et al. 2006).
//!
//! Two routes that share graph edges or vertices may or may not have to cross,
//! depending on how they enter and leave what they share. [`required_crossings`] counts
//! the crossings any drawing of the two paths must have: one per shared stretch whose
//! two ends put the routes on different sides, and one per shared vertex they pass
//! straight through in perpendicular directions. Ordering and nudging realise exactly
//! these.

use alloc::vec::Vec;
use core::cmp::Ordering;

use crate::geom::Direction;
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

/// The position of `v` in `path`, if it occurs.
fn position(path: &[VertexId], v: VertexId) -> Option<usize> {
    path.iter().position(|&u| u == v)
}

/// Whether `path` uses the edge between `u` and `w`.
fn has_edge(path: &[VertexId], u: VertexId, w: VertexId) -> bool {
    path.windows(2)
        .any(|e| (e[0] == u && e[1] == w) || (e[0] == w && e[1] == u))
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
    a: &[VertexId],
    b: &[VertexId],
    prev: VertexId,
    end: VertexId,
) -> Option<Ordering> {
    let heading = dir(graph, prev, end);
    let next_a = beyond(a, position(a, end)?, prev)?;
    let next_b = beyond(b, position(b, end)?, prev)?;

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
    let mut count = 0;
    let mut k = 0;
    while k + 1 < a.len() {
        if !has_edge(b, a[k], a[k + 1]) {
            count += u32::from(k > 0 && straight_crossing(graph, a, b, k));
            k += 1;
            continue;
        }

        // A shared stretch from a[k] to a[end].
        let mut end = k + 1;
        while end + 1 < a.len() && has_edge(b, a[end], a[end + 1]) {
            end += 1;
        }

        let at_end = side_at(graph, a, b, a[end - 1], a[end]);
        let at_start = side_at(graph, a, b, a[k + 1], a[k]).map(Ordering::reverse);
        if let (Some(s), Some(e)) = (at_start, at_end) {
            count += u32::from(s != e);
        }

        k = end;
    }

    count
}

/// Whether `a` passes straight through `a[k]` while `b` passes straight through it in
/// the perpendicular direction, sharing no edge there.
fn straight_crossing(graph: &RoutingGraph, a: &[VertexId], b: &[VertexId], k: usize) -> bool {
    let Some(j) = position(b, a[k]) else {
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
    let mut total = 0;
    for (i, a) in routes.iter().enumerate() {
        for b in &routes[i + 1..] {
            total += required_crossings(graph, &a.vertices, &b.vertices);
        }
    }

    total
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
        let mut improved = false;
        for i in 0..routes.len() {
            for j in 0..routes.len() {
                if i != j && try_replace(graph, routes, i, j, weights) {
                    replaced += 1;
                    improved = true;
                }
            }
        }

        if !improved {
            break;
        }
    }

    replaced
}

/// Replaces the section of route `i` between its first and last vertex shared with
/// route `j` by `j`'s section, if they cross more than once and that helps.
fn try_replace(
    graph: &RoutingGraph,
    routes: &mut [Route],
    i: usize,
    j: usize,
    weights: Weights,
) -> bool {
    let (a, b) = (&routes[i].vertices, &routes[j].vertices);
    if required_crossings(graph, a, b) < 2 {
        return false;
    }

    let shared: Vec<usize> = (0..a.len()).filter(|&k| b.contains(&a[k])).collect();
    let (Some(&first), Some(&last)) = (shared.first(), shared.last()) else {
        return false;
    };
    let (pf, pl) = (
        position(b, a[first]).expect("shared"),
        position(b, a[last]).expect("shared"),
    );
    let mut section: Vec<VertexId> = if pf <= pl {
        b[pf..=pl].to_vec()
    } else {
        b[pl..=pf].iter().rev().copied().collect()
    };
    let mut candidate = a[..first].to_vec();
    candidate.append(&mut section);
    candidate.extend_from_slice(&a[last + 1..]);
    if !is_simple(&candidate) {
        return false;
    }

    let total = |path: &[VertexId]| -> u32 {
        routes
            .iter()
            .enumerate()
            .filter(|&(k, _)| k != i)
            .map(|(_, r)| required_crossings(graph, path, &r.vertices))
            .sum()
    };
    if total(&candidate) >= total(a) {
        return false;
    }

    let mut route = Route::from_vertices(graph, candidate, 0);
    route.cost = cost_of(weights, route.length, route.bends);
    routes[i] = route;

    true
}

/// Whether no vertex repeats.
fn is_simple(path: &[VertexId]) -> bool {
    let mut sorted = path.to_vec();
    sorted.sort_unstable();

    sorted.windows(2).all(|w| w[0] != w[1])
}
