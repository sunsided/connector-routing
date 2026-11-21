//! The order of nets across shared edges, and from it the order of segments on each
//! line.
//!
//! Following the orthogonal variant of Pupyrev et al. in arXiv 2309.01671, every
//! horizontal edge looks to the left and every vertical edge looks down: two nets on an
//! edge are compared by walking both from the edge in that direction while they take
//! the same edges. At the first divergence, the net that turns more to the left of the
//! walking direction lies on the left side. A net that ends ranks between a left turn
//! and going straight; a net that branches is followed along its leftmost branch. Ties
//! (identical walks) are broken by the walk in the other direction, then by net index.
//! Every walk is a sequence of ranked choices, so this is a total order on each edge,
//! and it agrees on every edge of a shared straight stretch.

use alloc::collections::{BTreeMap, BTreeSet};
use alloc::vec::Vec;
use core::cmp::Ordering;

use crate::geom::{Axis, Direction};
use crate::graph::{RoutingGraph, VertexId};
use crate::nets::Net;

use super::segments::Segments;

/// The rank of a choice relative to the walking heading; smaller is further left.
fn rank(heading: Direction, arm: Option<Direction>) -> u8 {
    match arm {
        Some(a) if a == heading.left() => 0,
        None => 1,
        Some(a) if a == heading => 2,
        Some(_) => 3,
    }
}

/// The leftmost continuation of `net` at `at` when arriving with `heading`, or `None`
/// if the net ends there.
fn continuation(net: &Net, at: VertexId, heading: Direction) -> Option<Direction> {
    let arms = net.arms_at(at);
    [heading.left(), heading, heading.right()]
        .into_iter()
        .find(|&d| arms.contains(d))
}

/// Compares two nets by walking both from `start` with `heading`: `Less` if `a` diverges
/// further to the left. `None` if the walks never diverge, or if `strict` and they
/// "diverge" because one of them ends: a net ending at a port does not say on which side
/// the other must pass.
fn walk(
    graph: &RoutingGraph,
    a: &Net,
    b: &Net,
    start: VertexId,
    heading: Direction,
    strict: bool,
) -> Option<Ordering> {
    let (mut at, mut heading) = (start, heading);
    // Nets with loops could walk forever; no simple walk is longer than this.
    for _ in 0..=2 * graph.len() {
        let ca = continuation(a, at, heading);
        let cb = continuation(b, at, heading);
        if strict && (ca.is_none() || cb.is_none()) {
            return None;
        }

        let order = rank(heading, ca).cmp(&rank(heading, cb));
        if order.is_ne() {
            return Some(order);
        }

        let next = graph.neighbor(at, ca?)?;
        heading = ca?;
        at = next;
    }

    None
}

/// The order of nets `a` and `b` (by index `ia`, `ib`) across the edge from `low` to
/// `high`, in increasing coordinate across the edge: `Less` means `a` is below `b` on a
/// horizontal edge, left of `b` on a vertical one.
pub(crate) fn compare_on_edge(
    graph: &RoutingGraph,
    nets: &[Net],
    (ia, ib): (usize, usize),
    (low, high): (VertexId, VertexId),
    axis: Axis,
) -> Ordering {
    let (a, b) = (&nets[ia], &nets[ib]);
    let (look, back) = match axis {
        Axis::X => (Direction::Left, Direction::Right),
        Axis::Y => (Direction::Down, Direction::Up),
    };
    // Left of `Left` is below and left of `Down` is to the right; left of `Right` is
    // above and left of `Up` is to the left.
    let first = |strict| {
        walk(graph, a, b, low, look, strict).map(|o| if axis == Axis::X { o } else { o.reverse() })
    };
    let second = |strict| {
        walk(graph, a, b, high, back, strict).map(|o| if axis == Axis::X { o.reverse() } else { o })
    };

    first(true)
        .or_else(|| second(true))
        .or_else(|| first(false))
        .or_else(|| second(false))
        .unwrap_or_else(|| ia.cmp(&ib))
}

/// For every graph edge along `axis` used by at least two nets, the nets using it.
/// Keyed by the edge's low vertex.
fn edge_users(nets: &[Net], axis: Axis) -> BTreeMap<VertexId, Vec<usize>> {
    let forward = match axis {
        Axis::X => Direction::Right,
        Axis::Y => Direction::Up,
    };
    let mut users: BTreeMap<VertexId, Vec<usize>> = BTreeMap::new();
    for (index, net) in nets.iter().enumerate() {
        for (v, dir) in net.edges() {
            if dir == forward {
                users.entry(v).or_default().push(index);
            }
        }
    }

    users.retain(|_, list| list.len() >= 2);

    users
}

/// The rank of every segment along `axis` among the segments on its line: a
/// topological order of the per-edge orders, the `required` arcs and the `extra` arcs,
/// ties by low end and id. If the extra arcs create a cycle they are dropped for that
/// line.
pub(crate) fn line_ranks(
    graph: &RoutingGraph,
    nets: &[Net],
    segments: &Segments,
    axis: Axis,
    required: &[(usize, usize)],
    extra: &[(usize, usize)],
) -> Vec<u32> {
    let forward = match axis {
        Axis::X => Direction::Right,
        Axis::Y => Direction::Up,
    };
    // The pairwise order on every shared edge. Pairs on one straight stretch compare
    // the same on all its edges.
    let mut arcs = BTreeSet::new();
    for (low, users) in edge_users(nets, axis) {
        let high = graph.neighbor(low, forward).expect("a used edge exists");
        for (i, &a) in users.iter().enumerate() {
            for &b in &users[i + 1..] {
                let sa = segments.of_edge(graph, a, low, forward).expect("net edge");
                let sb = segments.of_edge(graph, b, low, forward).expect("net edge");
                if compare_on_edge(graph, nets, (a, b), (low, high), axis).is_lt() {
                    arcs.insert((sa, sb));
                } else {
                    arcs.insert((sb, sa));
                }
            }
        }
    }

    arcs.extend(required.iter().copied());
    let mut lines: BTreeMap<i32, Vec<usize>> = BTreeMap::new();
    for (id, segment) in segments.list.iter().enumerate() {
        if segment.axis == axis {
            lines.entry(segments.line[id]).or_default().push(id);
        }
    }

    let mut ranks = alloc::vec![0; segments.list.len()];
    for members in lines.values() {
        let with_extra: BTreeSet<(usize, usize)> = arcs.iter().chain(extra).copied().collect();
        let order = topological(graph, segments, members, &with_extra)
            .or_else(|| topological(graph, segments, members, &arcs))
            .unwrap_or_else(|| breaking_cycles(graph, segments, members, &arcs));
        for (rank, &id) in order.iter().enumerate() {
            ranks[id] = u32::try_from(rank).expect("fewer than 2^32 segments");
        }
    }

    ranks
}

/// A topological order that breaks cycles: when no member is free, the smallest
/// remaining one goes next. Only reached if ties between ending nets make the pairwise
/// orders intransitive.
fn breaking_cycles(
    graph: &RoutingGraph,
    segments: &Segments,
    members: &[usize],
    arcs: &BTreeSet<(usize, usize)>,
) -> Vec<usize> {
    let key = |id: usize| {
        let s = &segments.list[id];
        (graph.point(s.low()).coordinate(s.axis), id)
    };
    let mut left: BTreeSet<(i32, usize)> = members.iter().map(|&m| key(m)).collect();
    let mut order = Vec::with_capacity(members.len());
    while !left.is_empty() {
        let free = left
            .iter()
            .copied()
            .find(|&(_, id)| !arcs.iter().any(|&(a, b)| b == id && left.contains(&key(a))));
        let next = free.unwrap_or_else(|| *left.first().expect("non-empty"));
        left.remove(&next);
        order.push(next.1);
    }

    order
}

/// A topological order of `members` under the arcs between them, smallest low end
/// (then id) first; `None` on a cycle.
fn topological(
    graph: &RoutingGraph,
    segments: &Segments,
    members: &[usize],
    arcs: &BTreeSet<(usize, usize)>,
) -> Option<Vec<usize>> {
    let inside: BTreeSet<usize> = members.iter().copied().collect();
    let mut indegree: BTreeMap<usize, usize> = members.iter().map(|&m| (m, 0)).collect();
    let mut out: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for &(a, b) in arcs {
        if inside.contains(&a) && inside.contains(&b) {
            *indegree.get_mut(&b).expect("member") += 1;
            out.entry(a).or_default().push(b);
        }
    }

    let key = |id: usize| {
        let s = &segments.list[id];
        (graph.point(s.low()).coordinate(s.axis), id)
    };
    let mut ready: BTreeSet<(i32, usize)> = indegree
        .iter()
        .filter(|&(_, &d)| d == 0)
        .map(|(&id, _)| key(id))
        .collect();
    let mut order = Vec::with_capacity(members.len());
    while let Some(first) = ready.pop_first() {
        order.push(first.1);
        for &next in out.get(&first.1).map_or(&[][..], Vec::as_slice) {
            let d = indegree.get_mut(&next).expect("member");
            *d -= 1;
            if *d == 0 {
                ready.insert(key(next));
            }
        }
    }

    (order.len() == members.len()).then_some(order)
}
