//! The order of nets across shared edges, and from it the order of segments on each
//! line.
//!
//! Every shared edge gets a pseudo direction (Wybrow et al. 2009, section 5.1) so that
//! each net runs through its shared stretches head to tail. Two nets on an edge are
//! compared by walking both from the edge towards its pseudo entry while they take the
//! same edges; at the first divergence, the net that turns more to the left of the
//! walking direction lies on the left side. A walk that reaches a net's end is
//! inconclusive and the walk towards the exit decides; a net that branches is followed
//! along its leftmost branch; identical walks fall back to net index. Every walk is a
//! sequence of ranked choices, so this is a total order on each edge, and all edges of a
//! shared stretch, bends included, walk to the same end and agree.

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

/// A walk comparison from one end of an edge, strict or not.
type Walk<'a> = dyn Fn(bool) -> Option<Ordering> + 'a;

/// The order of nets `a` and `b` (by index `ia`, `ib`) across the edge from `low` to
/// `high`, in increasing coordinate across the edge: `Less` means `a` is below `b` on a
/// horizontal edge, left of `b` on a vertical one. The flag is set when the order is
/// soft: both ends decide it and disagree, so the pair crosses once whichever order
/// the line takes.
pub(crate) fn compare_on_edge(
    graph: &RoutingGraph,
    nets: &[Net],
    (ia, ib): (usize, usize),
    (low, high): (VertexId, VertexId),
    axis: Axis,
    entry_is_low: bool,
) -> (Ordering, bool) {
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

    // When both ends decide and disagree, the pair crosses once in either order: the
    // order is soft and may give way to the others on the line.
    let (first, second): (&Walk<'_>, &Walk<'_>) = if entry_is_low {
        (&first, &second)
    } else {
        (&second, &first)
    };
    let (strict_first, strict_second) = (first(true), second(true));
    let soft = matches!((strict_first, strict_second), (Some(x), Some(y)) if x != y);
    let order = strict_first
        .or(strict_second)
        .or_else(|| first(false))
        .or_else(|| second(false))
        .unwrap_or_else(|| ia.cmp(&ib));

    (order, soft)
}

/// Pseudo directions of the shared edges (Wybrow et al. 2009, section 5.1): every edge
/// used by two or more nets gets a direction such that each net runs through a
/// connected stretch of shared edges head to tail. Keyed by `(low vertex, Right or Up)`;
/// `true` means the pseudo direction runs from the low end to the high end, so the low
/// end is where nets enter the edge.
///
/// Every shared edge then compares nets by walking towards the entry, so all edges of a
/// shared stretch, bends included, agree on the order and a necessary crossing happens
/// once, at the exit. Where a net would force a conflicting direction (the paper's split
/// point), the first assignment stays.
fn pseudo_directions(graph: &RoutingGraph, nets: &[Net]) -> BTreeMap<(VertexId, Direction), bool> {
    let mut users: BTreeMap<(VertexId, Direction), Vec<usize>> = BTreeMap::new();
    for (index, net) in nets.iter().enumerate() {
        for edge in net.edges() {
            users.entry(edge).or_default().push(index);
        }
    }

    users.retain(|_, list| list.len() >= 2);
    let mut pseudo: BTreeMap<(VertexId, Direction), bool> = BTreeMap::new();
    for &start in users.keys() {
        if pseudo.contains_key(&start) {
            continue;
        }

        pseudo.insert(start, true);
        let mut queue = alloc::vec![start];
        while let Some((low, forward)) = queue.pop() {
            let high = graph.neighbor(low, forward).expect("a used edge exists");
            let (tail, head, heading) = if pseudo[&(low, forward)] {
                (low, high, forward)
            } else {
                (high, low, forward.reverse())
            };
            for &n in &users[&(low, forward)] {
                // Continue at the head away from the edge, and back from the tail.
                for (at, dirs_from, outward) in
                    [(head, heading.reverse(), true), (tail, heading, false)]
                {
                    for d in nets[n].arms_at(at).iter().filter(|&d| d != dirs_from) {
                        let key = edge_key(graph, at, d);
                        if !users.contains_key(&key) || pseudo.contains_key(&key) {
                            continue;
                        }

                        // Outward from the head: the edge points away from `at`.
                        let away_is_forward = d.is_positive();
                        let from_low = if outward {
                            away_is_forward
                        } else {
                            !away_is_forward
                        };
                        pseudo.insert(key, from_low);
                        queue.push(key);
                    }
                }
            }
        }
    }

    pseudo
}

/// The key `(low vertex, Right or Up)` of the edge leaving `at` in direction `d`.
fn edge_key(graph: &RoutingGraph, at: VertexId, d: Direction) -> (VertexId, Direction) {
    if d.is_positive() {
        (at, d)
    } else {
        (graph.neighbor(at, d).expect("a net edge"), d.reverse())
    }
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
/// ties by low end and id. On a cycle the extra arcs are dropped first, then the soft
/// orders (pairs that cross once in either order), and only then is a cycle broken.
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
    let pseudo = pseudo_directions(graph, nets);
    let mut arcs = BTreeSet::new();
    let mut soft = BTreeSet::new();
    for (low, users) in edge_users(nets, axis) {
        let high = graph.neighbor(low, forward).expect("a used edge exists");
        for (i, &a) in users.iter().enumerate() {
            for &b in &users[i + 1..] {
                let sa = segments.of_edge(graph, a, low, forward).expect("net edge");
                let sb = segments.of_edge(graph, b, low, forward).expect("net edge");
                let entry_is_low = pseudo.get(&(low, forward)).copied().unwrap_or(true);
                let (order, is_soft) =
                    compare_on_edge(graph, nets, (a, b), (low, high), axis, entry_is_low);
                let arc = if order.is_lt() { (sa, sb) } else { (sb, sa) };
                if is_soft {
                    soft.insert(arc);
                } else {
                    arcs.insert(arc);
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
    // Try everything, then without the kiss arcs, then without the soft orders.
    let all: BTreeSet<(usize, usize)> = arcs.iter().chain(&soft).chain(extra).copied().collect();
    let with_soft: BTreeSet<(usize, usize)> = arcs.iter().chain(&soft).copied().collect();
    let with_extra: BTreeSet<(usize, usize)> = arcs.iter().chain(extra).copied().collect();
    for members in lines.values() {
        let order = topological(graph, segments, members, &all)
            .or_else(|| topological(graph, segments, members, &with_soft))
            .or_else(|| topological(graph, segments, members, &with_extra))
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
