//! Local improvement of hyperedge trees (Wybrow et al. 2012, section 3).
//!
//! The paper's second transformation shifts a straight segment of the tree sideways, in
//! the direction where more branches diverge from it: every branch on that side gets
//! shorter and every branch on the other side longer by the same amount, so the tree
//! shrinks when the sides are unbalanced. The shift stops at the end of the shortest
//! diverging branch, where that branch vanishes and its bend with it (the paper's first
//! transformation, merging parallel routes at a junction, happens as part of this). Shifts
//! that would leave the graph, create a cycle or not lower the cost are rejected, so the
//! tree never gets worse and stays a tree over orthogonal visibility graph edges.

use alloc::collections::{BTreeMap, BTreeSet};
use alloc::vec::Vec;

use crate::geom::{DirSet, Direction};
use crate::graph::{RoutingGraph, VertexId};
use crate::search::Weights;

/// A tree as an edge set, each edge `(a, b)` with `a` the smaller point.
pub(crate) type Edges = BTreeSet<(VertexId, VertexId)>;

/// The canonical form of the edge between neighbours `a` and `b`.
fn edge(graph: &RoutingGraph, a: VertexId, b: VertexId) -> (VertexId, VertexId) {
    if graph.point(a) < graph.point(b) {
        (a, b)
    } else {
        (b, a)
    }
}

/// The directions of the tree's edges at every vertex.
pub(crate) fn arms(graph: &RoutingGraph, edges: &Edges) -> BTreeMap<VertexId, DirSet> {
    let mut arms: BTreeMap<VertexId, DirSet> = BTreeMap::new();
    for &(a, b) in edges {
        let d = Direction::between(graph.point(a), graph.point(b)).expect("aligned edge");
        *arms.entry(a).or_default() |= DirSet::only(d);
        *arms.entry(b).or_default() |= DirSet::only(d.reverse());
    }

    arms
}

/// The cost of a tree: length and bends (vertices with arms on both axes) under
/// `weights`.
pub(crate) fn cost(graph: &RoutingGraph, edges: &Edges, weights: Weights) -> u64 {
    let length: u64 = edges
        .iter()
        .map(|&(a, b)| graph.point(a).manhattan(graph.point(b)))
        .sum();
    let bends = arms(graph, edges)
        .values()
        .filter(|a| {
            a.iter().any(|d| d.axis() == crate::geom::Axis::X)
                && a.iter().any(|d| d.axis() == crate::geom::Axis::Y)
        })
        .count();

    weights.length.saturating_mul(length).saturating_add(
        weights
            .bend
            .saturating_mul(u64::try_from(bends).unwrap_or(u64::MAX)),
    )
}

/// The maximal straight runs of the tree along `forward` (Right or Up), as vertex lists.
fn runs(graph: &RoutingGraph, edges: &Edges, forward: Direction) -> Vec<Vec<VertexId>> {
    let arms = arms(graph, edges);
    let mut out = Vec::new();
    for (&v, set) in &arms {
        if set.contains(forward) && !set.contains(forward.reverse()) {
            let mut run = alloc::vec![v];
            let mut at = v;
            while arms.get(&at).is_some_and(|a| a.contains(forward)) {
                at = graph
                    .neighbor(at, forward)
                    .expect("tree edges are graph edges");
                run.push(at);
            }

            out.push(run);
        }
    }

    out
}

/// The graph edges from `from` in direction `dir` up to the vertex whose coordinate
/// across `dir` equals `target`, or `None` if the graph does not reach it.
fn chain(
    graph: &RoutingGraph,
    from: VertexId,
    dir: Direction,
    target: i32,
) -> Option<(VertexId, Vec<(VertexId, VertexId)>)> {
    let axis = dir.axis();
    let mut at = from;
    let mut out = Vec::new();
    loop {
        let c = graph.point(at).coordinate(axis);
        if c == target {
            return Some((at, out));
        }

        if (dir.is_positive() && c > target) || (!dir.is_positive() && c < target) {
            return None;
        }

        let next = graph.neighbor(at, dir)?;
        out.push(edge(graph, at, next));
        at = next;
    }
}

/// Whether `edges` form a tree that contains every terminal.
fn is_tree(graph: &RoutingGraph, edges: &Edges, terminals: &[VertexId]) -> bool {
    let arms = arms(graph, edges);
    if arms.len() != edges.len() + 1 && !(edges.is_empty() && terminals.len() <= 1) {
        return false;
    }

    let Some(&start) = arms.keys().next() else {
        return terminals.len() <= 1;
    };
    let mut seen = BTreeSet::from([start]);
    let mut stack = alloc::vec![start];
    while let Some(v) = stack.pop() {
        for d in arms[&v].iter() {
            let w = graph.neighbor(v, d).expect("tree edges are graph edges");
            if seen.insert(w) {
                stack.push(w);
            }
        }
    }

    seen.len() == arms.len() && terminals.iter().all(|t| seen.contains(t))
}

/// Shifts `run` (along `forward`) towards `side`, to the end of the shortest branch
/// diverging on that side. Returns the new edge set, or `None` if the shift is not
/// possible.
fn shift(
    graph: &RoutingGraph,
    edges: &Edges,
    run: &[VertexId],
    forward: Direction,
    side: Direction,
) -> Option<Edges> {
    let arms = arms(graph, edges);
    let across = side.axis();
    // The nearest end of a branch on `side`: the shift's target line.
    let mut target: Option<i32> = None;
    for &u in run {
        if !arms[&u].contains(side) {
            continue;
        }

        let mut at = u;
        while arms.get(&at).is_some_and(|a| a.contains(side)) {
            at = graph.neighbor(at, side)?;
        }

        let c = graph.point(at).coordinate(across);
        target = Some(match target {
            None => c,
            Some(t) if side.is_positive() => t.min(c),
            Some(t) => t.max(c),
        });
    }

    let target = target?;
    let mut next = edges.clone();
    for pair in run.windows(2) {
        next.remove(&edge(graph, pair[0], pair[1]));
    }

    // Move every run vertex across to the target line.
    let mut moved = Vec::with_capacity(run.len());
    for &u in run {
        let (w, path) = chain(graph, u, side, target)?;
        if arms[&u].contains(side) {
            // The branch on this side loses its part up to the target line.
            for e in &path {
                next.remove(e);
            }
        }

        if arms[&u].contains(side.reverse()) {
            // A branch on the other side grows by the shift.
            next.extend(path);
        }

        moved.push(w);
    }

    // The run on its new line, through every moved vertex.
    for pair in moved.windows(2) {
        let (w, path) = chain(
            graph,
            pair[0],
            forward,
            graph.point(pair[1]).coordinate(forward.axis()),
        )?;
        if w != pair[1] {
            return None;
        }

        next.extend(path);
    }

    Some(next)
}

/// Removes non-terminal leaves until none is left.
fn prune(graph: &RoutingGraph, edges: &mut Edges, terminals: &[VertexId]) {
    loop {
        let arms = arms(graph, edges);
        let before = edges.len();
        edges.retain(|&(a, b)| {
            let leaf = |v: VertexId| arms[&v].iter().count() == 1 && !terminals.contains(&v);
            !(leaf(a) || leaf(b))
        });
        if edges.len() == before {
            return;
        }
    }
}

/// Improves a tree by unbalanced segment shifts until none lowers its cost (at most
/// `max_rounds` rounds). The result is never more expensive and still a tree over graph
/// edges containing every terminal.
pub(crate) fn improve(
    graph: &RoutingGraph,
    edges: &mut Edges,
    terminals: &[VertexId],
    weights: Weights,
    max_rounds: u32,
) {
    for _ in 0..max_rounds {
        if !improve_once(graph, edges, terminals, weights) {
            return;
        }
    }
}

/// One shift that lowers the cost, if any; returns whether it applied one.
fn improve_once(
    graph: &RoutingGraph,
    edges: &mut Edges,
    terminals: &[VertexId],
    weights: Weights,
) -> bool {
    let current = cost(graph, edges, weights);
    for forward in [Direction::Right, Direction::Up] {
        for run in runs(graph, edges, forward) {
            if run.iter().any(|v| terminals.contains(v)) {
                continue;
            }

            let arms = arms(graph, edges);
            let count = |d: Direction| run.iter().filter(|u| arms[u].contains(d)).count();
            for side in [forward.left(), forward.right()] {
                if count(side) <= count(side.reverse()) {
                    continue;
                }

                let Some(mut next) = shift(graph, edges, &run, forward, side) else {
                    continue;
                };
                prune(graph, &mut next, terminals);
                if is_tree(graph, &next, terminals) && cost(graph, &next, weights) < current {
                    *edges = next;
                    return true;
                }
            }
        }
    }

    false
}
