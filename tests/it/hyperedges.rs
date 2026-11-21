//! Hyperedge trees: connected, valid, within the spanning-tree bound, nudged without
//! breaking junctions.

use std::collections::{BTreeMap, BTreeSet};

use connector_routing::hyperedge::{Heuristic, Tree, route as tree_route, route_with};
use connector_routing::{
    Objective, Point, Port, Rect, Scene, Search, VertexId, check, nudge, route,
};
use proptest::prelude::*;

use crate::support::rect;

/// Whether the tree's edges connect all `terminals`.
fn connects(tree: &Tree, terminals: &[VertexId]) -> bool {
    let mut adjacency: BTreeMap<VertexId, Vec<VertexId>> = BTreeMap::new();
    for &(a, b) in &tree.edges {
        adjacency.entry(a).or_default().push(b);
        adjacency.entry(b).or_default().push(a);
    }

    let mut seen = BTreeSet::from([terminals[0]]);
    let mut stack = vec![terminals[0]];
    while let Some(v) = stack.pop() {
        for &w in adjacency.get(&v).into_iter().flatten() {
            if seen.insert(w) {
                stack.push(w);
            }
        }
    }

    terminals.iter().all(|t| seen.contains(t))
}

/// Whether the nudged segments of one net form a connected set touching every
/// terminal point.
fn nudged_connected(segments: &[(Point, Point)], terminals: &[Point]) -> bool {
    let on = |p: Point, (a, b): (Point, Point)| {
        p.x >= a.x.min(b.x)
            && p.x <= a.x.max(b.x)
            && p.y >= a.y.min(b.y)
            && p.y <= a.y.max(b.y)
            && (a.x == b.x || a.y == b.y)
    };
    // Axis-parallel segments meet iff their closed bounding boxes intersect.
    let touch = |s: (Point, Point), t: (Point, Point)| {
        s.0.x.min(s.1.x) <= t.0.x.max(t.1.x)
            && t.0.x.min(t.1.x) <= s.0.x.max(s.1.x)
            && s.0.y.min(s.1.y) <= t.0.y.max(t.1.y)
            && t.0.y.min(t.1.y) <= s.0.y.max(s.1.y)
    };
    if segments.is_empty() {
        return terminals.windows(2).all(|w| w[0] == w[1]);
    }

    let mut seen = BTreeSet::from([0_usize]);
    let mut stack = vec![0_usize];
    while let Some(i) = stack.pop() {
        for j in 0..segments.len() {
            if !seen.contains(&j) && touch(segments[i], segments[j]) {
                seen.insert(j);
                stack.push(j);
            }
        }
    }

    seen.len() == segments.len()
        && terminals
            .iter()
            .all(|&p| segments.iter().any(|&s| on(p, s)))
}

proptest! {
    #![proptest_config(crate::support::cases(48))]

    /// Both heuristics connect every terminal, avoid obstacles, cost at most the
    /// minimum spanning tree of the pairwise shortest routes, and survive nudging.
    #[test]
    fn trees_are_valid(
        rects in proptest::collection::vec(rect(15), 0..6),
        points in proptest::collection::btree_set((0..=15, 0..=15), 2..6),
    ) {
        let rects: Vec<Rect> = rects.iter().map(|r| Rect::new(r.min.x * 4, r.min.y * 4, r.max.x * 4, r.max.y * 4)).collect();
        let points: Vec<Point> = points.into_iter().map(|(x, y)| Point::new(x * 4, y * 4)).collect();
        prop_assume!(points.iter().all(|p| !rects.iter().any(|r| r.interior_contains(*p))));
        let scene = Scene::build(rects.iter().copied(), points.iter().map(|&p| Port::free(p))).unwrap();
        let ids: Vec<_> = scene.ports().map(|(id, _)| id).collect();
        let terminals: Vec<VertexId> = ids.iter().map(|&id| scene.port_vertex(id).unwrap()).collect();
        let objective = Objective::Weighted { length: 1, bend: 0 };

        // The minimum spanning tree of the metric closure (Prim) bounds both heuristics.
        let n = ids.len();
        let mut in_tree = vec![false; n];
        let mut best = vec![u64::MAX; n];
        best[0] = 0;
        let mut mst = 0;
        for _ in 0..n {
            let k = (0..n).filter(|&k| !in_tree[k]).min_by_key(|&k| best[k]).unwrap();
            in_tree[k] = true;
            mst += best[k];
            for j in 0..n {
                if !in_tree[j] {
                    let d = route(&scene, ids[k], ids[j], objective, Search::Ovg).unwrap().length;
                    best[j] = best[j].min(d);
                }
            }
        }

        for heuristic in [Heuristic::Sequential, Heuristic::Interleaved] {
            let tree = tree_route(&scene, &ids, objective, heuristic).unwrap();
            let raw = route_with(&scene, &ids, objective, heuristic, false).unwrap();
            prop_assert!(tree.length <= raw.length, "improvement never makes the tree longer");
            let weighted = Objective::Weighted { length: 1, bend: 7 };
            let better = route_with(&scene, &ids, weighted, heuristic, true).unwrap();
            let plain = route_with(&scene, &ids, weighted, heuristic, false).unwrap();
            prop_assert!(better.length + 7 * u64::from(better.bends) <= plain.length + 7 * u64::from(plain.bends));
            prop_assert!(connects(&better, &terminals));
            prop_assert!(connects(&tree, &terminals), "{:?} connects", heuristic);
            prop_assert!(tree.length <= mst, "{:?}: {} <= {}", heuristic, tree.length, mst);
            for &(a, b) in &tree.edges {
                check::orthogonal(&[scene.graph().point(a), scene.graph().point(b)], &rects).unwrap();
            }

            let (segments, _) = nudge::nudge_nets(scene.graph(), &rects, std::slice::from_ref(&tree.net), &nudge::Config::default());
            prop_assert!(nudged_connected(&segments[0], &points), "{:?} stays connected", heuristic);
            for &(a, b) in &segments[0] {
                check::orthogonal(&[a, b], &rects).unwrap();
            }
        }
    }
}

/// With two terminals both heuristics find the shortest route.
#[test]
fn two_terminals_are_a_shortest_path() {
    let rects = [Rect::new(10, 0, 20, 10)];
    let ports = [Port::free(Point::new(0, 5)), Port::free(Point::new(30, 5))];
    let scene = Scene::build(rects, ports).unwrap();
    let ids: Vec<_> = scene.ports().map(|(id, _)| id).collect();
    let objective = Objective::Weighted { length: 1, bend: 0 };
    let shortest = route(&scene, ids[0], ids[1], objective, Search::Ovg).unwrap();
    for heuristic in [Heuristic::Sequential, Heuristic::Interleaved] {
        let tree = tree_route(&scene, &ids, objective, heuristic).unwrap();
        assert_eq!(tree.length, shortest.length);
    }
}

/// The improvement does apply: a comb whose trunk has two branches above and one below
/// moves up to the upper branches' ends, where both vanish.
#[test]
fn shifting_shortens_an_unbalanced_trunk() {
    let ports = [(0, 20), (20, 20), (10, -20)].map(|(x, y)| Port::free(Point::new(x, y)));
    // Lines at y = 0 for the hand-drawn trunk.
    let extra = [(0, 0), (20, 0), (10, 0)].map(|(x, y)| Port::free(Point::new(x, y)));
    let scene = Scene::build([], ports.into_iter().chain(extra)).unwrap();
    let ids: Vec<_> = scene.ports().map(|(id, _)| id).collect();
    let graph = scene.graph();
    let walk = |pts: &[(i32, i32)]| {
        let mut edges = Vec::new();
        for w in pts.windows(2) {
            let (from, to) = (Point::new(w[0].0, w[0].1), Point::new(w[1].0, w[1].1));
            let dir = connector_routing::Direction::between(from, to).unwrap();
            let mut v = graph.vertex_at(from).unwrap();
            while graph.point(v) != to {
                let next = graph.neighbor(v, dir).unwrap();
                edges.push(if graph.point(v) < graph.point(next) {
                    (v, next)
                } else {
                    (next, v)
                });
                v = next;
            }
        }
        edges
    };
    let mut edges = walk(&[(0, 20), (0, 0), (20, 0), (20, 20)]);
    edges.extend(walk(&[(10, 0), (10, -20)]));
    edges.sort();
    let comb = Tree {
        edges,
        length: 80,
        bends: 3,
        net: connector_routing::Net::default(),
    };
    let objective = Objective::Weighted { length: 1, bend: 1 };
    let better =
        connector_routing::hyperedge::improve(&scene, &comb, &ids[..3], objective).unwrap();
    // The trunk moves to y = 20: 20 along the top, 40 down to (10, -20).
    assert_eq!(better.length, 60);
    assert!(connects(
        &better,
        &[ids[0], ids[1], ids[2]].map(|id| scene.port_vertex(id).unwrap())
    ));
}
