//! Fuzzes hyperedge routing: rectilinear Steiner trees over the orthogonal visibility
//! graph (Wybrow et al. 2012), with both heuristics and with or without segment shifting.
//!
//! The input describes a few boxes and terminals on a small grid, so terminals on box
//! boundaries, collinear terminals and touching boxes are common. Scene edits and tree
//! construction may fail with an error, but they must never panic.
//!
//! Every tree that is found must:
//!
//! - consist of obstacle-free axis-parallel graph edges;
//! - be connected and reach every terminal;
//! - report a length equal to the sum of its edge lengths.

#![no_main]
#![forbid(unsafe_code)]

use std::collections::BTreeMap;

use connector_routing::hyperedge::{self, Heuristic, Tree};
use connector_routing::{Objective, Point, Port, Rect, Scene, VertexId, check};
use libfuzzer_sys::fuzz_target;

/// The most obstacles.
const MAX_OBSTACLES: usize = 5;

/// The most terminals.
const MAX_TERMINALS: usize = 6;

/// Reads fuzz bytes one at a time, with zeros once the input runs out.
struct Bytes<'a>(core::slice::Iter<'a, u8>);

impl Bytes<'_> {
    /// The next byte, or zero past the end.
    fn byte(&mut self) -> u8 {
        self.0.next().copied().unwrap_or(0)
    }

    /// A count in `0..bound`.
    fn below(&mut self, bound: usize) -> usize {
        usize::from(self.byte()) % bound
    }

    /// A coordinate on the grid of 4 in `[0, 64)`.
    fn coordinate(&mut self) -> i32 {
        i32::from(self.byte() % 16) * 4
    }

    /// A rectangle, or `None` if its corners give it no area.
    fn rect(&mut self) -> Option<Rect> {
        Rect::try_new(
            self.coordinate(),
            self.coordinate(),
            self.coordinate(),
            self.coordinate(),
        )
    }
}

/// Follows union-find parents in `parent` from `v` to its root.
fn root(parent: &BTreeMap<VertexId, VertexId>, mut v: VertexId) -> VertexId {
    while let Some(&p) = parent.get(&v) {
        v = p;
    }

    v
}

/// Panics unless `tree` is a valid Steiner tree of `terminals` in `scene`.
fn assert_valid(scene: &Scene, tree: &Tree, terminals: &[VertexId]) {
    let graph = scene.graph();
    let rects = scene.rects();
    let mut parent: BTreeMap<VertexId, VertexId> = BTreeMap::new();
    let mut length = 0;
    for &(a, b) in &tree.edges {
        let segment = [graph.point(a), graph.point(b)];
        check::orthogonal(&segment, &rects)
            .unwrap_or_else(|v| panic!("invalid tree edge {segment:?}: {v:?}"));
        length += segment[0].manhattan(segment[1]);
        let (ra, rb) = (root(&parent, a), root(&parent, b));
        if ra != rb {
            parent.insert(ra, rb);
        }
    }

    assert_eq!(tree.length, length, "reported tree length");
    if tree.edges.is_empty() {
        return;
    }

    let reached: Vec<VertexId> = tree.edges.iter().flat_map(|&(a, b)| [a, b]).collect();
    let tree_root = root(&parent, reached[0]);
    for &t in terminals {
        assert!(
            reached.contains(&t),
            "terminal {:?} is on the tree",
            graph.point(t)
        );
        assert_eq!(root(&parent, t), tree_root, "the tree is connected");
    }
}

fuzz_target!(|data: &[u8]| {
    let mut bytes = Bytes(data.iter());
    let mut scene = Scene::new();
    for _ in 0..bytes.below(MAX_OBSTACLES + 1) {
        if let Some(rect) = bytes.rect() {
            let _ = scene.add_obstacle(rect);
        }
    }

    let ports: Vec<_> = (0..=bytes.below(MAX_TERMINALS - 1) + 1)
        .filter_map(|_| {
            let point = Point::new(bytes.coordinate(), bytes.coordinate());
            scene.add_port(Port::free(point)).ok()
        })
        .collect();
    let heuristic = if bytes.byte() % 2 == 0 {
        Heuristic::Sequential
    } else {
        Heuristic::Interleaved
    };
    let improve = bytes.byte() % 2 == 0;
    let objective = Objective::Weighted {
        length: 1,
        bend: u64::from(bytes.byte() % 8),
    };

    let Ok(tree) = hyperedge::route_with(&scene, &ports, objective, heuristic, improve) else {
        return;
    };
    let terminals: Vec<VertexId> = ports.iter().filter_map(|&p| scene.port_vertex(p)).collect();
    assert_valid(&scene, &tree, &terminals);
});
