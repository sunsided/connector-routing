//! The arXiv 2309.01671 pipeline on random layouts: valid, separated drawings.

use connector_routing::{Rect, check, pipeline};
use proptest::prelude::*;

/// Every edge of the channel graph is an obstacle-free axis-parallel segment.
#[test]
fn channel_graph_edges_are_free() {
    let boxes: Vec<Rect> = (0..4)
        .flat_map(|i| (0..3).map(move |j| Rect::new(i * 60, j * 50, i * 60 + 24, j * 50 + 16)))
        .collect();
    let edges: Vec<(usize, usize)> = (0..boxes.len())
        .map(|i| (i, (i * 5 + 3) % boxes.len()))
        .filter(|(a, b)| a != b)
        .collect();
    let ports: Vec<_> = pipeline::assign_ports(&boxes, &edges)
        .into_iter()
        .flat_map(|(a, b)| [a, b])
        .collect();
    let graph = pipeline::channel_graph(&boxes, &ports, Rect::new(-40, -40, 260, 190));
    for (a, b) in graph.edges() {
        check::orthogonal(&[graph.point(a), graph.point(b)], &boxes).unwrap();
    }

    for p in &ports {
        assert!(graph.vertex_at(p.point).is_some(), "every port is a vertex");
    }

    let config = pipeline::Config {
        graph: pipeline::GraphKind::Channels,
        ..pipeline::Config::default()
    };
    let drawing = pipeline::draw(&boxes, &edges, &config).unwrap();
    assert_eq!(
        drawing.graph,
        pipeline::GraphKind::Channels,
        "this grid routes on channels"
    );
}

proptest! {
    #![proptest_config(crate::support::cases(24))]

    /// Boxes on a coarse grid with random edges (self-loops and multi-edges included):
    /// every route starts and ends at its ports, leaves and enters them outwards, and
    /// avoids every box.
    #[test]
    fn drawings_are_valid(
        cells in proptest::collection::btree_set((0_i32..6, 0_i32..6), 2..10),
        picks in proptest::collection::vec((any::<usize>(), any::<usize>()), 1..14),
    ) {
        let boxes: Vec<Rect> = cells.iter().map(|&(i, j)| Rect::new(i * 60, j * 60, i * 60 + 24, j * 60 + 16)).collect();
        let edges: Vec<(usize, usize)> = picks.iter().map(|&(a, b)| (a % boxes.len(), b % boxes.len())).collect();
        for graph in [pipeline::GraphKind::Visibility, pipeline::GraphKind::Channels] {
            let config = pipeline::Config { graph, ..pipeline::Config::default() };
            let drawing = pipeline::draw(&boxes, &edges, &config).unwrap();
            for (route, (s, t)) in drawing.routes.iter().zip(&drawing.ports) {
                check::orthogonal(route, &boxes).unwrap();
                prop_assert_eq!(route[0], s.point);
                prop_assert_eq!(*route.last().unwrap(), t.point);
            }

        }
    }
}

proptest! {
    #![proptest_config(crate::support::cases(24))]

    /// Full nudging never shrinks a box, keeps boxes apart, keeps ports on their moved
    /// boxes, and routes clear of the moved boxes.
    #[test]
    fn full_nudging_is_valid(
        cells in proptest::collection::btree_set((0_i32..6, 0_i32..6), 2..10),
        picks in proptest::collection::vec((any::<usize>(), any::<usize>()), 1..12),
    ) {
        let boxes: Vec<Rect> = cells.iter().map(|&(i, j)| Rect::new(i * 60, j * 60, i * 60 + 24, j * 60 + 16)).collect();
        let edges: Vec<(usize, usize)> = picks.iter().map(|&(a, b)| (a % boxes.len(), b % boxes.len())).filter(|(a, b)| a != b).collect();
        prop_assume!(!edges.is_empty());
        let config = pipeline::Config { full_nudging: true, ..pipeline::Config::default() };
        let drawing = pipeline::draw(&boxes, &edges, &config).unwrap();
        if !drawing.full {
            return Ok(());
        }

        for (old, new) in boxes.iter().zip(&drawing.boxes) {
            prop_assert!(new.width() >= old.width() && new.height() >= old.height());
        }

        for (i, a) in drawing.boxes.iter().enumerate() {
            for b in &drawing.boxes[i + 1..] {
                prop_assert!(!a.interiors_overlap(b), "boxes stay apart");
            }
        }

        for route in &drawing.routes {
            check::orthogonal(route, &drawing.boxes).unwrap();
            for end in [route[0], *route.last().unwrap()] {
                prop_assert!(drawing.boxes.iter().any(|b| b.boundary_contains(end)), "ports stay on their boxes");
            }
        }
    }
}
