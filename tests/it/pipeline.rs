//! The arXiv 2309.01671 pipeline on random layouts: valid, separated drawings.

use connector_routing::{Rect, check, pipeline};
use proptest::prelude::*;

proptest! {
    #![proptest_config(ProptestConfig::with_cases(24))]

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
        let drawing = pipeline::draw(&boxes, &edges, &pipeline::Config::default()).unwrap();
        for (route, (s, t)) in drawing.routes.iter().zip(&drawing.ports) {
            check::orthogonal(route, &boxes).unwrap();
            prop_assert_eq!(route[0], s.point);
            prop_assert_eq!(*route.last().unwrap(), t.point);
        }
    }
}
