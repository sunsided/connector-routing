//! Fuzzes the drawing pipeline of arXiv 2309.01671: port assignment, global routing,
//! crossing reduction and constrained or full nudging.
//!
//! The input describes up to [`MAX_BOXES`] boxes on a small grid, so touching boxes,
//! narrow channels and aligned sides are common, and up to [`MAX_EDGES`] edges between
//! them, self-loops and multi-edges included. Boxes that overlap an earlier one are
//! skipped, because [`pipeline::draw`] takes non-overlapping boxes. A byte picks the
//! routing graph, full nudging and the number of crossing-reduction rounds. Drawing may
//! fail with an error, but it must never panic.
//!
//! Every drawing must:
//!
//! - route every edge from its source port to its target port (under full nudging, from
//!   the moved source box to the moved target box);
//! - keep every route an orthogonal polyline that enters no box of the drawing;
//! - under full nudging, never shrink a box and keep the boxes apart.

#![no_main]
#![forbid(unsafe_code)]

use connector_routing::{Rect, check, pipeline};
use libfuzzer_sys::fuzz_target;

/// The most boxes, which keeps a run short.
const MAX_BOXES: usize = 6;

/// The most edges.
const MAX_EDGES: usize = 8;

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

    /// A box with corners on the grid of 4 in `[0, 128)` and sides of 4 to 64.
    fn rect(&mut self) -> Rect {
        let (x, y) = (
            i32::from(self.byte() % 32) * 4,
            i32::from(self.byte() % 32) * 4,
        );
        let (w, h) = (
            i32::from(self.byte() % 16 + 1) * 4,
            i32::from(self.byte() % 16 + 1) * 4,
        );

        Rect::new(x, y, x + w, y + h)
    }

    /// A pipeline configuration from one byte.
    fn config(&mut self) -> pipeline::Config {
        let bits = self.byte();
        let graph = if bits & 1 == 0 {
            pipeline::GraphKind::Visibility
        } else {
            pipeline::GraphKind::Channels
        };

        pipeline::Config {
            graph,
            full_nudging: bits & 2 != 0,
            crossing_rounds: u32::from(bits >> 2) % 5,
            ..pipeline::Config::default()
        }
    }
}

/// Up to [`MAX_BOXES`] boxes, skipping each one that overlaps an earlier one.
fn boxes(bytes: &mut Bytes<'_>) -> Vec<Rect> {
    let mut out: Vec<Rect> = Vec::new();
    for _ in 0..=bytes.below(MAX_BOXES) {
        let rect = bytes.rect();
        if !out.iter().any(|r| r.interiors_overlap(&rect)) {
            out.push(rect);
        }
    }

    out
}

/// Panics unless `drawing` is a valid drawing of `edges` between `boxes`.
fn assert_valid(drawing: &pipeline::Drawing, boxes: &[Rect], edges: &[(usize, usize)]) {
    let ends = drawing.routes.iter().zip(&drawing.ports).zip(edges);
    for ((route, (source, target)), &(from, to)) in ends {
        check::orthogonal(route, &drawing.boxes)
            .unwrap_or_else(|v| panic!("invalid route {route:?}: {v:?}"));
        let (first, last) = (route[0], route[route.len() - 1]);
        if drawing.full {
            // Ports move with their boxes; `drawing.ports` keeps the assigned positions.
            assert!(
                drawing.boxes[from].boundary_contains(first),
                "route starts on its source box"
            );
            assert!(
                drawing.boxes[to].boundary_contains(last),
                "route ends on its target box"
            );
        } else {
            assert_eq!(first, source.point, "route starts at its source port");
            assert_eq!(last, target.point, "route ends at its target port");
        }
    }

    if drawing.full {
        for (old, new) in boxes.iter().zip(&drawing.boxes) {
            assert!(
                new.width() >= old.width() && new.height() >= old.height(),
                "{old:?} shrank to {new:?}"
            );
        }

        for (i, a) in drawing.boxes.iter().enumerate() {
            for b in &drawing.boxes[i + 1..] {
                assert!(!a.interiors_overlap(b), "boxes {a:?} and {b:?} overlap");
            }
        }
    }
}

fuzz_target!(|data: &[u8]| {
    let mut bytes = Bytes(data.iter());
    let boxes = boxes(&mut bytes);
    let config = bytes.config();
    let edges: Vec<(usize, usize)> = (0..=bytes.below(MAX_EDGES))
        .map(|_| (bytes.below(boxes.len()), bytes.below(boxes.len())))
        .collect();

    if let Ok(drawing) = pipeline::draw(&boxes, &edges, &config) {
        assert_eq!(drawing.routes.len(), edges.len(), "one route per edge");
        assert_valid(&drawing, &boxes, &edges);
    }
});
