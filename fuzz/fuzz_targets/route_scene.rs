//! Fuzzes single-connector routing on arbitrary scenes.
//!
//! The input describes a few rectangles and ports. Most coordinates sit on a small grid,
//! so touching and overlapping boxes, ports on box boundaries and coincident ports are
//! common; some are read as full `i32` values, so the range checks against
//! [`MAX_COORD`](connector_routing::MAX_COORD) and the exact `i64`/`i128` arithmetic near
//! the limits are exercised too. Scene edits and routing may fail with an error, but they
//! must never panic.
//!
//! Every route that is found must:
//!
//! - be an orthogonal polyline that enters no obstacle interior;
//! - start at the source port and end at the target port;
//! - report a length equal to the length of its polyline.

#![no_main]
#![forbid(unsafe_code)]

use connector_routing::{
    DirSet, Direction, Objective, Point, Port, PortId, Rect, Route, Scene, Search, check,
};
use libfuzzer_sys::fuzz_target;

/// The most obstacles in one scene, which keeps the visibility graph small.
const MAX_OBSTACLES: usize = 6;

/// The most ports in one scene.
const MAX_PORTS: usize = 5;

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

    /// A coordinate: usually on the grid `[-32, 32)`, one time in eight any `i32`.
    fn coordinate(&mut self) -> i32 {
        let raw = self.byte();
        if raw % 8 == 0 {
            i32::from_le_bytes([self.byte(), self.byte(), self.byte(), self.byte()])
        } else {
            i32::from(raw >> 2) - 32
        }
    }

    /// A point from two coordinates.
    fn point(&mut self) -> Point {
        Point::new(self.coordinate(), self.coordinate())
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

    /// A set of directions from the low four bits of one byte.
    fn directions(&mut self) -> DirSet {
        let bits = self.byte();

        Direction::ALL
            .into_iter()
            .enumerate()
            .filter(|&(i, _)| bits & (1 << i) != 0)
            .fold(DirSet::NONE, |set, (_, dir)| set | DirSet::only(dir))
    }

    /// An objective: one of the exact orders, or small weights.
    fn objective(&mut self) -> Objective {
        match self.byte() % 3 {
            0 => Objective::ShortestThenFewestBends,
            1 => Objective::FewestBendsThenShortest,
            _ => Objective::Weighted {
                length: u64::from(self.byte()),
                bend: u64::from(self.byte()),
            },
        }
    }

    /// A search strategy.
    fn search(&mut self) -> Search {
        if self.byte() % 2 == 0 {
            Search::Ovg
        } else {
            Search::OneBend
        }
    }
}

/// Builds a scene from the input; rejected obstacles and ports are skipped.
fn scene(bytes: &mut Bytes<'_>) -> (Scene, Vec<(PortId, Point)>) {
    let mut scene = Scene::new();
    for _ in 0..bytes.below(MAX_OBSTACLES + 1) {
        if let Some(rect) = bytes.rect() {
            let _ = scene.add_obstacle(rect);
        }
    }

    let mut ports = Vec::new();
    for _ in 0..bytes.below(MAX_PORTS + 1) {
        let point = bytes.point();
        if let Ok(id) = scene.add_port(Port::new(point, bytes.directions())) {
            ports.push((id, point));
        }
    }

    (scene, ports)
}

/// Panics unless `route` is a valid route from `source` to `target` around `obstacles`.
fn assert_valid(route: &Route, source: Point, target: Point, obstacles: &[Rect]) {
    let points = &route.points;
    check::orthogonal(points, obstacles)
        .unwrap_or_else(|v| panic!("invalid route {points:?}: {v:?}"));
    assert_eq!(points.first(), Some(&source), "route starts at the source");
    assert_eq!(points.last(), Some(&target), "route ends at the target");

    let length: u64 = points.windows(2).map(|w| w[0].manhattan(w[1])).sum();
    assert_eq!(route.length, length, "reported length of {points:?}");
}

fuzz_target!(|data: &[u8]| {
    let mut bytes = Bytes(data.iter());
    let (scene, ports) = scene(&mut bytes);
    let obstacles = scene.rects();

    for &(source, from) in &ports {
        for &(target, to) in &ports {
            let objective = bytes.objective();
            let search = bytes.search();
            if let Ok(route) = connector_routing::route(&scene, source, target, objective, search) {
                assert_valid(&route, from, to, &obstacles);
            }
        }
    }
});
