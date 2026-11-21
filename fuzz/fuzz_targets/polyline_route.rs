//! Fuzzes incremental poly-line routing (Wybrow et al. 2005): obstacles and connectors
//! are added, then one more obstacle is added and every connector is rerouted.
//!
//! Coordinates sit on a small grid, so collinear corners, touching and overlapping
//! obstacles and endpoints on obstacle boundaries are common. Edits may fail with an
//! error, but they must never panic.
//!
//! Every committed route must be a polyline that enters no obstacle interior and runs
//! from its source to its target, before and after the extra obstacle.

#![no_main]
#![forbid(unsafe_code)]

use connector_routing::polyline::{self, ConnectorId, PolylineRouter};
use connector_routing::{Point, Rect, check};
use libfuzzer_sys::fuzz_target;

/// The most obstacles before the extra one.
const MAX_OBSTACLES: usize = 5;

/// The most connectors.
const MAX_CONNECTORS: usize = 4;

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

    /// A coordinate on the grid `[-32, 32)`.
    fn coordinate(&mut self) -> i32 {
        i32::from(self.byte() % 64) - 32
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
}

/// Panics unless every connector with a committed route runs from its source to its
/// target around `obstacles`.
fn assert_valid(
    router: &PolylineRouter,
    connectors: &[(ConnectorId, Point, Point)],
    obstacles: &[Rect],
) {
    for &(id, from, to) in connectors {
        let Some(route) = router.route(id) else {
            continue;
        };
        check::polyline(route, obstacles)
            .unwrap_or_else(|v| panic!("invalid route {route:?}: {v:?}"));
        assert_eq!(route.first(), Some(&from), "route starts at its source");
        assert_eq!(route.last(), Some(&to), "route ends at its target");
    }
}

fuzz_target!(|data: &[u8]| {
    let mut bytes = Bytes(data.iter());
    let mut router = PolylineRouter::new(polyline::Config::default());
    let mut obstacles = Vec::new();
    for _ in 0..bytes.below(MAX_OBSTACLES + 1) {
        if let Some(rect) = bytes.rect()
            && router.add_obstacle(rect).is_ok()
        {
            obstacles.push(rect);
        }
    }

    let connectors: Vec<(ConnectorId, Point, Point)> = (0..=bytes.below(MAX_CONNECTORS))
        .filter_map(|_| {
            let (from, to) = (bytes.point(), bytes.point());
            let id = router.add_connector(from, to).ok()?;
            Some((id, from, to))
        })
        .collect();
    assert_valid(&router, &connectors, &obstacles);

    let Some(extra) = bytes.rect() else {
        return;
    };
    if router.add_obstacle(extra).is_err() {
        return;
    }

    obstacles.push(extra);
    for &(id, _, _) in &connectors {
        router.reroute(id);
    }

    assert_valid(&router, &connectors, &obstacles);
});
