//! Fuzzes sleeve routing (constrained Delaunay triangulation plus funnel) on arbitrary
//! obstacles and endpoints.
//!
//! Most coordinates sit on a small grid, so collinear and cocircular points, touching
//! boxes and endpoints on box boundaries are common: they are where the exact
//! orientation and in-circle predicates and the constraint recovery degenerate. One
//! coordinate in eight is any `i32`, so the range check against
//! [`MAX_COORD`](connector_routing::MAX_COORD) and the `i128` in-circle determinant near
//! its limit are exercised too. Construction may fail with an error, but it must never
//! panic.
//!
//! Every route that is found must be a polyline that enters no obstacle interior and runs
//! from its first endpoint to its second, and batched routing must find a route exactly
//! for the pairs that single routing connects.

#![no_main]
#![forbid(unsafe_code)]

use connector_routing::sleeve::SleeveRouter;
use connector_routing::{Point, Rect, check};
use libfuzzer_sys::fuzz_target;

/// The most obstacles, which keeps the triangulation small.
const MAX_OBSTACLES: usize = 6;

/// The most endpoint pairs.
const MAX_PAIRS: usize = 4;

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
}

/// Panics unless `route` is a valid route from `from` to `to` around `obstacles`.
fn assert_valid(route: &[Point], from: Point, to: Point, obstacles: &[Rect]) {
    check::polyline(route, obstacles).unwrap_or_else(|v| panic!("invalid route {route:?}: {v:?}"));
    assert_eq!(
        route.first(),
        Some(&from),
        "route starts at its first endpoint"
    );
    assert_eq!(route.last(), Some(&to), "route ends at its second endpoint");
}

fuzz_target!(|data: &[u8]| {
    let mut bytes = Bytes(data.iter());
    let obstacles: Vec<Rect> = (0..bytes.below(MAX_OBSTACLES + 1))
        .filter_map(|_| bytes.rect())
        .collect();
    let pairs: Vec<(Point, Point)> = (0..=bytes.below(MAX_PAIRS))
        .map(|_| (bytes.point(), bytes.point()))
        .collect();
    let margin = u32::from(bytes.byte());
    let endpoints: Vec<Point> = pairs.iter().flat_map(|&(a, b)| [a, b]).collect();
    let Ok(router) = SleeveRouter::new(&obstacles, &endpoints, margin) else {
        return;
    };

    let batched = router.route_all(&pairs);
    for (&(from, to), route) in pairs.iter().zip(&batched) {
        let single = router.route(from, to);
        assert_eq!(
            route.is_some(),
            single.is_some(),
            "batched and single agree on {from:?} to {to:?}"
        );
        if let Some(route) = route {
            assert_valid(route, from, to, &obstacles);
        }
    }
});
