//! Independent route validation.
//!
//! These checks deliberately share no predicate with the routers: segment-obstacle
//! intersection is decided by clipping the segment's parameter interval against the
//! open rectangle with exact rational arithmetic, which works for any segment
//! direction. Use them in tests and to validate routes from other sources.

use alloc::vec::Vec;
use core::cmp::Ordering;

use crate::geom::{Point, Rect};

/// Why a route is invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Violation {
    /// The route has no point.
    Empty,
    /// Segment `segment` (from point `segment` to `segment + 1`) is not axis-parallel,
    /// in a check that requires orthogonal routes.
    NotOrthogonal {
        /// The index of the segment's first point.
        segment: usize,
    },
    /// Segment `segment` enters the open interior of `obstacle`.
    EntersObstacle {
        /// The index of the segment's first point.
        segment: usize,
        /// The index of the obstacle in the checked slice.
        obstacle: usize,
    },
}

/// Checks that `route` is a non-empty orthogonal polyline that enters no obstacle
/// interior. Zero-length segments are allowed.
///
/// # Errors
///
/// The first [`Violation`] found.
///
/// # Examples
///
/// ```
/// use connector_routing::{Point, Rect, check};
///
/// let obstacles = [Rect::new(0, 0, 10, 10)];
/// let around = [Point::new(-1, 5), Point::new(-1, 11), Point::new(11, 11)];
/// assert!(check::orthogonal(&around, &obstacles).is_ok());
///
/// let through = [Point::new(-1, 5), Point::new(11, 5)];
/// assert!(check::orthogonal(&through, &obstacles).is_err());
/// ```
pub fn orthogonal(route: &[Point], obstacles: &[Rect]) -> Result<(), Violation> {
    for (segment, pair) in route.windows(2).enumerate() {
        if pair[0].x != pair[1].x && pair[0].y != pair[1].y {
            return Err(Violation::NotOrthogonal { segment });
        }
    }

    polyline(route, obstacles)
}

/// Checks that `route` is a non-empty polyline (any segment directions) that enters no
/// obstacle interior.
///
/// # Errors
///
/// The first [`Violation`] found.
pub fn polyline(route: &[Point], obstacles: &[Rect]) -> Result<(), Violation> {
    if route.is_empty() {
        return Err(Violation::Empty);
    }

    for (segment, pair) in route.windows(2).enumerate() {
        for (obstacle, rect) in obstacles.iter().enumerate() {
            if segment_enters(pair[0], pair[1], rect) {
                return Err(Violation::EntersObstacle { segment, obstacle });
            }
        }
    }

    if route.len() == 1 {
        for (obstacle, rect) in obstacles.iter().enumerate() {
            if segment_enters(route[0], route[0], rect) {
                return Err(Violation::EntersObstacle {
                    segment: 0,
                    obstacle,
                });
            }
        }
    }

    Ok(())
}

/// The number of proper crossings between two orthogonal polylines: points where one
/// passes from one side of the other to the other side.
///
/// Counts pairs of a horizontal segment of one route and a vertical segment of the
/// other whose relative interiors intersect. Touching endpoints and collinear
/// overlaps are not crossings. Meant for nudged routes, where shared stretches have
/// been separated.
#[must_use]
pub fn orthogonal_crossings(a: &[Point], b: &[Point]) -> usize {
    let mut count = 0;
    for pa in a.windows(2) {
        for pb in b.windows(2) {
            if segments_cross(pa[0], pa[1], pb[0], pb[1])
                || segments_cross(pb[0], pb[1], pa[0], pa[1])
            {
                count += 1;
            }
        }
    }

    count
}

/// Whether horizontal segment `h0 h1` and vertical segment `v0 v1` cross at a point
/// interior to both.
fn segments_cross(h0: Point, h1: Point, v0: Point, v1: Point) -> bool {
    if h0.y != h1.y || v0.x != v1.x || h0.x == h1.x || v0.y == v1.y {
        return false;
    }

    let (x0, x1) = (h0.x.min(h1.x), h0.x.max(h1.x));
    let (y0, y1) = (v0.y.min(v1.y), v0.y.max(v1.y));

    x0 < v0.x && v0.x < x1 && y0 < h0.y && h0.y < y1
}

/// The smallest distance between two parallel, overlapping axis-parallel segments of
/// different routes, over all such pairs; `None` if no pair overlaps.
///
/// Two horizontal segments overlap when their `x` ranges share more than a point
/// (likewise for vertical ones). This is the separation nudging should achieve.
#[must_use]
pub fn min_parallel_separation(routes: &[Vec<Point>]) -> Option<u64> {
    let mut best: Option<u64> = None;
    for (i, a) in routes.iter().enumerate() {
        for b in &routes[i + 1..] {
            for pa in a.windows(2) {
                for pb in b.windows(2) {
                    if let Some(d) = parallel_gap(pa[0], pa[1], pb[0], pb[1]) {
                        best = Some(best.map_or(d, |old| old.min(d)));
                    }
                }
            }
        }
    }

    best
}

/// The distance between two parallel axis-parallel segments whose ranges overlap in
/// more than a point.
fn parallel_gap(a0: Point, a1: Point, b0: Point, b1: Point) -> Option<u64> {
    let horizontal = |u: Point, v: Point| u.y == v.y && u.x != v.x;
    let vertical = |u: Point, v: Point| u.x == v.x && u.y != v.y;
    if horizontal(a0, a1) && horizontal(b0, b1) && overlap_ranges(a0.x, a1.x, b0.x, b1.x) {
        return Some(u64::from(a0.y.abs_diff(b0.y)));
    }

    if vertical(a0, a1) && vertical(b0, b1) && overlap_ranges(a0.y, a1.y, b0.y, b1.y) {
        return Some(u64::from(a0.x.abs_diff(b0.x)));
    }

    None
}

/// Whether the ranges `[min(p, q), max(p, q)]` and `[min(r, s), max(r, s)]` share more
/// than a point.
fn overlap_ranges(p: i32, q: i32, r: i32, s: i32) -> bool {
    p.min(q).max(r.min(s)) < p.max(q).min(r.max(s))
}

/// A bound on the segment parameter `t`, the fraction `num / den` with `den > 0`.
#[derive(Debug, Clone, Copy)]
struct Bound {
    /// The numerator.
    num: i128,
    /// The denominator, positive.
    den: i128,
    /// Whether the bound itself is excluded.
    open: bool,
}

impl Bound {
    /// Compares the values of two bounds exactly.
    fn cmp_value(self, other: Self) -> Ordering {
        (self.num * other.den).cmp(&(other.num * self.den))
    }
}

/// Whether the closed segment `a b` meets the open interior of `rect`.
///
/// The segment is `a + t (b - a)` for `t` in `[0, 1]`; each axis restricts `t` to an
/// open interval, and the segment enters the interior iff the intersection is not
/// empty.
fn segment_enters(a: Point, b: Point, rect: &Rect) -> bool {
    let mut lower = Bound {
        num: 0,
        den: 1,
        open: false,
    };
    let mut upper = Bound {
        num: 1,
        den: 1,
        open: false,
    };
    let axes = [
        (
            i128::from(a.x),
            i128::from(b.x) - i128::from(a.x),
            rect.min.x,
            rect.max.x,
        ),
        (
            i128::from(a.y),
            i128::from(b.y) - i128::from(a.y),
            rect.min.y,
            rect.max.y,
        ),
    ];
    for (start, delta, min, max) in axes {
        let (min, max) = (i128::from(min), i128::from(max));
        if delta == 0 {
            if !(min < start && start < max) {
                return false;
            }

            continue;
        }

        // min < start + t delta < max, solved for t; the bounds swap for delta < 0.
        let (low, high) = if delta > 0 {
            ((min - start, delta), (max - start, delta))
        } else {
            ((start - max, -delta), (start - min, -delta))
        };
        let low = Bound {
            num: low.0,
            den: low.1,
            open: true,
        };
        let high = Bound {
            num: high.0,
            den: high.1,
            open: true,
        };
        if low.cmp_value(lower).is_ge() {
            lower = low;
        }

        if high.cmp_value(upper).is_le() {
            upper = high;
        }
    }

    match lower.cmp_value(upper) {
        Ordering::Less => true,
        Ordering::Equal => !lower.open && !upper.open,
        Ordering::Greater => false,
    }
}

#[cfg(test)]
mod tests {
    //! Unit tests for the independent predicates.

    use super::*;

    /// Diagonal segments through, along and past a corner.
    #[test]
    fn diagonal_segments() {
        let r = Rect::new(0, 0, 10, 10);
        assert!(segment_enters(Point::new(-5, -5), Point::new(15, 15), &r));
        assert!(!segment_enters(Point::new(-5, 5), Point::new(5, 15), &r));
        assert!(!segment_enters(Point::new(-1, 0), Point::new(20, 0), &r));
        assert!(segment_enters(Point::new(5, 5), Point::new(5, 5), &r));
        assert!(!segment_enters(Point::new(10, 5), Point::new(10, 5), &r));
        assert!(segment_enters(Point::new(-4, 6), Point::new(6, -4), &r));
    }

    /// Crossings count interior intersections only.
    #[test]
    fn crossings_ignore_touching() {
        let a = [Point::new(0, 5), Point::new(10, 5)];
        let b = [Point::new(5, 0), Point::new(5, 10)];
        let c = [Point::new(10, 0), Point::new(10, 10)];
        assert_eq!(orthogonal_crossings(&a, &b), 1);
        assert_eq!(orthogonal_crossings(&a, &c), 0);
    }
}
