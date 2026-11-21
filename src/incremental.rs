//! Which committed routes a scene change affects (Wybrow, Marriott and Stuckey 2005;
//! Dwyer, Marriott and Wybrow 2006).
//!
//! A diagram editor or layout engine that moves obstacles needs to reroute only some
//! connectors: those whose route the change makes invalid, and those that a removed or
//! moved-away obstacle may let take a cheaper route. These helpers find both sets from
//! the committed routes alone.

use alloc::vec::Vec;

use crate::geom::{Point, Rect, abs_diff};
use crate::route::Route;
use crate::search::{Weights, cost_of};

/// The indices of routes that enter the open interior of `rect`: they must be rerouted
/// after an obstacle is added or moved there.
///
/// # Examples
///
/// ```
/// use connector_routing::{Point, Rect, Route, incremental};
///
/// let route = Route {
///     vertices: Vec::new(),
///     points: vec![Point::new(0, 5), Point::new(20, 5)],
///     length: 20,
///     bends: 0,
///     cost: 20,
/// };
/// assert_eq!(incremental::invalidated_by(&[route], Rect::new(8, 0, 12, 10)), [0]);
/// ```
#[must_use]
pub fn invalidated_by(routes: &[Route], rect: Rect) -> Vec<usize> {
    routes
        .iter()
        .enumerate()
        .filter(|(_, r)| r.points.windows(2).any(|w| rect.blocks_segment(w[0], w[1])))
        .map(|(i, _)| i)
        .collect()
}

/// The indices of routes that may become cheaper once `region` is free: those whose
/// lower bound through `region` is below their current cost under `weights`.
///
/// The bound is the shortest orthogonal path from source to target through some point
/// of `region`, ignoring obstacles and bends (the Manhattan form of the pruning of
/// Wybrow et al. 2005, section 2.3). Every other route is already optimal after the
/// change if it was before, so only these need rerouting.
#[must_use]
pub fn may_improve(routes: &[Route], region: Rect, weights: Weights) -> Vec<usize> {
    routes
        .iter()
        .enumerate()
        .filter(|(_, r)| {
            let (Some(&u), Some(&v)) = (r.points.first(), r.points.last()) else {
                return false;
            };
            let bound = weights.length.saturating_mul(detour_through(u, v, &region));

            bound < cost_of(weights, r.length, r.bends)
        })
        .map(|(i, _)| i)
        .collect()
}

/// The length of the shortest orthogonal path from `u` to `v` through some point of
/// `region`: the Manhattan distance plus twice the gap between the bounding box of `u`
/// and `v` and the region, per axis.
fn detour_through(u: Point, v: Point, region: &Rect) -> u64 {
    let gap = |a: i32, b: i32, lo: i32, hi: i32| {
        let (low, high) = (a.min(b), a.max(b));
        if hi < low {
            abs_diff(low, hi)
        } else if lo > high {
            abs_diff(lo, high)
        } else {
            0
        }
    };
    let extra =
        gap(u.x, v.x, region.min.x, region.max.x) + gap(u.y, v.y, region.min.y, region.max.y);

    u.manhattan(v) + 2 * extra
}

#[cfg(test)]
mod tests {
    //! Unit tests for the detour bound.

    use super::*;

    /// A region beside the box of the endpoints costs twice the gap.
    #[test]
    fn detour_bound() {
        let region = Rect::new(10, 10, 12, 12);
        assert_eq!(
            detour_through(Point::new(0, 0), Point::new(4, 0), &region),
            4 + 2 * (6 + 10)
        );
        assert_eq!(
            detour_through(Point::new(0, 0), Point::new(20, 20), &region),
            40
        );
    }
}
