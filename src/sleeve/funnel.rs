//! The funnel algorithm: the shortest path through a sequence of portals.
//!
//! The funnel is the cone from the apex (the last fixed path vertex) through the
//! narrowest left and right portal ends seen so far. Each portal tightens one side;
//! when a side would cross the other, the crossed side's end becomes the new apex and
//! the scan restarts from it. Exact orientation tests decide every step.

use alloc::vec::Vec;

use super::cdt::orient;
use crate::geom::Point;

/// The shortest path from `start` to `end` through `portals`, each `(left, right)` as
/// seen in the direction of travel. The path's inner vertices are portal ends.
pub(crate) fn funnel(start: Point, end: Point, portals: &[(Point, Point)]) -> Vec<Point> {
    let mut gates: Vec<(Point, Point)> = Vec::with_capacity(portals.len() + 2);
    gates.push((start, start));
    gates.extend_from_slice(portals);
    gates.push((end, end));

    let mut path = alloc::vec![start];
    let (mut apex, mut left, mut right) = (start, start, start);
    let (mut left_i, mut right_i) = (0, 0);
    let mut i = 1;
    while i < gates.len() {
        let (pl, pr) = gates[i];

        // Tighten the right side if the new right end is not outside it.
        if orient(apex, right, pr) >= 0 {
            if apex == right || orient(apex, left, pr) < 0 {
                right = pr;
                right_i = i;
            } else {
                // The right side crosses the left one: the left end is a corner.
                path.push(left);
                apex = left;
                (right, right_i) = (apex, left_i);
                i = left_i + 1;
                continue;
            }
        }

        // Tighten the left side likewise.
        if orient(apex, left, pl) <= 0 {
            if apex == left || orient(apex, right, pl) > 0 {
                left = pl;
                left_i = i;
            } else {
                path.push(right);
                apex = right;
                (left, left_i) = (apex, right_i);
                i = right_i + 1;
                continue;
            }
        }

        i += 1;
    }

    if path.last() != Some(&end) {
        path.push(end);
    }

    path.dedup();

    path
}

#[cfg(test)]
mod tests {
    //! A corridor that turns a corner.

    use super::*;

    /// Through portals that pass a corner at (10, 10), the path bends exactly there.
    #[test]
    fn bends_at_the_corner() {
        let portals = [
            (Point::new(10, 10), Point::new(10, 0)),
            (Point::new(10, 10), Point::new(20, 0)),
        ];
        let path = funnel(Point::new(0, 20), Point::new(20, 20), &portals);
        // The corridor passes below (10, 10), so the path dips to that corner.
        assert_eq!(
            path,
            [Point::new(0, 20), Point::new(10, 10), Point::new(20, 20)]
        );

        let portals = [
            (Point::new(10, 30), Point::new(10, 10)),
            (Point::new(20, 30), Point::new(10, 10)),
        ];
        let path = funnel(Point::new(0, 0), Point::new(20, 0), &portals);
        assert_eq!(
            path,
            [Point::new(0, 0), Point::new(10, 10), Point::new(20, 0)]
        );
    }
}
