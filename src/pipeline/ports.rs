//! Port assignment (arXiv 2309.01671, section 2).
//!
//! Each edge leaves its boxes through the sides that the straight segment between the
//! box centres crosses. Where that segment crosses a side in its outer quarter, the edge
//! becomes an L instead (one port moves to the adjacent side), bending away from the
//! barycentre of all box centres. Ports on a side follow the circular order of their
//! edges around the box and are spread evenly; parallel edges get mirrored orders at
//! their two ends so they do not cross. Self-loops get neighbouring ports on the least
//! populated side of their box.

use alloc::collections::BTreeMap;
use alloc::vec::Vec;
use core::cmp::Ordering;

use crate::geom::{Point, Rect, Side};
use crate::scene::Port;

/// One port request: an edge end on a side of a box.
#[derive(Debug, Clone, Copy)]
struct End {
    /// The edge index.
    edge: usize,
    /// Whether this is the edge's target end.
    target: bool,
    /// The box.
    rect: usize,
    /// The side.
    side: Side,
    /// The point the edge heads to from this box (the other box centre), for ordering.
    toward: Point,
    /// The tie-break among parallel edges: the edge index, negated at one end.
    tie: i64,
}

/// Assigns ports to every edge between `boxes`. Returns `(source, target)` ports in
/// edge order, each pointing out of its box.
///
/// # Panics
///
/// Panics if an edge refers to a box index out of range.
#[must_use]
pub fn assign_ports(boxes: &[Rect], edges: &[(usize, usize)]) -> Vec<(Port, Port)> {
    let barycentre = barycentre(boxes);
    let mut ends = Vec::with_capacity(2 * edges.len());
    let mut loops = Vec::new();
    for (edge, &(u, v)) in edges.iter().enumerate() {
        if u == v {
            loops.push((edge, u));
            continue;
        }

        let (su, sv) = sides(&boxes[u], &boxes[v], barycentre);
        let tie = i64::try_from(edge).expect("fewer than 2^63 edges");
        let (tu, tv) = if u < v { (tie, -tie) } else { (-tie, tie) };
        ends.push(End {
            edge,
            target: false,
            rect: u,
            side: su,
            toward: boxes[v].center(),
            tie: tu,
        });
        ends.push(End {
            edge,
            target: true,
            rect: v,
            side: sv,
            toward: boxes[u].center(),
            tie: tv,
        });
    }

    for (edge, rect) in loops {
        let side = least_populated(&ends, rect);
        let tie = i64::try_from(edge).expect("fewer than 2^63 edges");
        let toward = boxes[rect].center();
        ends.push(End {
            edge,
            target: false,
            rect,
            side,
            toward,
            tie: 2 * tie,
        });
        ends.push(End {
            edge,
            target: true,
            rect,
            side,
            toward,
            tie: 2 * tie + 1,
        });
    }

    place(boxes, &ends, edges.len())
}

/// The centroid of the box centres, rounded down.
fn barycentre(boxes: &[Rect]) -> Point {
    let n = i64::try_from(boxes.len().max(1)).expect("fewer than 2^63 boxes");
    let (sx, sy) = boxes.iter().fold((0_i64, 0_i64), |(x, y), b| {
        let c = b.center();
        (x + i64::from(c.x), y + i64::from(c.y))
    });

    Point::new(
        i32::try_from(sx.div_euclid(n)).expect("a mean of i32"),
        i32::try_from(sy.div_euclid(n)).expect("a mean of i32"),
    )
}

/// The side of `a` that the segment from its centre towards `towards` leaves through,
/// and whether it leaves through the outer quarter of that side.
fn exit_side(a: &Rect, towards: Point) -> (Side, bool) {
    let c = a.center();
    let (dx, dy) = (
        i64::from(towards.x) - i64::from(c.x),
        i64::from(towards.y) - i64::from(c.y),
    );
    let (w, h) = (
        i64::try_from(a.width()).expect("width"),
        i64::try_from(a.height()).expect("height"),
    );
    if dx.abs() * h >= dy.abs() * w {
        let side = if dx >= 0 { Side::Right } else { Side::Left };
        // The exit point is `|dy| w / (2 |dx|)` from the side's middle; the outer quarter
        // starts `h / 4` from it.
        let outer = 2 * dy.abs() * w > dx.abs() * h;
        (side, outer)
    } else {
        let side = if dy >= 0 { Side::Top } else { Side::Bottom };
        let outer = 2 * dx.abs() * h > dy.abs() * w;
        (side, outer)
    }
}

/// The sides of the ports of an edge between boxes `a` and `b`.
fn sides(a: &Rect, b: &Rect, barycentre: Point) -> (Side, Side) {
    let (ca, cb) = (a.center(), b.center());
    let (sa, outer_a) = exit_side(a, cb);
    let (sb, outer_b) = exit_side(b, ca);
    if !(outer_a || outer_b) || ca.x == cb.x || ca.y == cb.y {
        return (sa, sb);
    }

    // Two L shapes: horizontal out of `a` then vertical into `b`, bending at
    // (cb.x, ca.y); or vertical out of `a` then horizontal into `b`, at (ca.x, cb.y).
    // Take the bend farther from the barycentre.
    let first = Point::new(cb.x, ca.y);
    let second = Point::new(ca.x, cb.y);
    let horizontal_first =
        first.distance_squared(barycentre) >= second.distance_squared(barycentre);
    let (right, up) = (cb.x > ca.x, cb.y > ca.y);
    if horizontal_first {
        let out = if right { Side::Right } else { Side::Left };
        let into = if up { Side::Bottom } else { Side::Top };
        (out, into)
    } else {
        let out = if up { Side::Top } else { Side::Bottom };
        let into = if right { Side::Left } else { Side::Right };
        (out, into)
    }
}

/// The side of box `rect` with the fewest ends so far, in the order right, top, left,
/// bottom on ties.
fn least_populated(ends: &[End], rect: usize) -> Side {
    *Side::ALL
        .iter()
        .min_by_key(|&&s| {
            ends.iter()
                .filter(|e| e.rect == rect && e.side == s)
                .count()
        })
        .expect("four sides")
}

/// Orders the ends on every side counter-clockwise around the box and spreads them
/// evenly along the side.
fn place(boxes: &[Rect], ends: &[End], edge_count: usize) -> Vec<(Port, Port)> {
    let mut by_side: BTreeMap<(usize, Side), Vec<End>> = BTreeMap::new();
    for e in ends {
        by_side.entry((e.rect, e.side)).or_default().push(*e);
    }

    let mut out =
        alloc::vec![(Port::free(Point::default()), Port::free(Point::default())); edge_count];
    for ((rect, side), mut list) in by_side {
        let r = boxes[rect];
        let c = r.center();
        list.sort_by(|a, b| counter_clockwise(c, a, b));
        let k = i64::try_from(list.len()).expect("few ends");
        for (i, e) in list.iter().enumerate() {
            let i = i64::try_from(i).expect("few ends");
            let point = along_side(&r, side, i + 1, k + 1);
            let port = Port::new(point, side.outward());
            if e.target {
                out[e.edge].1 = port;
            } else {
                out[e.edge].0 = port;
            }
        }
    }

    out
}

/// Compares two ends by the angle of their direction from `c`, counter-clockwise
/// (exactly, by cross products), ties by their tie-break.
fn counter_clockwise(c: Point, a: &End, b: &End) -> Ordering {
    let va = (
        i64::from(a.toward.x) - i64::from(c.x),
        i64::from(a.toward.y) - i64::from(c.y),
    );
    let vb = (
        i64::from(b.toward.x) - i64::from(c.x),
        i64::from(b.toward.y) - i64::from(c.y),
    );
    // Angles measured from the outward normal of the side would be ideal; within one
    // side all directions lie in an open half-plane, so the cross product orders them.
    let cross = va.0 * vb.1 - va.1 * vb.0;

    0.cmp(&cross).then(a.tie.cmp(&b.tie))
}

/// The point at fraction `num / den` along `side` of `r`, counter-clockwise.
fn along_side(r: &Rect, side: Side, num: i64, den: i64) -> Point {
    let lerp = |from: i32, to: i32| {
        let value = i64::from(from) + (i64::from(to) - i64::from(from)) * num / den;
        i32::try_from(value).expect("between two i32")
    };
    match side {
        Side::Right => Point::new(r.max.x, lerp(r.min.y, r.max.y)),
        Side::Top => Point::new(lerp(r.max.x, r.min.x), r.max.y),
        Side::Left => Point::new(r.min.x, lerp(r.max.y, r.min.y)),
        Side::Bottom => Point::new(lerp(r.min.x, r.max.x), r.min.y),
    }
}

#[cfg(test)]
mod tests {
    //! Hand-checked port sides.

    use super::*;

    /// Boxes side by side get facing ports; a diagonal neighbour near a corner gets an L.
    #[test]
    fn sides_follow_centres() {
        let boxes = [
            Rect::new(0, 0, 10, 10),
            Rect::new(40, 0, 50, 10),
            Rect::new(40, 40, 50, 50),
        ];
        let ports = assign_ports(&boxes, &[(0, 1), (0, 2)]);
        assert_eq!(ports[0].0.point, Point::new(10, 5));
        assert_eq!(ports[0].1.point, Point::new(40, 5));
        let (a, b) = ports[1];
        assert!(
            a.point.x == 10 || a.point.y == 10,
            "leaves box 0 on its right or top side"
        );
        assert!(
            b.point.x == 40 || b.point.y == 40,
            "enters box 2 on its left or bottom side"
        );
    }

    /// Parallel edges get mirrored orders at their two ends.
    #[test]
    fn parallel_edges_are_mirrored() {
        let boxes = [Rect::new(0, 0, 10, 30), Rect::new(40, 0, 50, 30)];
        let ports = assign_ports(&boxes, &[(0, 1), (0, 1)]);
        let (a0, b0) = ports[0];
        let (a1, b1) = ports[1];
        assert_eq!(
            a0.point.y > a1.point.y,
            b0.point.y > b1.point.y,
            "no crossing between parallel edges"
        );
    }
}
