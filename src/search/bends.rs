//! The exact free-space bend estimate used by every orthogonal A*.

use crate::geom::{DirSet, Direction, Point};

/// The minimum number of bends an orthogonal route needs in the obstacle-free plane to
/// get from `from`, currently moving in `heading`, to `to`, arriving with a heading in
/// `arrivals`. `None` if `arrivals` is empty.
///
/// This is a shortest-path distance in a relaxation of every routing graph (obstacles
/// removed, any bend point allowed, a bend in place included), so it is admissible and
/// consistent. It differs from the `s_d` table of Wybrow et al. (2009, section 4) in five
/// cases; see `docs/design.md`.
///
/// # Examples
///
/// ```
/// use connector_routing::{DirSet, Direction, Point, min_bends};
///
/// let origin = Point::new(0, 0);
/// // Straight ahead: no bend.
/// assert_eq!(min_bends(origin, Direction::Right, Point::new(5, 0), DirSet::RIGHT), Some(0));
/// // Ahead and to the left, arriving upwards: one bend.
/// assert_eq!(min_bends(origin, Direction::Right, Point::new(5, 5), DirSet::UP), Some(1));
/// // Behind, arriving in the current heading: a full loop of four bends.
/// assert_eq!(min_bends(origin, Direction::Right, Point::new(-5, 0), DirSet::RIGHT), Some(4));
/// ```
#[must_use]
pub fn min_bends(from: Point, heading: Direction, to: Point, arrivals: DirSet) -> Option<u32> {
    arrivals
        .iter()
        .map(|arrival| bends_to(from, heading, to, arrival))
        .min()
}

/// [`min_bends`] for one arrival heading.
fn bends_to(from: Point, heading: Direction, to: Point, arrival: Direction) -> u32 {
    let (dx, dy) = rotate_to_right(
        i64::from(to.x) - i64::from(from.x),
        i64::from(to.y) - i64::from(from.y),
        heading,
    );
    let turns = (arrival.index() + 4 - heading.index()) % 4;

    // With the heading rotated to `Right`: 0 = Right, 1 = Up, 2 = Left, 3 = Down.
    match turns {
        0 if dy == 0 && dx >= 0 => 0,
        0 if dx > 0 => 2,
        0 => 4,
        // A bend in place is allowed, so a target straight to the side needs one bend.
        1 if dx >= 0 && dy > 0 => 1,
        3 if dx >= 0 && dy < 0 => 1,
        1 | 3 => 3,
        _ if dy != 0 => 2,
        _ => 4,
    }
}

/// Rotates the vector `(dx, dy)` so that `heading` becomes [`Direction::Right`].
fn rotate_to_right(dx: i64, dy: i64, heading: Direction) -> (i64, i64) {
    match heading {
        Direction::Right => (dx, dy),
        Direction::Up => (dy, -dx),
        Direction::Left => (-dx, -dy),
        Direction::Down => (-dy, dx),
    }
}

#[cfg(test)]
mod tests {
    //! The closed form against breadth-first search over `(cell, heading)` on a free
    //! grid, and against the paper's Fig. 2(a) cases.

    extern crate std;

    use std::collections::VecDeque;
    use std::vec;

    use super::*;

    /// Half-width of the BFS grid; large enough that loops never touch its border.
    const R: i32 = 12;

    /// Bends from every `(cell, heading)` to `target` arriving with `arrival`, by
    /// backward breadth-first search over 0/1 costs (a straight move is free).
    fn bfs(target: Point, arrival: Direction) -> std::vec::Vec<u32> {
        let side = usize::try_from(2 * R + 1).unwrap();
        let index = |p: Point, d: Direction| {
            let x = usize::try_from(p.x + R).unwrap();
            let y = usize::try_from(p.y + R).unwrap();
            (y * side + x) * 4 + d.index()
        };
        let inside = |p: Point| p.x.abs() <= R && p.y.abs() <= R;
        let mut dist = vec![u32::MAX; side * side * 4];
        let mut queue = VecDeque::new();
        dist[index(target, arrival)] = 0;
        queue.push_back((target, arrival));

        // Reverse edges: (p, d) is reached from (p - step(d), e) with cost turns(e, d).
        while let Some((p, d)) = queue.pop_front() {
            let here = dist[index(p, d)];
            let (sx, sy) = d.delta();
            let prev = Point::new(p.x - sx, p.y - sy);
            if !inside(prev) {
                continue;
            }

            for e in Direction::ALL {
                let cost = match e.turns_to(d) {
                    0 => 0,
                    1 => 1,
                    _ => continue,
                };
                let slot = index(prev, e);
                if here + cost < dist[slot] {
                    dist[slot] = here + cost;
                    if cost == 0 {
                        queue.push_front((prev, e));
                    } else {
                        queue.push_back((prev, e));
                    }
                }
            }
        }

        dist
    }

    /// Every start, heading, target and arrival in a small window agrees with BFS.
    #[test]
    fn closed_form_matches_bfs() {
        let side = usize::try_from(2 * R + 1).unwrap();
        for arrival in Direction::ALL {
            let target = Point::new(0, 0);
            let dist = bfs(target, arrival);
            for x in -4..=4 {
                for y in -4..=4 {
                    for heading in Direction::ALL {
                        let p = Point::new(x, y);
                        let ix = usize::try_from(x + R).unwrap();
                        let iy = usize::try_from(y + R).unwrap();
                        let expected = dist[(iy * side + ix) * 4 + heading.index()];
                        // At the target itself BFS needs a non-empty move; a route
                        // that is already there with the right heading needs nothing.
                        let expected = if p == target && heading == arrival {
                            0
                        } else {
                            expected
                        };
                        assert_eq!(
                            bends_to(p, heading, target, arrival),
                            expected,
                            "from {p} heading {heading:?} arriving {arrival:?}"
                        );
                    }
                }
            }
        }
    }

    /// The paper's `s_d` cases 0 to 4 (Fig. 2(a)) where ours agrees, and two where it
    /// differs.
    #[test]
    fn compare_with_paper_table() {
        let o = Point::new(0, 0);
        let d = Direction::Up;
        // 0: same direction, target straight ahead.
        assert_eq!(bends_to(o, d, Point::new(0, 5), Direction::Up), 0);
        // 1: perpendicular arrival, target in a direction of the heading and to that side.
        assert_eq!(bends_to(o, d, Point::new(3, 5), Direction::Right), 1);
        // 2: same direction, target ahead but off the line.
        assert_eq!(bends_to(o, d, Point::new(3, 5), Direction::Up), 2);
        // 2: reversed arrival, target off the line.
        assert_eq!(bends_to(o, d, Point::new(3, 5), Direction::Down), 2);
        // 3: perpendicular arrival, target behind.
        assert_eq!(bends_to(o, d, Point::new(3, -5), Direction::Right), 3);
        // 4: reversed arrival, target on the line behind; same direction, target behind.
        assert_eq!(bends_to(o, d, Point::new(0, -5), Direction::Down), 4);
        assert_eq!(bends_to(o, d, Point::new(3, -5), Direction::Up), 4);
        // Tighter than the paper: perpendicular arrival on the wrong side (paper: 1).
        assert_eq!(bends_to(o, d, Point::new(3, 5), Direction::Left), 3);
        // Below the paper as read: the target exactly beside, one bend in place (paper: 3).
        assert_eq!(bends_to(o, d, Point::new(5, 0), Direction::Right), 1);
    }
}
