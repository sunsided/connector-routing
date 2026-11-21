//! Shared test helpers: random scenes and reference implementations.

use std::collections::{BTreeMap, BTreeSet};

use connector_routing::{DirSet, Direction, Point, Port, Rect};
use proptest::prelude::*;

/// A proptest configuration with `default` cases, unless `PROPTEST_CASES` sets the count
/// (`task test:prop`). `ProptestConfig::with_cases` alone would ignore the variable.
pub fn cases(default: u32) -> ProptestConfig {
    let cases = std::env::var("PROPTEST_CASES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default);

    ProptestConfig::with_cases(cases)
}

/// A random rectangle with corners in `0..=size`.
pub fn rect(size: i32) -> impl Strategy<Value = Rect> {
    (0..size, 0..size, 1..=size / 2, 1..=size / 2)
        .prop_map(move |(x, y, w, h)| Rect::new(x, y, (x + w).min(size + 4), (y + h).min(size + 4)))
}

/// A random non-empty direction set.
pub fn dirs() -> impl Strategy<Value = DirSet> {
    (1_u8..16).prop_map(|bits| {
        Direction::ALL
            .into_iter()
            .enumerate()
            .filter(|(i, _)| bits & (1 << i) != 0)
            .fold(DirSet::NONE, |set, (_, d)| set | d)
    })
}

/// A random port in `0..=size`.
pub fn port(size: i32) -> impl Strategy<Value = Port> {
    (0..=size, 0..=size, dirs()).prop_map(|(x, y, d)| Port::new(Point::new(x, y), d))
}

/// The orthogonal visibility graph straight from the definition of Wybrow et al.
/// (2009, section 3): nodes in `X_I x Y_I` visible from an interesting point on both
/// axes, edges to the nearest node in each direction when nothing intervenes.
pub fn naive_ovg(rects: &[Rect], points: &[Point]) -> (Vec<Point>, Vec<(Point, Point)>) {
    let free = |a: Point, b: Point| !rects.iter().any(|r| r.blocks_segment(a, b));
    let blocked = |p: Point| rects.iter().any(|r| r.interior_contains(p));
    let interesting: Vec<Point> = points.iter().copied().filter(|&p| !blocked(p)).collect();
    let xs: BTreeSet<i32> = interesting.iter().map(|p| p.x).collect();
    let ys: BTreeSet<i32> = interesting.iter().map(|p| p.y).collect();

    let mut nodes = BTreeSet::new();
    for &x in &xs {
        for &y in &ys {
            let p = Point::new(x, y);
            if blocked(p) {
                continue;
            }

            let vertical = interesting.iter().any(|q| q.x == x && free(p, *q));
            let horizontal = interesting.iter().any(|q| q.y == y && free(p, *q));
            if vertical && horizontal {
                nodes.insert(p);
            }
        }
    }

    let mut rows: BTreeMap<i32, Vec<Point>> = BTreeMap::new();
    let mut columns: BTreeMap<i32, Vec<Point>> = BTreeMap::new();
    for &p in &nodes {
        rows.entry(p.y).or_default().push(p);
        columns.entry(p.x).or_default().push(p);
    }

    let mut edges = Vec::new();
    for line in rows.values().chain(columns.values()) {
        for pair in line.windows(2) {
            if free(pair[0], pair[1]) {
                edges.push((pair[0], pair[1]));
            }
        }
    }

    edges.sort();

    (nodes.into_iter().collect(), edges)
}

/// The interesting points of a scene: obstacle corners, port points and the corners of
/// the routing frame (the bounding box grown by one unit).
pub fn interesting(rects: &[Rect], ports: &[Port]) -> Vec<Point> {
    let mut points: Vec<Point> = rects
        .iter()
        .flat_map(Rect::corners)
        .chain(
            ports
                .iter()
                .flat_map(|p| p.interesting_points().collect::<Vec<_>>()),
        )
        .collect();
    if let (Some(x0), Some(x1), Some(y0), Some(y1)) = (
        points.iter().map(|p| p.x).min(),
        points.iter().map(|p| p.x).max(),
        points.iter().map(|p| p.y).min(),
        points.iter().map(|p| p.y).max(),
    ) {
        points.extend(Rect::new(x0 - 1, y0 - 1, x1 + 1, y1 + 1).corners());
    }

    points
}

/// The optimal cost of an orthogonal route on the full unit grid of the bounding box
/// of `points`, padded by `pad`, with unit length cost `length` and bend cost `bend`.
///
/// Uses the router's semantics: the first move leaves `source` in one of its
/// directions, the last move arrives at `target` opposite to one of its directions,
/// moves never reverse, and coincident ports need no move. `None` if unreachable.
pub fn grid_optimum(
    rects: &[Rect],
    points: &[Point],
    source: Port,
    target: Port,
    (length, bend): (u64, u64),
    pad: i32,
) -> Option<u64> {
    use std::cmp::Reverse;
    use std::collections::BinaryHeap;

    if source.point == target.point {
        return Some(0);
    }

    let x0 = points.iter().map(|p| p.x).min()? - pad;
    let x1 = points.iter().map(|p| p.x).max()? + pad;
    let y0 = points.iter().map(|p| p.y).min()? - pad;
    let y1 = points.iter().map(|p| p.y).max()? + pad;
    // A unit step from `p` in `d` stays out of every open interior.
    let step_free = |p: Point, d: Direction| {
        let (dx, dy) = d.delta();
        let q = Point::new(p.x + dx, p.y + dy);
        let (lo, hi) = (
            Point::new(p.x.min(q.x), p.y.min(q.y)),
            Point::new(p.x.max(q.x), p.y.max(q.y)),
        );
        let inside = q.x >= x0 && q.x <= x1 && q.y >= y0 && q.y <= y1;
        let enters = rects.iter().any(|r| {
            if dy == 0 {
                r.min.y < lo.y && lo.y < r.max.y && r.min.x <= lo.x && hi.x <= r.max.x
            } else {
                r.min.x < lo.x && lo.x < r.max.x && r.min.y <= lo.y && hi.y <= r.max.y
            }
        });
        (inside && !enters).then_some(q)
    };

    let mut best: BTreeMap<(Point, usize), u64> = BTreeMap::new();
    let mut heap = BinaryHeap::new();
    for d in source.directions.iter() {
        if let Some(q) = step_free(source.point, d) {
            heap.push(Reverse((length, q, d.index())));
        }
    }

    let arrivals = target.directions.reversed();
    while let Some(Reverse((cost, p, h))) = heap.pop() {
        if best.contains_key(&(p, h)) {
            continue;
        }

        best.insert((p, h), cost);
        let heading = Direction::from_index(h);
        if p == target.point && arrivals.contains(heading) {
            return Some(cost);
        }

        for d in [heading, heading.left(), heading.right()] {
            if let Some(q) = step_free(p, d) {
                let extra = if d == heading { length } else { length + bend };
                heap.push(Reverse((cost + extra, q, d.index())));
            }
        }
    }

    None
}
