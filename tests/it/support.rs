//! Shared test helpers: random scenes and reference implementations.

use std::collections::{BTreeMap, BTreeSet};

use connector_routing::{DirSet, Direction, Point, Port, Rect};
use proptest::prelude::*;

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

/// The interesting points of a scene: obstacle corners and port points.
pub fn interesting(rects: &[Rect], ports: &[Port]) -> Vec<Point> {
    rects
        .iter()
        .flat_map(Rect::corners)
        .chain(ports.iter().map(|p| p.point))
        .collect()
}
