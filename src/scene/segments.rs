//! The interesting segments of the orthogonal visibility graph and the graph itself.
//!
//! Each maximal obstacle-free segment through an interesting point is stored once per
//! line, keyed by its line coordinate and start, with the interesting points that
//! generate it and its intersections with perpendicular segments in order. Two maximal
//! segments on one line are equal or disjoint (an obstacle of positive width separates
//! them), so the key is unique.

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::geom::{Axis, Direction, Point, Rect};
use crate::graph::{RoutingGraph, VertexId};

/// The start or end of a segment that is not stopped by any obstacle.
const UNBOUNDED: i64 = 1 << 40;

/// One maximal interesting segment.
#[derive(Debug, Clone, Default)]
struct Segment {
    /// The end coordinate along the segment's axis, at least its start;
    /// [`UNBOUNDED`] if no obstacle stops it.
    end: i64,
    /// The interesting points on the segment. Never empty while stored.
    generators: Vec<Point>,
    /// The graph vertices on the segment, keyed by their coordinate along the axis.
    vertices: BTreeMap<i32, VertexId>,
}

/// The key of a segment: its line coordinate (`y` for horizontal segments) and its
/// start along the axis, `-`[`UNBOUNDED`] if unbounded.
type Key = (i32, i64);

/// All interesting segments along one axis.
#[derive(Debug, Clone, Default)]
struct Lines {
    /// The segments by key.
    map: BTreeMap<Key, Segment>,
}

impl Lines {
    /// The key of the segment on `line` that contains `along`, if any.
    fn find(&self, line: i32, along: i32) -> Option<Key> {
        let (&key, segment) = self
            .map
            .range((line, i64::MIN)..=(line, i64::from(along)))
            .next_back()?;

        (segment.end >= i64::from(along)).then_some(key)
    }
}

/// The interesting segments of both axes and the visibility graph they induce.
#[derive(Debug, Clone, Default)]
pub(super) struct Segments {
    /// Horizontal segments (along [`Axis::X`], keyed by `y`).
    horizontal: Lines,
    /// Vertical segments (along [`Axis::Y`], keyed by `x`).
    vertical: Lines,
    /// The orthogonal visibility graph.
    graph: RoutingGraph,
}

/// The point at coordinate `along` on `line` of a segment along `axis`.
fn point_on(axis: Axis, line: i32, along: i32) -> Point {
    match axis {
        Axis::X => Point::new(along, line),
        Axis::Y => Point::new(line, along),
    }
}

/// The direction of increasing coordinates along `axis`.
fn forward(axis: Axis) -> Direction {
    match axis {
        Axis::X => Direction::Right,
        Axis::Y => Direction::Up,
    }
}

/// The extent `(start, end)` of the maximal obstacle-free segment along `axis` through
/// `p`, which must not lie in any obstacle interior.
fn extent(rects: &[Rect], p: Point, axis: Axis) -> (i64, i64) {
    let line = p.coordinate(axis.other());
    let along = p.coordinate(axis);
    let (mut start, mut end) = (-UNBOUNDED, UNBOUNDED);
    for rect in rects {
        let (lo, hi) = rect.range(axis.other());
        if !(lo < line && line < hi) {
            continue;
        }

        let (a, b) = rect.range(axis);
        if b <= along {
            start = start.max(i64::from(b));
        } else if a >= along {
            end = end.min(i64::from(a));
        }
    }

    (start, end)
}

/// Clamps an extent bound into `i32`.
fn clamp(value: i64) -> i32 {
    i32::try_from(value.clamp(i64::from(i32::MIN), i64::from(i32::MAX))).expect("clamped")
}

impl Segments {
    /// The visibility graph.
    pub(super) fn graph(&self) -> &RoutingGraph {
        &self.graph
    }

    /// The segments along `axis`.
    fn lines(&self, axis: Axis) -> &Lines {
        match axis {
            Axis::X => &self.horizontal,
            Axis::Y => &self.vertical,
        }
    }

    /// The segments along `axis`, mutably.
    fn lines_mut(&mut self, axis: Axis) -> &mut Lines {
        match axis {
            Axis::X => &mut self.horizontal,
            Axis::Y => &mut self.vertical,
        }
    }

    /// Rebuilds everything from the obstacles and the interesting points in one batch.
    pub(super) fn build(&mut self, rects: &[Rect], points: &[Point]) {
        *self = Self::default();
        for &p in points {
            if is_blocked(rects, p) {
                continue;
            }

            for axis in [Axis::X, Axis::Y] {
                self.add_unlinked(rects, p, axis);
            }
        }

        self.intersect_all();
    }

    /// Adds `p` as a generator of its segment along `axis`, creating the segment
    /// without vertices if it does not exist yet.
    fn add_unlinked(&mut self, rects: &[Rect], p: Point, axis: Axis) {
        let line = p.coordinate(axis.other());
        let lines = self.lines_mut(axis);
        if let Some(key) = lines.find(line, p.coordinate(axis)) {
            lines.map.get_mut(&key).expect("found").generators.push(p);
            return;
        }

        let (start, end) = extent(rects, p, axis);
        lines.map.insert(
            (line, start),
            Segment {
                end,
                generators: alloc::vec![p],
                vertices: BTreeMap::new(),
            },
        );
    }

    /// Creates a vertex at every intersection and links consecutive vertices along
    /// every segment.
    fn intersect_all(&mut self) {
        let mut crossings = Vec::new();
        for (&(x, start), vertical) in &self.vertical.map {
            let (lo, hi) = (clamp(start), clamp(vertical.end));
            for (&(y, h_start), horizontal) in
                self.horizontal.map.range((lo, i64::MIN)..=(hi, i64::MAX))
            {
                if h_start <= i64::from(x) && i64::from(x) <= horizontal.end {
                    crossings.push(((y, h_start), (x, start)));
                }
            }
        }

        for (h_key, v_key) in crossings {
            let p = Point::new(v_key.0, h_key.0);
            let v = self.graph.insert(p);
            self.horizontal
                .map
                .get_mut(&h_key)
                .expect("crossing")
                .vertices
                .insert(p.x, v);
            self.vertical
                .map
                .get_mut(&v_key)
                .expect("crossing")
                .vertices
                .insert(p.y, v);
        }

        for axis in [Axis::X, Axis::Y] {
            let chains: Vec<Vec<VertexId>> = self
                .lines(axis)
                .map
                .values()
                .map(|s| s.vertices.values().copied().collect())
                .collect();
            for chain in chains {
                for pair in chain.windows(2) {
                    self.graph.link(pair[0], forward(axis), pair[1]);
                }
            }
        }
    }

    /// Removes every segment that a change of an obstacle with one of the closed
    /// rectangles `regions` can affect, with its vertices. Returns their generators.
    ///
    /// A segment is affected when its line meets a region's closed range across the
    /// axis and its closed extent meets the region's closed range along it. That covers
    /// segments the region blocks, segments it stops, and segments along its sides.
    pub(super) fn take_affected(&mut self, regions: &[Rect]) -> Vec<Point> {
        let mut generators = Vec::new();
        for axis in [Axis::X, Axis::Y] {
            let keys: Vec<Key> = self
                .lines(axis)
                .map
                .iter()
                .filter(|&(&(line, start), segment)| {
                    regions.iter().any(|r| {
                        let (lo, hi) = r.range(axis.other());
                        let (a, b) = r.range(axis);
                        lo <= line
                            && line <= hi
                            && start <= i64::from(b)
                            && i64::from(a) <= segment.end
                    })
                })
                .map(|(&key, _)| key)
                .collect();
            for key in keys {
                generators.extend(self.remove_segment(axis, key).generators);
            }
        }

        generators
    }

    /// Ensures every unblocked point in `points` lies on its segments, computing and
    /// inserting missing segments with their intersections.
    pub(super) fn restore(&mut self, rects: &[Rect], points: &[Point]) {
        for &p in points {
            if is_blocked(rects, p) {
                continue;
            }

            for axis in [Axis::X, Axis::Y] {
                let line = p.coordinate(axis.other());
                let lines = self.lines_mut(axis);
                if let Some(key) = lines.find(line, p.coordinate(axis)) {
                    let segment = lines.map.get_mut(&key).expect("found");
                    if !segment.generators.contains(&p) {
                        segment.generators.push(p);
                    }

                    continue;
                }

                let (start, end) = extent(rects, p, axis);
                self.insert_segment(axis, line, start, end, p);
            }
        }
    }

    /// Removes `p` from the generators of its segments, and removes segments left
    /// without generators.
    pub(super) fn drop_generator(&mut self, p: Point) {
        for axis in [Axis::X, Axis::Y] {
            let lines = self.lines_mut(axis);
            let Some(key) = lines.find(p.coordinate(axis.other()), p.coordinate(axis)) else {
                continue;
            };

            let segment = lines.map.get_mut(&key).expect("found");
            segment.generators.retain(|&g| g != p);
            if segment.generators.is_empty() {
                self.remove_segment(axis, key);
            }
        }
    }

    /// Inserts a new segment generated by `generator`, creating and linking a vertex at
    /// every crossing with a perpendicular segment.
    fn insert_segment(&mut self, axis: Axis, line: i32, start: i64, end: i64, generator: Point) {
        let other = axis.other();
        let crossings: Vec<(Key, i32)> = self
            .lines(other)
            .map
            .range((clamp(start), i64::MIN)..=(clamp(end), i64::MAX))
            .filter(|&(&(_, s), segment)| s <= i64::from(line) && i64::from(line) <= segment.end)
            .map(|(&key, _)| (key, key.0))
            .collect();

        let mut vertices = BTreeMap::new();
        for (key, along) in crossings {
            let v = self.graph.insert(point_on(axis, line, along));
            self.attach(other, key, line, v);
            vertices.insert(along, v);
        }

        let chain: Vec<VertexId> = vertices.values().copied().collect();
        for pair in chain.windows(2) {
            self.graph.link(pair[0], forward(axis), pair[1]);
        }

        self.lines_mut(axis).map.insert(
            (line, start),
            Segment {
                end,
                generators: alloc::vec![generator],
                vertices,
            },
        );
    }

    /// Inserts vertex `v` at coordinate `along` into the chain of the segment `key`
    /// along `axis`, linking it to its predecessor and successor.
    fn attach(&mut self, axis: Axis, key: Key, along: i32, v: VertexId) {
        let segment = self
            .lines_mut(axis)
            .map
            .get_mut(&key)
            .expect("crossed segment");
        let before = segment.vertices.range(..along).next_back().map(|(_, &u)| u);
        let after = segment.vertices.range(along..).next().map(|(_, &u)| u);
        segment.vertices.insert(along, v);
        if let Some(u) = before {
            self.graph.link(u, forward(axis), v);
        }

        if let Some(w) = after {
            self.graph.link(v, forward(axis), w);
        }
    }

    /// Removes segment `key` along `axis` and all its vertices, which also leave the
    /// perpendicular segments through them. Returns the removed segment.
    fn remove_segment(&mut self, axis: Axis, key: Key) -> Segment {
        let segment = self
            .lines_mut(axis)
            .map
            .remove(&key)
            .expect("a stored segment");
        let other = axis.other();
        for (&along, &v) in &segment.vertices {
            let p = point_on(axis, key.0, along);
            let lines = self.lines_mut(other);
            let cross = lines
                .find(p.coordinate(axis), p.coordinate(other))
                .expect("every vertex lies on a perpendicular segment");
            lines
                .map
                .get_mut(&cross)
                .expect("found")
                .vertices
                .remove(&p.coordinate(other));
            self.graph.remove(v);
        }

        segment
    }
}

/// Whether `p` lies in the open interior of any of `rects`.
fn is_blocked(rects: &[Rect], p: Point) -> bool {
    rects.iter().any(|r| r.interior_contains(p))
}
