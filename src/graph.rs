//! The [`RoutingGraph`]: a partial grid whose vertices have at most one neighbour per
//! [`Direction`].
//!
//! The orthogonal visibility graph is one; the searches, congestion, ordering and
//! nudging rely only on the partial-grid structure, so other builders can plug in.

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::geom::{Direction, Point};

/// A vertex of a [`RoutingGraph`].
///
/// Ids are slots: they stay valid while other vertices are added or removed, and the
/// slot of a removed vertex is reused later. Use [`RoutingGraph::capacity`] to size
/// per-vertex arrays.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertexId(pub(crate) u32);

impl VertexId {
    /// The slot index, below [`RoutingGraph::capacity`].
    #[must_use]
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// The marker for "no neighbour" in [`Slot::neighbors`].
const NONE: u32 = u32::MAX;

/// One vertex slot.
#[derive(Debug, Clone)]
struct Slot {
    /// The vertex position. Meaningless for a free slot.
    point: Point,
    /// The neighbour in each direction, indexed by [`Direction::index`], or [`NONE`].
    neighbors: [u32; 4],
    /// Whether the slot holds a vertex.
    alive: bool,
}

/// A partial grid: points joined by axis-parallel, obstacle-free edges.
///
/// Every vertex has at most one neighbour in each direction, and the edge to it is the
/// segment between the two points. Edges are undirected: `b` is the right neighbour of
/// `a` exactly when `a` is the left neighbour of `b`.
#[derive(Debug, Clone, Default)]
pub struct RoutingGraph {
    /// Vertex slots, live and free.
    slots: Vec<Slot>,
    /// Free slot indices, reused last in, first out.
    free: Vec<u32>,
    /// The vertex at each point.
    by_point: BTreeMap<Point, VertexId>,
}

impl RoutingGraph {
    /// Creates an empty graph.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The number of vertices.
    #[must_use]
    pub fn len(&self) -> usize {
        self.by_point.len()
    }

    /// Whether the graph has no vertices.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.by_point.is_empty()
    }

    /// One more than the largest slot index ever used; the size for per-vertex arrays.
    #[must_use]
    pub fn capacity(&self) -> usize {
        self.slots.len()
    }

    /// The vertex at `point`, if any.
    #[must_use]
    pub fn vertex_at(&self, point: Point) -> Option<VertexId> {
        self.by_point.get(&point).copied()
    }

    /// The position of vertex `v`.
    ///
    /// # Panics
    ///
    /// Panics if `v` is not a vertex of this graph.
    #[must_use]
    pub fn point(&self, v: VertexId) -> Point {
        let slot = &self.slots[v.index()];
        assert!(slot.alive, "vertex {v:?} is not in the graph");

        slot.point
    }

    /// Whether `v` is a live vertex of this graph.
    #[must_use]
    pub fn contains(&self, v: VertexId) -> bool {
        self.slots.get(v.index()).is_some_and(|slot| slot.alive)
    }

    /// The neighbour of `v` in direction `dir`, if any.
    #[must_use]
    pub fn neighbor(&self, v: VertexId, dir: Direction) -> Option<VertexId> {
        let id = self.slots[v.index()].neighbors[dir.index()];

        (id != NONE).then_some(VertexId(id))
    }

    /// Every vertex, in increasing point order.
    pub fn vertices(&self) -> impl Iterator<Item = VertexId> + '_ {
        self.by_point.values().copied()
    }

    /// Every edge once, as `(a, b)` with `b` to the right of or above `a`, in increasing
    /// point order of `a`.
    pub fn edges(&self) -> impl Iterator<Item = (VertexId, VertexId)> + '_ {
        self.vertices().flat_map(move |v| {
            [Direction::Right, Direction::Up]
                .into_iter()
                .filter_map(move |dir| self.neighbor(v, dir).map(|w| (v, w)))
        })
    }

    /// The graph as sorted point and edge lists, independent of vertex ids.
    ///
    /// Two graphs are the same geometric graph exactly when their canonical forms are
    /// equal.
    #[must_use]
    pub fn canonical(&self) -> (Vec<Point>, Vec<(Point, Point)>) {
        let points = self.by_point.keys().copied().collect();
        let edges = self
            .edges()
            .map(|(a, b)| (self.point(a), self.point(b)))
            .collect();

        (points, edges)
    }

    /// Adds an unconnected vertex at `point`, which must not hold a vertex yet.
    pub(crate) fn insert(&mut self, point: Point) -> VertexId {
        debug_assert!(!self.by_point.contains_key(&point), "duplicate vertex");
        let slot = Slot {
            point,
            neighbors: [NONE; 4],
            alive: true,
        };
        let id = if let Some(index) = self.free.pop() {
            self.slots[index as usize] = slot;
            index
        } else {
            let index = u32::try_from(self.slots.len()).expect("fewer than 2^32 vertices");
            self.slots.push(slot);
            index
        };

        let v = VertexId(id);
        self.by_point.insert(point, v);

        v
    }

    /// Makes `b` the neighbour of `a` in direction `dir` (and `a` that of `b` in the
    /// reverse direction), replacing earlier links of both in those directions.
    pub(crate) fn link(&mut self, a: VertexId, dir: Direction, b: VertexId) {
        self.slots[a.index()].neighbors[dir.index()] = b.0;
        self.slots[b.index()].neighbors[dir.reverse().index()] = a.0;
    }

    /// Clears the link of `a` in direction `dir` (and its counterpart).
    pub(crate) fn unlink(&mut self, a: VertexId, dir: Direction) {
        if let Some(b) = self.neighbor(a, dir) {
            self.slots[b.index()].neighbors[dir.reverse().index()] = NONE;
        }

        self.slots[a.index()].neighbors[dir.index()] = NONE;
    }

    /// Removes vertex `v`, joining its left and right (and its lower and upper)
    /// neighbours directly, as removing a point from the middle of a segment does.
    pub(crate) fn remove(&mut self, v: VertexId) {
        for (back, forward) in [
            (Direction::Left, Direction::Right),
            (Direction::Down, Direction::Up),
        ] {
            match (self.neighbor(v, back), self.neighbor(v, forward)) {
                (Some(a), Some(b)) => self.link(a, forward, b),
                (Some(_), None) => self.unlink(v, back),
                (None, Some(_)) => self.unlink(v, forward),
                (None, None) => {}
            }
        }

        let slot = &mut self.slots[v.index()];
        slot.alive = false;
        slot.neighbors = [NONE; 4];
        self.by_point.remove(&slot.point);
        self.free.push(v.0);
    }
}

#[cfg(test)]
mod tests {
    //! Unit tests for linking and removal.

    use super::*;

    /// Removing a middle vertex joins its neighbours; slots are reused.
    #[test]
    fn remove_joins_neighbours() {
        let mut graph = RoutingGraph::new();
        let a = graph.insert(Point::new(0, 0));
        let b = graph.insert(Point::new(5, 0));
        let c = graph.insert(Point::new(9, 0));
        graph.link(a, Direction::Right, b);
        graph.link(b, Direction::Right, c);

        graph.remove(b);
        assert_eq!(graph.neighbor(a, Direction::Right), Some(c));
        assert_eq!(graph.neighbor(c, Direction::Left), Some(a));
        assert_eq!(graph.len(), 2);

        let d = graph.insert(Point::new(1, 1));
        assert_eq!(d, b);
        assert_eq!(graph.canonical().1, [(Point::new(0, 0), Point::new(9, 0))]);
    }
}
