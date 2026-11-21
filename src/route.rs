//! [`Route`]: an orthogonal connector route through a [`RoutingGraph`].

use alloc::vec::Vec;

use crate::geom::{Direction, Point};
use crate::graph::{RoutingGraph, VertexId};

/// An orthogonal route through a routing graph.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Route {
    /// Every graph vertex the route passes, from source to target. Consecutive vertices
    /// are graph neighbours. A route between coincident ports has one vertex.
    pub vertices: Vec<VertexId>,
    /// The route as a polyline: source, bend points, target. Collinear points are
    /// merged, so consecutive segments alternate between horizontal and vertical.
    pub points: Vec<Point>,
    /// The total length (Manhattan, which equals Euclidean for orthogonal routes).
    pub length: u64,
    /// The number of bends.
    pub bends: u32,
    /// The cost the route was optimised for, in the units of the objective used.
    pub cost: u64,
}

impl Route {
    /// Builds a route from its vertex sequence, computing points, length and bends.
    ///
    /// # Panics
    ///
    /// Panics if `vertices` is empty or two consecutive vertices are not aligned.
    #[must_use]
    pub fn from_vertices(graph: &RoutingGraph, vertices: Vec<VertexId>, cost: u64) -> Self {
        let raw: Vec<Point> = vertices.iter().map(|&v| graph.point(v)).collect();
        let points = simplify(&raw);
        let length = raw.windows(2).map(|w| w[0].manhattan(w[1])).sum();
        let bends = u32::try_from(points.len().saturating_sub(2)).expect("fewer than 2^32 bends");

        Self {
            vertices,
            points,
            length,
            bends,
            cost,
        }
    }

    /// Builds a route from a polyline whose straight runs follow graph edges, walking
    /// the graph to recover every vertex.
    ///
    /// Returns `None` if a point is not a vertex or a run is not a chain of edges.
    #[must_use]
    pub fn from_points(graph: &RoutingGraph, points: &[Point], cost: u64) -> Option<Self> {
        let first = graph.vertex_at(*points.first()?)?;
        let mut vertices = alloc::vec![first];
        let mut v = first;
        for pair in points.windows(2) {
            if pair[0] == pair[1] {
                continue;
            }

            let dir = Direction::between(pair[0], pair[1])?;
            while graph.point(v) != pair[1] {
                v = graph.neighbor(v, dir)?;
                vertices.push(v);
            }
        }

        Some(Self::from_vertices(graph, vertices, cost))
    }

    /// The direction of the first move, or `None` for a single-vertex route.
    #[must_use]
    pub fn first_direction(&self) -> Option<Direction> {
        Direction::between(*self.points.first()?, *self.points.get(1)?)
    }

    /// The direction of the last move, or `None` for a single-vertex route.
    #[must_use]
    pub fn last_direction(&self) -> Option<Direction> {
        let n = self.points.len();

        Direction::between(*self.points.get(n.checked_sub(2)?)?, self.points[n - 1])
    }
}

/// Removes repeated points and merges collinear runs of an orthogonal polyline.
#[must_use]
pub fn simplify(points: &[Point]) -> Vec<Point> {
    let mut out: Vec<Point> = Vec::with_capacity(points.len());
    for &p in points {
        if out.last() == Some(&p) {
            continue;
        }

        if out.len() >= 2 {
            let a = out[out.len() - 2];
            let b = out[out.len() - 1];
            if (a.x == b.x && b.x == p.x) || (a.y == b.y && b.y == p.y) {
                out.pop();
            }
        }

        out.push(p);
    }

    out
}

#[cfg(test)]
mod tests {
    //! Unit tests for polyline simplification.

    use super::*;

    /// Collinear and repeated points disappear; bends stay.
    #[test]
    fn simplify_merges_collinear() {
        let pts = [
            Point::new(0, 0),
            Point::new(2, 0),
            Point::new(2, 0),
            Point::new(5, 0),
            Point::new(5, 3),
            Point::new(5, 7),
        ];
        assert_eq!(
            simplify(&pts),
            [Point::new(0, 0), Point::new(5, 0), Point::new(5, 7)]
        );
    }
}
