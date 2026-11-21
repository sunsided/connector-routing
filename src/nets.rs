//! Routed nets as edge sets of a routing graph: the common input of ordering and
//! nudging, for two-terminal connectors and hyperedge trees alike.

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::geom::{DirSet, Direction};
use crate::graph::{RoutingGraph, VertexId};
use crate::route::Route;

/// One routed net: the graph edges it uses and its terminals.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Net {
    /// For each vertex the net touches, the directions of its edges there.
    pub(crate) arms: BTreeMap<VertexId, DirSet>,
    /// The terminal vertices (ports). Segments through a terminal are fixed.
    pub(crate) terminals: Vec<VertexId>,
}

impl Net {
    /// The net of a connector route.
    #[must_use]
    pub fn from_route(graph: &RoutingGraph, route: &Route) -> Self {
        let mut net = Self::default();
        net.add_path(graph, &route.vertices);
        if let (Some(&first), Some(&last)) = (route.vertices.first(), route.vertices.last()) {
            net.terminals.push(first);
            if last != first {
                net.terminals.push(last);
            }
        }

        net
    }

    /// Adds the edges of a vertex path.
    pub(crate) fn add_path(&mut self, graph: &RoutingGraph, path: &[VertexId]) {
        if let [only] = path {
            self.arms.entry(*only).or_default();
        }

        for pair in path.windows(2) {
            let dir = Direction::between(graph.point(pair[0]), graph.point(pair[1]))
                .expect("consecutive route vertices are aligned");
            *self.arms.entry(pair[0]).or_default() |= DirSet::only(dir);
            *self.arms.entry(pair[1]).or_default() |= DirSet::only(dir.reverse());
        }
    }

    /// The directions of the net's edges at `v`.
    #[must_use]
    pub(crate) fn arms_at(&self, v: VertexId) -> DirSet {
        self.arms.get(&v).copied().unwrap_or_default()
    }

    /// Whether the net uses the edge leaving `v` in direction `dir`.
    #[must_use]
    pub(crate) fn has_edge(&self, v: VertexId, dir: Direction) -> bool {
        self.arms_at(v).contains(dir)
    }

    /// Every edge once, as `(v, dir)` with `dir` either [`Direction::Right`] or
    /// [`Direction::Up`].
    pub(crate) fn edges(&self) -> impl Iterator<Item = (VertexId, Direction)> + '_ {
        self.arms.iter().flat_map(|(&v, &arms)| {
            [Direction::Right, Direction::Up]
                .into_iter()
                .filter(move |&d| arms.contains(d))
                .map(move |d| (v, d))
        })
    }
}
