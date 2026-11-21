//! Maximal straight runs ("segments") of routed nets and their current geometry.

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::geom::{Axis, Direction, Point};
use crate::graph::{RoutingGraph, VertexId};
use crate::nets::Net;

/// One maximal collinear run of a net's edges.
#[derive(Debug, Clone)]
pub(crate) struct Segment {
    /// The index of the net the segment belongs to.
    pub(crate) net: usize,
    /// The axis the segment runs along: [`Axis::X`] for horizontal segments.
    pub(crate) axis: Axis,
    /// The graph vertices on the run, in increasing coordinate order. At least two.
    pub(crate) vertices: Vec<VertexId>,
    /// Whether the segment passes a terminal, which pins its line coordinate.
    pub(crate) fixed: bool,
}

impl Segment {
    /// The first vertex (smallest coordinate along the axis).
    pub(crate) fn low(&self) -> VertexId {
        self.vertices[0]
    }

    /// The last vertex (largest coordinate along the axis).
    pub(crate) fn high(&self) -> VertexId {
        *self.vertices.last().expect("a segment has two vertices")
    }

    /// The direction from the low end to the high end.
    pub(crate) fn forward(&self) -> Direction {
        match self.axis {
            Axis::X => Direction::Right,
            Axis::Y => Direction::Up,
        }
    }
}

/// All segments of a set of nets, with the current line coordinate of each.
#[derive(Debug, Clone)]
pub(crate) struct Segments {
    /// The segments.
    pub(crate) list: Vec<Segment>,
    /// The current line coordinate of each segment (`y` of horizontal ones); starts at
    /// the graph's coordinate and changes as passes move segments.
    pub(crate) line: Vec<i32>,
    /// The segment of each net edge, keyed by `(net, vertex, Right or Up)`.
    of_edge: BTreeMap<(usize, VertexId, Direction), usize>,
}

impl Segments {
    /// Splits every net into maximal straight runs.
    pub(crate) fn new(graph: &RoutingGraph, nets: &[Net]) -> Self {
        let mut segments = Self {
            list: Vec::new(),
            line: Vec::new(),
            of_edge: BTreeMap::new(),
        };
        for (index, net) in nets.iter().enumerate() {
            for (v, dir) in net.edges() {
                if !segments.of_edge.contains_key(&(index, v, dir)) {
                    segments.add_run(graph, index, net, v, dir);
                }
            }
        }

        for (index, net) in nets.iter().enumerate() {
            for &t in &net.terminals {
                segments.add_terminal_points(graph, index, net, t);
            }
        }

        segments
    }

    /// Adds a fixed single-vertex segment for terminal `t` along each axis the net has
    /// no run along at `t`, so the pass along that axis keeps other segments on the
    /// correct side of the port.
    fn add_terminal_points(&mut self, graph: &RoutingGraph, index: usize, net: &Net, t: VertexId) {
        let arms = net.arms_at(t);
        for axis in [Axis::X, Axis::Y] {
            let along = arms.iter().any(|d| d.axis() == axis);
            if along || arms.is_empty() {
                continue;
            }

            self.line.push(graph.point(t).coordinate(axis.other()));
            self.list.push(Segment {
                net: index,
                axis,
                vertices: alloc::vec![t],
                fixed: true,
            });
        }
    }

    /// The arcs that keep segments on the correct side of terminal points on their line:
    /// a segment of another net through the terminal of net `a` must lie opposite to
    /// `a`'s arm there, or it would cross `a`'s last segment.
    pub(crate) fn terminal_arcs(&self, nets: &[Net], axis: Axis) -> Vec<(usize, usize)> {
        let mut arcs = Vec::new();
        for (p, point) in self.list.iter().enumerate() {
            if point.axis != axis || point.vertices.len() != 1 {
                continue;
            }

            let t = point.vertices[0];
            let Some(arm) = nets[point.net].arms_at(t).iter().next() else {
                continue;
            };
            for (s, segment) in self.list.iter().enumerate() {
                if segment.axis == axis
                    && segment.net != point.net
                    && segment.vertices.len() > 1
                    && segment.vertices.contains(&t)
                {
                    arcs.push(if arm.is_positive() { (s, p) } else { (p, s) });
                }
            }
        }

        arcs
    }

    /// Adds the run through the net edge `(v, dir)`.
    fn add_run(
        &mut self,
        graph: &RoutingGraph,
        index: usize,
        net: &Net,
        v: VertexId,
        dir: Direction,
    ) {
        let mut start = v;
        while let Some(prev) = graph.neighbor(start, dir.reverse()) {
            if !net.has_edge(prev, dir) {
                break;
            }

            start = prev;
        }

        let mut vertices = alloc::vec![start];
        let mut at = start;
        while net.has_edge(at, dir) {
            at = graph.neighbor(at, dir).expect("net edges are graph edges");
            vertices.push(at);
        }

        let id = self.list.len();
        for &u in &vertices[..vertices.len() - 1] {
            self.of_edge.insert((index, u, dir), id);
        }

        let fixed = vertices.iter().any(|u| net.terminals.contains(u));
        let axis = dir.axis();
        self.line.push(graph.point(start).coordinate(axis.other()));
        self.list.push(Segment {
            net: index,
            axis,
            vertices,
            fixed,
        });
    }

    /// The segment of net `net` containing the edge leaving `v` in direction `dir`.
    pub(crate) fn of_edge(
        &self,
        graph: &RoutingGraph,
        net: usize,
        v: VertexId,
        dir: Direction,
    ) -> Option<usize> {
        let (low, forward) = if dir.is_positive() {
            (v, dir)
        } else {
            (graph.neighbor(v, dir)?, dir.reverse())
        };

        self.of_edge.get(&(net, low, forward)).copied()
    }

    /// The current coordinate along its axis of the end of segment `s` at vertex `end`:
    /// the line of the perpendicular segment of the same net through `end`, or the
    /// vertex's own coordinate where the run ends without a bend.
    pub(crate) fn end_coordinate(
        &self,
        graph: &RoutingGraph,
        nets: &[Net],
        s: usize,
        end: VertexId,
    ) -> i32 {
        let segment = &self.list[s];
        let net = &nets[segment.net];
        let across = segment.forward().left();
        for dir in [across, across.reverse()] {
            if net.has_edge(end, dir) {
                if let Some(t) = self.of_edge(graph, segment.net, end, dir) {
                    return self.line[t];
                }
            }
        }

        graph.point(end).coordinate(segment.axis)
    }

    /// The current extent `(low, high)` of segment `s` along its axis.
    pub(crate) fn extent(&self, graph: &RoutingGraph, nets: &[Net], s: usize) -> (i32, i32) {
        let segment = &self.list[s];
        let a = self.end_coordinate(graph, nets, s, segment.low());
        let b = self.end_coordinate(graph, nets, s, segment.high());

        (a.min(b), a.max(b))
    }

    /// The current position of graph vertex `v` as seen by net `net`: the lines of the
    /// net's segments through it.
    pub(crate) fn position(
        &self,
        graph: &RoutingGraph,
        net: usize,
        v: VertexId,
        nets: &[Net],
    ) -> Point {
        let mut p = graph.point(v);
        let arms = nets[net].arms_at(v);
        for dir in arms.iter() {
            if let Some(s) = self.of_edge(graph, net, v, dir) {
                match self.list[s].axis {
                    Axis::X => p.y = self.line[s],
                    Axis::Y => p.x = self.line[s],
                }
            }
        }

        p
    }
}
