//! Channel resources and capacities for negotiated congestion.
//!
//! The `x` axis is cut into *slabs* between consecutive distinct `x` coordinates of the
//! graph's vertices. Within a slab, the obstacle-free `y` intervals are its *gaps*. A
//! horizontal edge crosses one gap per slab it spans, and every horizontal edge in the
//! same gap of the same slab competes for the same tracks, whatever line it is on. The
//! `(slab, gap)` pairs are the resources of horizontal edges; vertical edges use the
//! symmetric construction.

use alloc::vec::Vec;

use crate::geom::{Axis, Direction, Rect};
use crate::graph::{RoutingGraph, VertexId};

/// A capacity that never runs out (a gap reaching infinity).
pub(crate) const UNLIMITED: u32 = u32::MAX;

/// The resources of a routing graph and, for every edge, the resources it uses.
#[derive(Debug, Clone)]
pub(crate) struct Resources {
    /// The capacity of each resource in tracks.
    pub(crate) capacity: Vec<u32>,
    /// For each vertex slot, the edge index of its edge to the right and upwards.
    edge_of: Vec<[u32; 2]>,
    /// CSR offsets into [`Resources::uses`] per edge index.
    offsets: Vec<u32>,
    /// `(resource, width)` pairs of all edges, the width being the length of the edge
    /// inside the resource's slab.
    uses: Vec<(u32, u32)>,
}

/// The marker for "no edge" in [`Resources::edge_of`].
const NO_EDGE: u32 = u32::MAX;

/// The slabs and gaps along one axis.
struct Slabs {
    /// The slab boundaries, sorted and distinct.
    cuts: Vec<i32>,
    /// For each slab, its gaps as `(low, high)` closed intervals across the axis, sorted;
    /// `None` bounds are unbounded.
    gaps: Vec<Vec<(Option<i32>, Option<i32>)>>,
    /// The resource id of the first gap of each slab.
    first: Vec<u32>,
}

impl Slabs {
    /// Builds the slabs along `axis` for the given cuts and obstacles.
    fn new(axis: Axis, cuts: Vec<i32>, rects: &[Rect], next_id: &mut u32) -> Self {
        let mut gaps = Vec::with_capacity(cuts.len().saturating_sub(1));
        let mut first = Vec::with_capacity(gaps.capacity());
        for pair in cuts.windows(2) {
            let slab_gaps = gaps_in(axis, pair[0], pair[1], rects);
            first.push(*next_id);
            *next_id += u32::try_from(slab_gaps.len()).expect("fewer than 2^32 gaps");
            gaps.push(slab_gaps);
        }

        Self { cuts, gaps, first }
    }

    /// The resource of the gap containing `across` in slab `slab`.
    fn resource(&self, slab: usize, across: i32) -> u32 {
        let gaps = &self.gaps[slab];
        let index = gaps
            .iter()
            .position(|&(lo, hi)| lo.is_none_or(|l| l <= across) && hi.is_none_or(|h| across <= h))
            .expect("an edge runs through free space");

        self.first[slab] + u32::try_from(index).expect("fewer than 2^32 gaps")
    }
}

/// The obstacle-free closed intervals across `axis` within the open slab `(a, b)`.
fn gaps_in(axis: Axis, a: i32, b: i32, rects: &[Rect]) -> Vec<(Option<i32>, Option<i32>)> {
    let mut blocked: Vec<(i32, i32)> = rects
        .iter()
        .filter(|r| {
            let (lo, hi) = r.range(axis);
            lo < b && a < hi
        })
        .map(|r| r.range(axis.other()))
        .collect();
    blocked.sort_unstable();

    let mut gaps = Vec::new();
    let mut low: Option<i32> = None;
    for (lo, hi) in blocked {
        if low.is_none_or(|l| l <= lo) {
            gaps.push((low, Some(lo)));
        }

        low = Some(low.map_or(hi, |l| l.max(hi)));
    }

    gaps.push((low, None));

    gaps
}

/// The capacity of a gap in tracks of pitch `separation`: `floor(height / separation) + 1`,
/// unlimited for unbounded gaps.
fn gap_capacity(gap: (Option<i32>, Option<i32>), separation: u32) -> u32 {
    match gap {
        (Some(lo), Some(hi)) => {
            let height = u64::from(hi.abs_diff(lo));
            u32::try_from(height / u64::from(separation.max(1)) + 1).unwrap_or(UNLIMITED - 1)
        }
        _ => UNLIMITED,
    }
}

impl Resources {
    /// Builds the resources of `graph` around `rects` for tracks of pitch `separation`.
    pub(crate) fn new(graph: &RoutingGraph, rects: &[Rect], separation: u32) -> Self {
        let mut xs: Vec<i32> = graph.vertices().map(|v| graph.point(v).x).collect();
        let mut ys: Vec<i32> = graph.vertices().map(|v| graph.point(v).y).collect();
        for cuts in [&mut xs, &mut ys] {
            cuts.sort_unstable();
            cuts.dedup();
        }

        let mut next_id = 0;
        let horizontal = Slabs::new(Axis::X, xs, rects, &mut next_id);
        let vertical = Slabs::new(Axis::Y, ys, rects, &mut next_id);
        let mut capacity = Vec::with_capacity(next_id as usize);
        for slabs in [&horizontal, &vertical] {
            for gaps in &slabs.gaps {
                capacity.extend(gaps.iter().map(|&g| gap_capacity(g, separation)));
            }
        }

        let mut resources = Self {
            capacity,
            edge_of: alloc::vec![[NO_EDGE; 2]; graph.capacity()],
            offsets: alloc::vec![0],
            uses: Vec::new(),
        };
        for (a, b) in graph.edges() {
            resources.add_edge(graph, a, b, &horizontal, &vertical);
        }

        resources
    }

    /// Records the resources of the edge from `a` to `b` (right of or above `a`).
    fn add_edge(
        &mut self,
        graph: &RoutingGraph,
        a: VertexId,
        b: VertexId,
        horizontal: &Slabs,
        vertical: &Slabs,
    ) {
        let (pa, pb) = (graph.point(a), graph.point(b));
        let (slabs, along, across, side) = if pa.y == pb.y {
            (horizontal, (pa.x, pb.x), pa.y, 0)
        } else {
            (vertical, (pa.y, pb.y), pa.x, 1)
        };

        let start = slabs
            .cuts
            .binary_search(&along.0)
            .expect("a vertex coordinate");
        let end = slabs
            .cuts
            .binary_search(&along.1)
            .expect("a vertex coordinate");
        for slab in start..end {
            let width = slabs.cuts[slab + 1].abs_diff(slabs.cuts[slab]);
            self.uses.push((slabs.resource(slab, across), width));
        }

        let index = u32::try_from(self.offsets.len() - 1).expect("fewer than 2^32 edges");
        self.edge_of[a.index()][side] = index;
        self.offsets
            .push(u32::try_from(self.uses.len()).expect("fewer than 2^32 uses"));
    }

    /// The `(resource, width)` pairs of the edge leaving `from` in direction `dir`
    /// towards `to`.
    pub(crate) fn of_edge(&self, from: VertexId, dir: Direction, to: VertexId) -> &[(u32, u32)] {
        let (low, side) = match dir {
            Direction::Right => (from, 0),
            Direction::Left => (to, 0),
            Direction::Up => (from, 1),
            Direction::Down => (to, 1),
        };
        let index = self.edge_of[low.index()][side] as usize;
        let (start, end) = (
            self.offsets[index] as usize,
            self.offsets[index + 1] as usize,
        );

        &self.uses[start..end]
    }

    /// The number of resources.
    pub(crate) fn len(&self) -> usize {
        self.capacity.len()
    }
}

#[cfg(test)]
mod tests {
    //! Hand-checked gaps and capacities.

    use super::*;
    use crate::{Point, Port, Scene};

    /// The resources of the edge from `a` to its right neighbour.
    fn right_edge(resources: &Resources, graph: &RoutingGraph, a: Point) -> Vec<(u32, u32)> {
        let va = graph.vertex_at(a).expect("vertex");
        let vb = graph
            .neighbor(va, Direction::Right)
            .expect("an edge to the right");

        resources.of_edge(va, Direction::Right, vb).to_vec()
    }

    /// Between two boxes stacked with a gap of 10, the channel holds 10 / 4 + 1 tracks,
    /// and two horizontal edges on different lines in it share the resource.
    #[test]
    fn channel_between_boxes() {
        let rects = [Rect::new(0, 0, 20, 10), Rect::new(0, 20, 20, 30)];
        let ports = [
            Port::free(Point::new(-5, 12)),
            Port::free(Point::new(25, 18)),
        ];
        let scene = Scene::build(rects, ports).unwrap();
        let resources = Resources::new(scene.graph(), &rects, 4);

        let low = right_edge(&resources, scene.graph(), Point::new(0, 10));
        let high = right_edge(&resources, scene.graph(), Point::new(0, 20));
        assert_eq!(low.len(), 1);
        assert_eq!(low[0].0, high[0].0, "lines in one gap share the resource");
        assert_eq!(resources.capacity[low[0].0 as usize], 3);
    }

    /// Gaps of a slab are the complement of the blocked intervals.
    #[test]
    fn gaps_complement_blocked() {
        let rects = [
            Rect::new(0, 0, 10, 5),
            Rect::new(0, 3, 10, 8),
            Rect::new(0, 12, 10, 14),
        ];
        let gaps = gaps_in(Axis::X, 2, 4, &rects);
        assert_eq!(
            gaps,
            [(None, Some(0)), (Some(8), Some(12)), (Some(14), None)]
        );
    }
}
