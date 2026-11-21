//! Nudging: final placement of routed segments (Wybrow et al. 2009, section 5;
//! arXiv 2309.01671).
//!
//! Routes that share graph edges lie on top of each other. Nudging moves every segment
//! that does not end at a port within its channel, so that shared stretches separate,
//! single segments run down the middle of their alley, and bundles spread evenly. It
//! keeps the order of overlapping segments (by the comparator described in `docs/design.md`) and every obstacle
//! side, so routes keep their topology and introduce no crossings.
//!
//! ```mermaid
//! flowchart LR
//!     S[segments of all nets] --> X[x pass: vertical segments]
//!     X --> Y[y pass: horizontal segments]
//!     subgraph pass
//!         O[line order] --> C[scan-line constraints]
//!         C --> G[largest common gap per component]
//!         G --> P[place between extreme solutions]
//!     end
//! ```
//!
//! The constraints are plain data ([`constraints`]), so a layout engine can keep them
//! while moving obstacles (Dwyer, Marriott and Wybrow 2006).

mod constraints;
mod order;
mod segments;
mod solve;

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::geom::{Axis, Direction, Point, Rect};
use crate::graph::{RoutingGraph, VertexId};
use crate::nets::Net;
use crate::route::Route;

pub use constraints::{Constraint, Item, Kind};

use constraints::Object;
use segments::Segments;

/// Parameters of nudging.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// The desired distance between parallel segments of different routes, and between
    /// segments and obstacle sides where there is room.
    pub separation: u32,
    /// The largest gap segments are spread to in channels bounded on both sides; wider
    /// channels keep their bundles centred at this gap. Use `u32::MAX` to spread
    /// segments over the whole channel (arXiv 2309.01671).
    pub max_separation: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            separation: 4,
            max_separation: 8,
        }
    }
}

/// Nudged routes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Nudged {
    /// One polyline per route, point for point like [`Route::points`]. A Z-shaped run
    /// whose middle shrinks to nothing keeps its (now coincident) points; use
    /// [`simplify`](crate::simplify) to drop them.
    pub routes: Vec<Vec<Point>>,
    /// The smallest gap achieved between segments that had to be separated, or `None`
    /// if no segments needed separation. Below [`Config::separation`] only where a
    /// channel is too narrow.
    pub separation: Option<u32>,
    /// The number of pairs of segments of different routes still lying on top of each
    /// other: both fixed by ports on the same line, or in a channel with no room. Zero
    /// when every shared stretch was separated.
    pub overlaps: usize,
}

/// Nudges routes of the orthogonal visibility graph of `graph` around `obstacles`.
///
/// # Examples
///
/// ```
/// use connector_routing::{Connector, DirSet, Point, Port, Rect, Scene, global, nudge};
///
/// let mut scene = Scene::new();
/// scene.add_obstacle(Rect::new(0, 0, 40, 10))?;
/// scene.add_obstacle(Rect::new(0, 30, 40, 40))?;
/// // Two connectors through the channel between the boxes share its middle line.
/// let a = scene.add_port(Port::new(Point::new(0, 10), DirSet::UP))?;
/// let b = scene.add_port(Port::new(Point::new(40, 30), DirSet::DOWN))?;
/// let c = scene.add_port(Port::new(Point::new(4, 10), DirSet::UP))?;
/// let d = scene.add_port(Port::new(Point::new(36, 30), DirSet::DOWN))?;
/// let connectors = [Connector { source: a, target: b }, Connector { source: c, target: d }];
/// let routing = global::route_all(&scene, &connectors, &global::Config::default())?;
///
/// let nudged = nudge::nudge(scene.graph(), &scene.rects(), &routing.routes, &nudge::Config::default());
/// assert!(nudged.separation.is_none_or(|s| s >= 4));
/// # Ok::<(), connector_routing::Error>(())
/// ```
#[must_use]
pub fn nudge(
    graph: &RoutingGraph,
    obstacles: &[Rect],
    routes: &[Route],
    config: &Config,
) -> Nudged {
    let nets: Vec<Net> = routes.iter().map(|r| Net::from_route(graph, r)).collect();
    let (segments, separation) = run(graph, obstacles, &nets, *config);
    let polylines: Vec<Vec<Point>> = routes
        .iter()
        .enumerate()
        .map(|(i, r)| {
            bend_vertices(graph, &r.vertices)
                .into_iter()
                .map(|v| segments.position(graph, i, v, &nets))
                .collect()
        })
        .collect();

    let overlaps = count_overlaps(&polylines);

    Nudged {
        routes: polylines,
        separation: separation.map(|s| u32::try_from(s).unwrap_or(u32::MAX)),
        overlaps,
    }
}

/// Nudges arbitrary nets (connector paths or hyperedge trees) and returns, per net,
/// its segments as `(start, end)` pairs with the smallest achieved separation.
#[must_use]
pub fn nudge_nets(
    graph: &RoutingGraph,
    obstacles: &[Rect],
    nets: &[Net],
    config: &Config,
) -> (Vec<Vec<(Point, Point)>>, Option<u32>) {
    let (segments, separation) = run(graph, obstacles, nets, *config);
    let mut out = alloc::vec![Vec::new(); nets.len()];
    for s in segments.list.iter().filter(|s| s.vertices.len() > 1) {
        let a = segments.position(graph, s.net, s.low(), nets);
        let b = segments.position(graph, s.net, s.high(), nets);
        out[s.net].push((a, b));
    }

    (
        out,
        separation.map(|s| u32::try_from(s).unwrap_or(u32::MAX)),
    )
}

/// The separation constraints of the pass that moves segments along `axis` (the `x`
/// pass moves vertical segments, `axis = Axis::Y`) for the given routes, before any
/// movement. Items index the segments in the order of [`segment_lines`].
#[must_use]
pub fn constraints(
    graph: &RoutingGraph,
    obstacles: &[Rect],
    routes: &[Route],
    axis: Axis,
) -> Vec<Constraint> {
    let nets: Vec<Net> = routes.iter().map(|r| Net::from_route(graph, r)).collect();
    let segments = Segments::new(graph, &nets);
    let objects = pass_objects(graph, obstacles, &nets, &segments, axis, &[]);
    let mut constraints = constraints::generate(&objects);
    constraints.extend(shape_constraints(graph, &nets, &segments, axis));

    constraints
}

/// The constraints that keep every route's shape in the pass moving segments along
/// `axis`: a segment stays on its side of the far end of each perpendicular segment it
/// bends into, so no segment reverses.
fn shape_constraints(
    graph: &RoutingGraph,
    nets: &[Net],
    segments: &Segments,
    axis: Axis,
) -> Vec<Constraint> {
    let mut out = Vec::new();
    for (id, s) in segments.list.iter().enumerate() {
        if s.axis != axis || s.fixed {
            continue;
        }

        // Every vertex of the run, so branches attached mid-run (hyperedge junctions)
        // keep their direction too.
        for &end in &s.vertices {
            let across = s.forward().left();
            for dir in [across, across.reverse()] {
                if !nets[s.net].has_edge(end, dir) {
                    continue;
                }

                let Some(p) = segments.of_edge(graph, s.net, end, dir) else {
                    continue;
                };
                let perpendicular = &segments.list[p];
                let far = if perpendicular.low() == end {
                    perpendicular.high()
                } else if perpendicular.high() == end {
                    perpendicular.low()
                } else {
                    // The perpendicular run passes through: no end to keep apart from.
                    continue;
                };
                let other = far_item(graph, nets, segments, p, far);
                let (left, right) = if dir.is_positive() {
                    (Item::Segment(id), other)
                } else {
                    (other, Item::Segment(id))
                };
                out.push(Constraint {
                    left,
                    right,
                    kind: Kind::Shape,
                });
            }
        }
    }

    out
}

/// What holds the far end `far` of perpendicular segment `p`: the segment of the same
/// net it bends into there, or the fixed coordinate of `far`.
fn far_item(
    graph: &RoutingGraph,
    nets: &[Net],
    segments: &Segments,
    p: usize,
    far: VertexId,
) -> Item {
    let perpendicular = &segments.list[p];
    let across = perpendicular.forward().left();
    for dir in [across, across.reverse()] {
        if nets[perpendicular.net].has_edge(far, dir) {
            if let Some(q) = segments.of_edge(graph, perpendicular.net, far, dir) {
                return Item::Segment(q);
            }
        }
    }

    Item::Bound(graph.point(far).coordinate(perpendicular.axis))
}

/// The segments of the routes as `(route, axis, line coordinate)`, in the index order
/// used by [`constraints`].
#[must_use]
pub fn segment_lines(graph: &RoutingGraph, routes: &[Route]) -> Vec<(usize, Axis, i32)> {
    let nets: Vec<Net> = routes.iter().map(|r| Net::from_route(graph, r)).collect();
    let segments = Segments::new(graph, &nets);

    segments
        .list
        .iter()
        .zip(&segments.line)
        .map(|(s, &l)| (s.net, s.axis, l))
        .collect()
}

/// The number of pairs of collinear segments of different polylines that overlap in
/// more than a point.
fn count_overlaps(polylines: &[Vec<Point>]) -> usize {
    let mut count = 0;
    for (i, a) in polylines.iter().enumerate() {
        for b in &polylines[i + 1..] {
            for pa in a.windows(2) {
                for pb in b.windows(2) {
                    count += usize::from(collinear_overlap(pa[0], pa[1], pb[0], pb[1]));
                }
            }
        }
    }

    count
}

/// Whether two axis-parallel segments lie on one line and share more than a point.
fn collinear_overlap(a0: Point, a1: Point, b0: Point, b1: Point) -> bool {
    let overlap = |p: i32, q: i32, r: i32, s: i32| p.min(q).max(r.min(s)) < p.max(q).min(r.max(s));
    if a0.y == a1.y && b0.y == b1.y && a0.y == b0.y {
        return overlap(a0.x, a1.x, b0.x, b1.x);
    }

    a0.x == a1.x && b0.x == b1.x && a0.x == b0.x && overlap(a0.y, a1.y, b0.y, b1.y)
}

/// The vertices of a path where it starts, bends or ends.
fn bend_vertices(graph: &RoutingGraph, path: &[VertexId]) -> Vec<VertexId> {
    let mut out = Vec::new();
    for (i, &v) in path.iter().enumerate() {
        let keep = i == 0
            || i + 1 == path.len()
            || Direction::between(graph.point(path[i - 1]), graph.point(v))
                != Direction::between(graph.point(v), graph.point(path[i + 1]));
        if keep {
            out.push(v);
        }
    }

    out
}

/// Runs both passes and returns the moved segments with the smallest separation.
fn run(
    graph: &RoutingGraph,
    obstacles: &[Rect],
    nets: &[Net],
    config: Config,
) -> (Segments, Option<i64>) {
    let mut segments = Segments::new(graph, nets);
    let first = pass(graph, obstacles, nets, &mut segments, Axis::Y, config, &[]);
    let kisses = kiss_arcs(graph, nets, &segments);
    let second = pass(
        graph,
        obstacles,
        nets,
        &mut segments,
        Axis::X,
        config,
        &kisses,
    );
    let separation = match (first, second) {
        (Some(a), Some(b)) => Some(a.min(b)),
        (a, b) => a.or(b),
    };

    (segments, separation)
}

/// The scan objects of the pass moving segments along `axis`: those segments, with
/// their line order, and the obstacle sides.
fn pass_objects(
    graph: &RoutingGraph,
    obstacles: &[Rect],
    nets: &[Net],
    segments: &Segments,
    axis: Axis,
    extra: &[(usize, usize)],
) -> Vec<Object> {
    let required = segments.terminal_arcs(nets, axis);
    let ranks = order::line_ranks(graph, nets, segments, axis, &required, extra);
    let mut objects = constraints::obstacle_objects(obstacles, axis.other());
    for (id, s) in segments.list.iter().enumerate() {
        if s.axis == axis {
            objects.push(Object {
                item: Item::Segment(id),
                position: segments.line[id],
                extent: segments.extent(graph, nets, id),
                rank: ranks[id],
            });
        }
    }

    objects
}

/// One pass: moves the segments along `axis` across it. Returns the smallest achieved
/// separation.
fn pass(
    graph: &RoutingGraph,
    obstacles: &[Rect],
    nets: &[Net],
    segments: &mut Segments,
    axis: Axis,
    config: Config,
    extra: &[(usize, usize)],
) -> Option<i64> {
    let objects = pass_objects(graph, obstacles, nets, segments, axis, extra);
    let mut constraints = constraints::generate(&objects);
    constraints.extend(shape_constraints(graph, nets, segments, axis));
    let (wall_low, wall_high) = walls(graph, obstacles, segments, axis.other(), config);
    let scan_rank = scan_ranks(&objects, segments.list.len());
    let movable: Vec<bool> = segments
        .list
        .iter()
        .map(|s| s.axis == axis && !s.fixed)
        .collect();
    let lines = segments.line.clone();
    let constant = |item: Item| -> i64 {
        match item {
            Item::Segment(i) => i64::from(lines[i]),
            Item::ObstacleLow(i) => i64::from(obstacles[i].range(axis.other()).0),
            Item::ObstacleHigh(i) => i64::from(obstacles[i].range(axis.other()).1),
            Item::WallLow => wall_low,
            Item::WallHigh => wall_high,
            Item::Bound(c) => i64::from(c),
        }
    };
    let problem = solve::Problem {
        constraints: &constraints,
        constant: &constant,
        movable: &movable,
        order: &scan_rank,
        current: &lines,
        separation: i64::from(config.separation),
        max_separation: i64::from(config.max_separation),
    };
    let solution = problem.solve();
    for (id, &m) in movable.iter().enumerate() {
        if m {
            segments.line[id] =
                i32::try_from(solution.position[id]).expect("positions stay between the walls");
        }
    }

    solution.separation
}

/// The scan rank of every segment: its index in the scan order of all objects.
fn scan_ranks(objects: &[Object], count: usize) -> Vec<usize> {
    let mut keyed: Vec<(i64, u32, usize)> = objects
        .iter()
        .filter_map(|o| match o.item {
            Item::Segment(id) => Some((i64::from(o.position), o.rank, id)),
            _ => None,
        })
        .collect();
    keyed.sort_unstable();
    let mut rank = alloc::vec![0; count];
    for (r, &(_, _, id)) in keyed.iter().enumerate() {
        rank[id] = r;
    }

    rank
}

/// The region walls along `position_axis`: the extent of the graph, obstacles and
/// segments, grown by enough room for every segment at the separation.
fn walls(
    graph: &RoutingGraph,
    obstacles: &[Rect],
    segments: &Segments,
    position_axis: Axis,
    config: Config,
) -> (i64, i64) {
    let coords = graph
        .vertices()
        .map(|v| graph.point(v).coordinate(position_axis))
        .chain(obstacles.iter().flat_map(|r| {
            let (a, b) = r.range(position_axis);
            [a, b]
        }));
    let (lo, hi) = coords.fold((i64::MAX, i64::MIN), |(lo, hi), c| {
        (lo.min(i64::from(c)), hi.max(i64::from(c)))
    });
    let count = i64::try_from(segments.list.len()).unwrap_or(i64::MAX / 4);
    let margin = i64::from(config.separation.max(1)) * (count + 2);

    (lo.saturating_sub(margin), hi.saturating_add(margin))
}

/// The order requirements between horizontal segments that touch end to end at a
/// vertex where both turn away from each other ("kiss"), given the positions of the
/// vertical segments after the `x` pass.
///
/// If net `a` turns at `v` with vertical arm `va` and horizontal arm `ha`, and net `b`
/// with the opposite arms, the two routes cross exactly when `a`'s vertical lies on
/// `b`'s horizontal side and `b`'s horizontal on `a`'s vertical side. So once the `x`
/// pass has put `a`'s vertical on `b`'s horizontal side, `b`'s horizontal must go to the
/// side of `b`'s vertical arm.
fn kiss_arcs(graph: &RoutingGraph, nets: &[Net], segments: &Segments) -> Vec<(usize, usize)> {
    let mut ends: BTreeMap<VertexId, Vec<usize>> = BTreeMap::new();
    for (id, s) in segments.list.iter().enumerate() {
        if s.axis == Axis::X && s.vertices.len() > 1 {
            ends.entry(s.low()).or_default().push(id);
            ends.entry(s.high()).or_default().push(id);
        }
    }

    let mut arcs = Vec::new();
    for (&v, list) in &ends {
        for (i, &a) in list.iter().enumerate() {
            for &b in &list[i + 1..] {
                if let Some(arc) = kiss(graph, nets, segments, v, a, b) {
                    arcs.push(arc);
                }
            }
        }
    }

    arcs
}

/// The order requirement between horizontal segments `a` and `b` kissing at `v`.
fn kiss(
    graph: &RoutingGraph,
    nets: &[Net],
    segments: &Segments,
    v: VertexId,
    a: usize,
    b: usize,
) -> Option<(usize, usize)> {
    let (sa, sb) = (&segments.list[a], &segments.list[b]);
    let (na, nb) = (&nets[sa.net], &nets[sb.net]);
    if sa.net == sb.net || na.terminals.contains(&v) || nb.terminals.contains(&v) {
        return None;
    }

    let arm = |net: &Net, dirs: [Direction; 2]| dirs.into_iter().find(|&d| net.has_edge(v, d));
    let (ha, hb) = (
        arm(na, [Direction::Left, Direction::Right])?,
        arm(nb, [Direction::Left, Direction::Right])?,
    );
    let (va, vb) = (
        arm(na, [Direction::Up, Direction::Down])?,
        arm(nb, [Direction::Up, Direction::Down])?,
    );
    if ha == hb
        || va == vb
        || na.arms_at(v).iter().count() != 2
        || nb.arms_at(v).iter().count() != 2
    {
        return None;
    }

    let xa = segments.line[segments.of_edge(graph, sa.net, v, va)?];
    let xb = segments.line[segments.of_edge(graph, sb.net, v, vb)?];
    let a_on_b_side = if hb == Direction::Right {
        xa > xb
    } else {
        xa < xb
    };
    if !a_on_b_side {
        return None;
    }

    // `b`'s horizontal goes to the side of `b`'s vertical arm: above `a` if it points up.
    Some(if vb == Direction::Up { (a, b) } else { (b, a) })
}
