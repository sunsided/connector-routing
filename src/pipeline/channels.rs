//! The channel routing graph of arXiv 2309.01671 (section 2, "Routing Graph
//! Construction").
//!
//! For every box `u` and every box `v` entirely to its right, the vertical channel of
//! `(u, v)` is the largest empty rectangle whose vertical sides touch the right side of
//! `u` and the left side of `v`; each box keeps the narrowest such channel to its right.
//! The left and right ends of the drawing count as boxes of zero width and unbounded
//! height. Every channel gets its centre line as a representative; a channel that meets
//! another channel whose extent across contains its own is dropped. Horizontal channels
//! are symmetric. Each port adds a representative from the port outwards until it meets
//! a box or the frame. The graph's vertices are the ports and the crossings of vertical
//! and horizontal representatives; its edges join consecutive vertices along a
//! representative. The graph is a sparse subgraph of the routes the orthogonal
//! visibility graph offers.

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::geom::{Axis, Direction, Point, Rect};
use crate::graph::RoutingGraph;
use crate::scene::Port;

/// A box for channel construction: its range along the channel axis and across it,
/// `None` for unbounded (the frame ends).
#[derive(Debug, Clone, Copy)]
struct Wall {
    /// The low and high coordinate along the axis the channel spans.
    along: (i32, i32),
    /// The low and high coordinate across, `None` if unbounded.
    across: Option<(i32, i32)>,
}

/// One representative line: its line coordinate and closed extent along it.
type Line = (i32, (i32, i32));

/// The channels between `boxes` whose sides face each other along `axis` (`Axis::X`
/// for vertical channels between left and right neighbours), clipped to `frame`, as
/// representative lines across `axis`.
fn channel_lines(boxes: &[Rect], frame: Rect, axis: Axis) -> Vec<Line> {
    let across_axis = axis.other();
    let (lo, hi) = frame.range(axis);
    let mut walls: Vec<Wall> = boxes
        .iter()
        .map(|b| Wall {
            along: b.range(axis),
            across: Some(b.range(across_axis)),
        })
        .collect();
    walls.push(Wall {
        along: (lo, lo),
        across: None,
    });
    walls.push(Wall {
        along: (hi, hi),
        across: None,
    });
    let frame_across = frame.range(across_axis);

    // The narrowest channel to the right of (above) every wall.
    let mut channels: Vec<Rect> = Vec::new();
    for u in &walls {
        let mut best: Option<Rect> = None;
        for v in &walls {
            if v.along.0 < u.along.1 {
                continue;
            }

            let Some(channel) = channel_between(boxes, u, v, frame_across, axis) else {
                continue;
            };
            let width = |r: &Rect| r.range(axis).1 - r.range(axis).0;
            if best.is_none_or(|b| width(&channel) < width(&b)) {
                best = Some(channel);
            }
        }

        channels.extend(best);
    }

    channels.sort();
    channels.dedup();

    // Drop channels that meet another whose extent across contains their own.
    let kept: Vec<Rect> = channels
        .iter()
        .enumerate()
        .filter(|&(i, c)| {
            !channels.iter().enumerate().any(|(j, d)| {
                let (c0, c1) = c.range(across_axis);
                let (d0, d1) = d.range(across_axis);
                let contained =
                    d0 <= c0 && c1 <= d1 && (d0, d1) != (c0, c1) || ((d0, d1) == (c0, c1) && j < i);
                i != j && c.interiors_overlap(d) && contained
            })
        })
        .map(|(_, c)| *c)
        .collect();

    kept.iter()
        .map(|c| {
            let (a, b) = c.range(axis);
            (crate::geom::midpoint(a, b), c.range(across_axis))
        })
        .collect()
}

/// The largest empty rectangle between walls `u` and `v` (with `v` beyond `u` along
/// `axis`) whose sides touch both, or `None`.
fn channel_between(
    boxes: &[Rect],
    u: &Wall,
    v: &Wall,
    frame: (i32, i32),
    axis: Axis,
) -> Option<Rect> {
    let (x0, x1) = (u.along.1, v.along.0);
    if x0 >= x1 {
        return None;
    }

    // Both walls' extents across must overlap.
    let (ua, va) = (u.across.unwrap_or(frame), v.across.unwrap_or(frame));
    let (mut lo, mut hi) = (ua.0.max(va.0), ua.1.min(va.1));
    if lo >= hi {
        return None;
    }

    // The boxes inside the open strip between the walls.
    let strip: Vec<(i32, i32)> = boxes
        .iter()
        .filter(|b| {
            let (a, c) = b.range(axis);
            a < x1 && x0 < c
        })
        .map(|b| b.range(axis.other()))
        .collect();
    if strip.iter().any(|&(a, c)| a < hi && lo < c) {
        return None;
    }

    // Grow across until a box in the strip or the frame stops the channel.
    lo = strip
        .iter()
        .filter(|&&(_, c)| c <= lo)
        .map(|&(_, c)| c)
        .max()
        .unwrap_or(frame.0)
        .max(frame.0);
    hi = strip
        .iter()
        .filter(|&&(a, _)| a >= hi)
        .map(|&(a, _)| a)
        .min()
        .unwrap_or(frame.1)
        .min(frame.1);
    let rect = match axis {
        Axis::X => Rect::try_new(x0, lo, x1, hi),
        Axis::Y => Rect::try_new(lo, x0, hi, x1),
    }?;

    Some(rect)
}

/// The representative of a port: the segment from the port in direction `dir` until a
/// box interior or the frame, as `(axis it runs along, (line, extent))`.
#[expect(
    clippy::many_single_char_names,
    reason = "point and interval bounds in the usual notation"
)]
fn port_line(boxes: &[Rect], frame: Rect, port: &Port, dir: Direction) -> (Axis, Line) {
    let axis = dir.axis();
    let p = port.point;
    let (lo, hi) = frame.range(axis);
    let along = p.coordinate(axis);
    let line = p.coordinate(axis.other());
    let mut end = if dir.is_positive() { hi } else { lo };
    for b in boxes {
        let (a, c) = b.range(axis.other());
        if !(a < line && line < c) {
            continue;
        }

        let (s, t) = b.range(axis);
        if dir.is_positive() && s >= along {
            end = end.min(s);
        } else if !dir.is_positive() && t <= along {
            end = end.max(t);
        }
    }

    (axis, (line, (along.min(end), along.max(end))))
}

/// Merges overlapping or touching intervals on each line.
fn merge(lines: Vec<Line>) -> BTreeMap<i32, Vec<(i32, i32)>> {
    let mut by_line: BTreeMap<i32, Vec<(i32, i32)>> = BTreeMap::new();
    for (line, extent) in lines {
        by_line.entry(line).or_default().push(extent);
    }

    for list in by_line.values_mut() {
        list.sort_unstable();
        let mut out: Vec<(i32, i32)> = Vec::new();
        for (a, b) in list.drain(..) {
            match out.last_mut() {
                Some(last) if a <= last.1 => last.1 = last.1.max(b),
                _ => out.push((a, b)),
            }
        }

        *list = out;
    }

    by_line
}

/// The channel routing graph for `boxes` and `ports` inside `frame` (which must contain
/// everything).
///
/// # Examples
///
/// ```
/// use connector_routing::{DirSet, Point, Port, Rect, pipeline};
///
/// let boxes = [Rect::new(0, 0, 10, 10), Rect::new(30, 0, 40, 10)];
/// let ports = [Port::new(Point::new(10, 5), DirSet::RIGHT), Port::new(Point::new(30, 5), DirSet::LEFT)];
/// let graph = pipeline::channel_graph(&boxes, &ports, Rect::new(-10, -10, 50, 20));
/// assert!(graph.vertex_at(Point::new(10, 5)).is_some());
/// assert!(graph.vertex_at(Point::new(20, 5)).is_some(), "the channel between the boxes");
/// ```
#[must_use]
pub fn channel_graph(boxes: &[Rect], ports: &[Port], frame: Rect) -> RoutingGraph {
    let mut vertical = channel_lines(boxes, frame, Axis::X);
    let mut horizontal = channel_lines(boxes, frame, Axis::Y);
    for port in ports {
        for dir in port.directions.iter() {
            match port_line(boxes, frame, port, dir) {
                (Axis::X, line) => horizontal.push(line),
                (Axis::Y, line) => vertical.push(line),
            }
        }
    }

    let vertical = merge(vertical);
    let horizontal = merge(horizontal);
    let mut graph = RoutingGraph::new();
    let mut points: BTreeMap<Point, crate::graph::VertexId> = BTreeMap::new();
    let mut add =
        |graph: &mut RoutingGraph, p: Point| *points.entry(p).or_insert_with(|| graph.insert(p));
    for port in ports {
        add(&mut graph, port.point);
    }

    for (&x, extents) in &vertical {
        for &(y0, y1) in extents {
            for (&y, h) in horizontal.range(y0..=y1) {
                if h.iter().any(|&(a, b)| a <= x && x <= b) {
                    add(&mut graph, Point::new(x, y));
                }
            }
        }
    }

    link_lines(&mut graph, &vertical, Axis::Y, boxes);
    link_lines(&mut graph, &horizontal, Axis::X, boxes);

    graph
}

/// Links consecutive vertices along every line extent of one orientation.
fn link_lines(
    graph: &mut RoutingGraph,
    lines: &BTreeMap<i32, Vec<(i32, i32)>>,
    axis: Axis,
    boxes: &[Rect],
) {
    let forward = if axis == Axis::X {
        Direction::Right
    } else {
        Direction::Up
    };
    for (&line, extents) in lines {
        for &(a, b) in extents {
            let mut on: Vec<crate::graph::VertexId> = graph
                .vertices()
                .filter(|&v| {
                    let p = graph.point(v);
                    p.coordinate(axis.other()) == line
                        && a <= p.coordinate(axis)
                        && p.coordinate(axis) <= b
                })
                .collect();
            on.sort_by_key(|&v| graph.point(v).coordinate(axis));
            for pair in on.windows(2) {
                let (p, q) = (graph.point(pair[0]), graph.point(pair[1]));
                let free = !boxes.iter().any(|r| r.blocks_segment(p, q));
                if free && graph.neighbor(pair[0], forward).is_none() {
                    graph.link(pair[0], forward, pair[1]);
                }
            }
        }
    }
}
