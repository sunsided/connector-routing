//! Full nudging (arXiv 2309.01671, section 2, "full nudging"): boxes move too.
//!
//! Every pass also treats the two sides of every box as variables, keeps each box at
//! least its size, keeps boxes, segments and ports at least the separation apart, and
//! ties every port (and the route segment ending at it) to its box: a port on a side
//! across the pass axis moves with that side, a port on a side along it slides along the
//! side, and the box grows if its ports need more room than it has (as in the paper). Ties make the constraint graph cyclic, so the
//! leftmost (most compact) solution comes from Bellman-Ford longest paths over the tied
//! groups instead of one forward pass; a positive cycle means the separation cannot be
//! met and the pass reports failure.
//!
//! Compared with the paper's LP, the solution packs everything towards low coordinates
//! (it minimises every position at once, so also the drawing's extent and every box's
//! growth), instead of the paper's weighted sum.

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::geom::{Axis, Point, Rect};
use crate::graph::RoutingGraph;
use crate::nets::Net;

use super::constraints::{self, Constraint, Item, Kind};
use super::segments::Segments;

/// A node of the tied constraint graph: a segment, a box side, or the origin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Node {
    /// The fixed origin, at coordinate zero.
    Origin,
    /// A segment along the pass axis.
    Segment(usize),
    /// The low side of a box along the position axis.
    Low(usize),
    /// The high side.
    High(usize),
}

/// Union-find with offsets: `position(node) = position(root) + offset(node)`.
#[derive(Debug, Default)]
struct Ties {
    /// Parent and offset to it.
    parent: BTreeMap<Node, (Node, i64)>,
}

impl Ties {
    /// The root of `n` and the offset of `n` from it.
    fn find(&self, n: Node) -> (Node, i64) {
        let mut at = n;
        let mut offset = 0;
        while let Some(&(p, o)) = self.parent.get(&at) {
            offset += o;
            at = p;
        }

        (at, offset)
    }

    /// Ties `a` to be `b + offset`; returns whether that is consistent.
    fn tie(&mut self, a: Node, b: Node, offset: i64) -> bool {
        let (ra, oa) = self.find(a);
        let (rb, ob) = self.find(b);
        if ra == rb {
            return oa == ob + offset;
        }

        // root(a) = a - oa = b + offset - oa = root(b) + ob + offset - oa.
        self.parent.insert(ra, (rb, ob + offset - oa));

        true
    }
}

/// One full pass moving segments along `axis` and box sides across it. Returns `false`
/// (and changes nothing) if the separation cannot be met.
pub(crate) fn pass(
    graph: &RoutingGraph,
    boxes: &mut [Rect],
    nets: &[Net],
    segments: &mut Segments,
    axis: Axis,
    separation: i64,
    extra: &[(usize, usize)],
) -> bool {
    let position_axis = axis.other();
    let objects = super::pass_objects(graph, boxes, nets, segments, axis, extra);
    let mut list: Vec<Constraint> = constraints::generate_with(&objects, true);
    list.extend(super::shape_constraints(graph, nets, segments, axis));

    let mut ties = Ties::default();
    let mut slides = Vec::new();
    if !tie_ports(graph, boxes, nets, segments, axis, &mut ties, &mut slides) {
        return false;
    }

    // Arcs between tied groups: root(r) - root(l) >= gap + offset(l) - offset(r).
    let mut arcs: Vec<(Node, Node, i64)> = Vec::new();
    for &(l, r) in &slides {
        let ((gl, ol), (gr, or)) = (ties.find(l), ties.find(r));
        if gl != gr {
            arcs.push((gl, gr, ol - or));
        }
    }

    let node = |item: Item| -> Option<(Node, i64)> {
        match item {
            Item::Segment(s) => Some(ties.find(Node::Segment(s))),
            Item::ObstacleLow(b) => Some(ties.find(Node::Low(b))),
            Item::ObstacleHigh(b) => Some(ties.find(Node::High(b))),
            Item::Bound(c) => Some((Node::Origin, i64::from(c))),
            Item::WallLow | Item::WallHigh => None,
        }
    };
    for c in &list {
        let gap = match c.kind {
            Kind::Shape => 0,
            Kind::Wall => continue,
            Kind::Segments | Kind::Obstacle | Kind::Boxes => separation,
        };
        let (Some((l, ol)), Some((r, or))) = (node(c.left), node(c.right)) else {
            continue;
        };
        if l != r {
            arcs.push((l, r, gap + ol - or));
        }
    }

    for (b, rect) in boxes.iter().enumerate() {
        let (lo, hi) = rect.range(position_axis);
        let (l, ol) = ties.find(Node::Low(b));
        let (r, or) = ties.find(Node::High(b));
        if l != r {
            arcs.push((l, r, i64::from(hi) - i64::from(lo) + ol - or));
        }
    }

    let Some(solution) = longest_paths(&arcs, start(boxes, segments, axis)) else {
        return false;
    };
    let value = |n: Node| -> i64 {
        let (root, offset) = ties.find(n);
        solution.get(&root).copied().unwrap_or(0) + offset
    };

    let Ok(lines) = (0..segments.list.len())
        .map(|s| {
            if segments.list[s].axis == axis {
                i32::try_from(value(Node::Segment(s)))
            } else {
                Ok(segments.line[s])
            }
        })
        .collect::<Result<Vec<i32>, _>>()
    else {
        return false;
    };
    segments.line = lines;
    for (b, rect) in boxes.iter_mut().enumerate() {
        let (Ok(lo), Ok(hi)) = (
            i32::try_from(value(Node::Low(b))),
            i32::try_from(value(Node::High(b))),
        ) else {
            return false;
        };
        *rect = match position_axis {
            Axis::X => Rect::new(lo, rect.min.y, hi, rect.max.y),
            Axis::Y => Rect::new(rect.min.x, lo, rect.max.x, hi),
        };
    }

    true
}

/// Ties every port segment and terminal point along `axis` to the box its port lies on:
/// a port on a side across the pass axis moves with that side; a port on a side along
/// it may slide along the side (`slides` gets `low side <= port <= high side`), so a box
/// grows when its ports need more room. Free ports stay where they are (tied to the
/// origin). Returns `false` on an inconsistent tie.
fn tie_ports(
    graph: &RoutingGraph,
    boxes: &[Rect],
    nets: &[Net],
    segments: &Segments,
    axis: Axis,
    ties: &mut Ties,
    slides: &mut Vec<(Node, Node)>,
) -> bool {
    let position_axis = axis.other();
    for (s, segment) in segments.list.iter().enumerate() {
        if segment.axis != axis || !segment.fixed {
            continue;
        }

        // A straight route between two ports ties both boxes together.
        for &t in segment
            .vertices
            .iter()
            .filter(|v| nets[segment.net].terminals.contains(v))
        {
            // The port where the previous pass left it.
            let p: Point = segments.position(graph, segment.net, t, nets);
            let c = p.coordinate(position_axis);
            let ok = match boxes.iter().position(|b| b.boundary_contains(p)) {
                Some(b) => {
                    let (lo, hi) = boxes[b].range(position_axis);
                    if c == lo {
                        ties.tie(Node::Segment(s), Node::Low(b), 0)
                    } else if c == hi {
                        ties.tie(Node::Segment(s), Node::High(b), 0)
                    } else {
                        slides.push((Node::Low(b), Node::Segment(s)));
                        slides.push((Node::Segment(s), Node::High(b)));
                        true
                    }
                }
                None => ties.tie(Node::Segment(s), Node::Origin, i64::from(segments.line[s])),
            };
            if !ok {
                return false;
            }
        }
    }

    true
}

/// The coordinate everything packs towards: the lowest position of any box side or
/// segment along the pass's position axis.
fn start(boxes: &[Rect], segments: &Segments, axis: Axis) -> i64 {
    let sides = boxes.iter().map(|b| i64::from(b.range(axis.other()).0));
    let lines = segments
        .list
        .iter()
        .zip(&segments.line)
        .filter(|(s, _)| s.axis == axis)
        .map(|(_, &l)| i64::from(l));

    sides.chain(lines).min().unwrap_or(0)
}

/// The least positions satisfying every arc `x[r] - x[l] >= w`, all at least `floor`
/// and the origin at zero, by Bellman-Ford; `None` on a positive cycle or if the origin
/// would have to move.
fn longest_paths(arcs: &[(Node, Node, i64)], floor: i64) -> Option<BTreeMap<Node, i64>> {
    let mut value: BTreeMap<Node, i64> = BTreeMap::new();
    for &(l, r, _) in arcs {
        for n in [l, r] {
            value
                .entry(n)
                .or_insert(if n == Node::Origin { 0 } else { floor });
        }
    }

    for _ in 0..=value.len() {
        let mut changed = false;
        for &(l, r, w) in arcs {
            let candidate = value[&l] + w;
            if candidate > value[&r] {
                if r == Node::Origin {
                    return None;
                }

                value.insert(r, candidate);
                changed = true;
            }
        }

        if !changed {
            return Some(value);
        }
    }

    None
}
