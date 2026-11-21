//! Separation constraint generation by scan line (Dwyer, Marriott and Stuckey; as used by
//! Wybrow et al. 2009 and arXiv 2309.01671).
//!
//! In the pass that moves segments along `axis` (the `x` pass moves vertical segments),
//! the objects are those segments and the two sides of every obstacle, ordered by
//! position and, on one line, by the path order. A sweep along the other axis keeps the
//! open objects in that order; each opened object is constrained against its
//! neighbours, and the neighbours of a closed object against each other. Segments occupy
//! closed intervals, obstacle sides the open interior of the obstacle.

use alloc::collections::{BTreeMap, BTreeSet};
use alloc::vec::Vec;

use crate::geom::Rect;

/// An object of one nudging pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Item {
    /// A segment of a routed net, by its index in the pass's segment list.
    Segment(usize),
    /// The low side (`min`) of obstacle `0` along the pass axis.
    ObstacleLow(usize),
    /// The high side (`max`) of obstacle `0` along the pass axis.
    ObstacleHigh(usize),
    /// The low end of the nudging region.
    WallLow,
    /// The high end of the nudging region.
    WallHigh,
    /// A fixed coordinate: a port that a perpendicular segment of the same route ends
    /// at.
    Bound(i32),
}

/// What a constraint separates, which decides its minimum gap.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Kind {
    /// Two segments: at least the separation.
    Segments,
    /// A segment and an obstacle side: at least the separation if there is room,
    /// otherwise zero (routes may touch obstacle boundaries).
    Obstacle,
    /// A segment and the region boundary: zero.
    Wall,
    /// A segment and the far end of a perpendicular segment of the same route: zero, so
    /// no segment of a route reverses its direction (the route keeps its shape).
    Shape,
}

/// A separation constraint `position(right) - position(left) >= gap(kind)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Constraint {
    /// The object with the smaller position.
    pub left: Item,
    /// The object with the larger position.
    pub right: Item,
    /// Which gap applies.
    pub kind: Kind,
}

/// An object to place in the scan.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Object {
    /// What it is.
    pub(crate) item: Item,
    /// Its position along the pass axis.
    pub(crate) position: i32,
    /// Its extent along the sweep axis.
    pub(crate) extent: (i32, i32),
    /// Its rank among segments on the same line (zero for obstacles).
    pub(crate) rank: u32,
}

/// The scan order key: position, then obstacle high sides, segments by rank, obstacle
/// low sides, so a segment on an obstacle's side stays outside it.
type Key = (i64, u8, u32, usize);

/// The key of object `index`.
fn key(object: &Object, index: usize) -> Key {
    let class = match object.item {
        Item::ObstacleHigh(_) => 0,
        Item::Segment(_) => 1,
        Item::ObstacleLow(_) => 2,
        Item::WallLow | Item::WallHigh | Item::Bound(_) => 3,
    };
    let position = match object.item {
        Item::WallLow => i64::MIN,
        Item::WallHigh => i64::MAX,
        _ => i64::from(object.position),
    };

    (position, class, object.rank, index)
}

/// The obstacle sides of a pass along `axis`: `min` and `max` of every obstacle, with the
/// obstacle's extent along the other axis.
pub(crate) fn obstacle_objects(rects: &[Rect], axis: crate::geom::Axis) -> Vec<Object> {
    let mut objects = Vec::with_capacity(2 * rects.len());
    for (i, r) in rects.iter().enumerate() {
        let (lo, hi) = r.range(axis);
        let extent = r.range(axis.other());
        objects.push(Object {
            item: Item::ObstacleLow(i),
            position: lo,
            extent,
            rank: 0,
        });
        objects.push(Object {
            item: Item::ObstacleHigh(i),
            position: hi,
            extent,
            rank: 0,
        });
    }

    objects
}

/// Generates the separation constraints between `objects` by scan line.
///
/// Walls bound everything and are added implicitly.
pub(crate) fn generate(objects: &[Object]) -> Vec<Constraint> {
    // Events at one coordinate: obstacle closes, segment opens, segment closes, obstacle
    // opens, so open obstacle interiors and closed segments overlap correctly.
    let mut events: Vec<(i32, u8, usize)> = Vec::with_capacity(2 * objects.len());
    for (i, object) in objects.iter().enumerate() {
        let is_segment = matches!(object.item, Item::Segment(_));
        let (open, close) = if is_segment { (1, 2) } else { (3, 0) };
        events.push((object.extent.0, open, i));
        events.push((object.extent.1, close, i));
    }

    events.sort_unstable();

    let wall_low = Object {
        item: Item::WallLow,
        position: 0,
        extent: (0, 0),
        rank: 0,
    };
    let wall_high = Object {
        item: Item::WallHigh,
        position: 0,
        extent: (0, 0),
        rank: 0,
    };
    let mut active: BTreeMap<Key, Item> = BTreeMap::new();
    active.insert(key(&wall_low, usize::MAX - 1), Item::WallLow);
    active.insert(key(&wall_high, usize::MAX), Item::WallHigh);

    let mut constraints = BTreeSet::new();
    for (_, kind, i) in events {
        let k = key(&objects[i], i);
        let opening = kind == 1 || kind == 3;
        if opening {
            active.insert(k, objects[i].item);
        }

        let below = active.range(..k).next_back().map(|(_, &item)| item);
        let above = active
            .range((core::ops::Bound::Excluded(k), core::ops::Bound::Unbounded))
            .next()
            .map(|(_, &item)| item);
        if opening {
            for (left, right) in [
                (below, Some(objects[i].item)),
                (Some(objects[i].item), above),
            ] {
                if let (Some(l), Some(r)) = (left, right) {
                    push(&mut constraints, l, r);
                }
            }
        } else {
            active.remove(&k);
            if let (Some(l), Some(r)) = (below, above) {
                push(&mut constraints, l, r);
            }
        }
    }

    constraints.into_iter().collect()
}

/// Adds the constraint between `left` and `right` if at least one of them is a segment.
fn push(constraints: &mut BTreeSet<Constraint>, left: Item, right: Item) {
    let is_segment = |item: Item| matches!(item, Item::Segment(_));
    let is_wall = |item: Item| matches!(item, Item::WallLow | Item::WallHigh);
    let kind = match (is_segment(left), is_segment(right)) {
        (true, true) => Kind::Segments,
        (false, false) => return,
        _ if is_wall(left) || is_wall(right) => Kind::Wall,
        _ => Kind::Obstacle,
    };

    constraints.insert(Constraint { left, right, kind });
}

#[cfg(test)]
mod tests {
    //! A hand-checked scan.

    use super::*;

    /// Three stacked segments on one line between two obstacle sides form a chain.
    #[test]
    fn chain_between_obstacles() {
        let seg = |i: usize, rank: u32| Object {
            item: Item::Segment(i),
            position: 5,
            extent: (0, 10),
            rank,
        };
        let objects = [
            Object {
                item: Item::ObstacleHigh(0),
                position: 0,
                extent: (-5, 15),
                rank: 0,
            },
            seg(0, 0),
            seg(1, 1),
            seg(2, 2),
            Object {
                item: Item::ObstacleLow(1),
                position: 10,
                extent: (-5, 15),
                rank: 0,
            },
        ];
        let constraints = generate(&objects);
        let has = |l: Item, r: Item| constraints.iter().any(|c| c.left == l && c.right == r);
        assert!(has(Item::ObstacleHigh(0), Item::Segment(0)));
        assert!(has(Item::Segment(0), Item::Segment(1)));
        assert!(has(Item::Segment(1), Item::Segment(2)));
        assert!(has(Item::Segment(2), Item::ObstacleLow(1)));
        assert!(!has(Item::Segment(0), Item::Segment(2)));
    }
}
