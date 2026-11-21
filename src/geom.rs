//! Exact integer geometry: [`Point`], [`Rect`], [`Direction`], [`DirSet`] and [`Side`].
//!
//! Coordinates are `i32` within [`MAX_COORD`]; every derived quantity (differences,
//! lengths, products) is computed in `i64` or wider, so no predicate can overflow or
//! round.

use core::fmt;
use core::ops::{BitOr, BitOrAssign};

/// The largest absolute coordinate the crate accepts.
///
/// `2^29` keeps coordinate differences within `2^30`, so the in-circle determinant of
/// the constrained Delaunay triangulation (products of four differences, times a small
/// constant) fits into `i128`, and leaves headroom for nudging margins.
pub const MAX_COORD: i32 = 1 << 29;

/// A point with integer coordinates.
///
/// `y` grows "up" in the documentation ([`Direction::Up`] is `+y`). Nothing depends on
/// handedness, so screen coordinates with `y` growing down work unchanged.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub struct Point {
    /// The horizontal coordinate.
    pub x: i32,
    /// The vertical coordinate.
    pub y: i32,
}

impl Point {
    /// Creates a point.
    #[must_use]
    pub const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    /// The Manhattan (L1) distance to `other`.
    ///
    /// # Examples
    ///
    /// ```
    /// use connector_routing::Point;
    ///
    /// assert_eq!(Point::new(0, 0).manhattan(Point::new(3, -4)), 7);
    /// ```
    #[must_use]
    pub fn manhattan(self, other: Self) -> u64 {
        abs_diff(self.x, other.x) + abs_diff(self.y, other.y)
    }

    /// The squared Euclidean distance to `other`, exact.
    #[must_use]
    pub fn distance_squared(self, other: Self) -> u128 {
        let dx = u128::from(abs_diff(self.x, other.x));
        let dy = u128::from(abs_diff(self.y, other.y));

        dx * dx + dy * dy
    }

    /// The coordinate along `axis`.
    #[must_use]
    pub fn coordinate(self, axis: Axis) -> i32 {
        match axis {
            Axis::X => self.x,
            Axis::Y => self.y,
        }
    }

    /// Whether both coordinates lie within [`MAX_COORD`].
    #[must_use]
    pub fn is_in_range(self) -> bool {
        self.x.unsigned_abs() <= MAX_COORD.unsigned_abs()
            && self.y.unsigned_abs() <= MAX_COORD.unsigned_abs()
    }
}

impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

/// `|a - b|` without overflow.
#[must_use]
pub(crate) fn abs_diff(a: i32, b: i32) -> u64 {
    u64::from(a.abs_diff(b))
}

/// One of the two coordinate axes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Axis {
    /// The horizontal axis; horizontal segments run along it.
    X,
    /// The vertical axis; vertical segments run along it.
    Y,
}

impl Axis {
    /// The other axis.
    #[must_use]
    pub const fn other(self) -> Self {
        match self {
            Self::X => Self::Y,
            Self::Y => Self::X,
        }
    }
}

/// An axis-parallel direction of travel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Direction {
    /// Towards larger `x`.
    Right = 0,
    /// Towards larger `y`.
    Up = 1,
    /// Towards smaller `x`.
    Left = 2,
    /// Towards smaller `y`.
    Down = 3,
}

impl Direction {
    /// All four directions, counter-clockwise from [`Direction::Right`].
    pub const ALL: [Self; 4] = [Self::Right, Self::Up, Self::Left, Self::Down];

    /// The direction with index `i` in [`Direction::ALL`]; `i` is taken modulo 4.
    #[must_use]
    pub const fn from_index(i: usize) -> Self {
        Self::ALL[i % 4]
    }

    /// The index of this direction in [`Direction::ALL`].
    #[must_use]
    pub const fn index(self) -> usize {
        self as usize
    }

    /// The opposite direction.
    #[must_use]
    pub const fn reverse(self) -> Self {
        Self::from_index(self.index() + 2)
    }

    /// The direction after a quarter turn to the left (counter-clockwise).
    #[must_use]
    pub const fn left(self) -> Self {
        Self::from_index(self.index() + 1)
    }

    /// The direction after a quarter turn to the right (clockwise).
    #[must_use]
    pub const fn right(self) -> Self {
        Self::from_index(self.index() + 3)
    }

    /// The axis this direction moves along.
    #[must_use]
    pub const fn axis(self) -> Axis {
        match self {
            Self::Right | Self::Left => Axis::X,
            Self::Up | Self::Down => Axis::Y,
        }
    }

    /// Whether the direction moves towards larger coordinates.
    #[must_use]
    pub const fn is_positive(self) -> bool {
        matches!(self, Self::Right | Self::Up)
    }

    /// The unit step `(dx, dy)` of this direction.
    #[must_use]
    pub const fn delta(self) -> (i32, i32) {
        match self {
            Self::Right => (1, 0),
            Self::Up => (0, 1),
            Self::Left => (-1, 0),
            Self::Down => (0, -1),
        }
    }

    /// The direction from `from` to `to` if they differ in exactly one coordinate.
    #[must_use]
    pub fn between(from: Point, to: Point) -> Option<Self> {
        match (to.x.cmp(&from.x), to.y.cmp(&from.y)) {
            (core::cmp::Ordering::Greater, core::cmp::Ordering::Equal) => Some(Self::Right),
            (core::cmp::Ordering::Less, core::cmp::Ordering::Equal) => Some(Self::Left),
            (core::cmp::Ordering::Equal, core::cmp::Ordering::Greater) => Some(Self::Up),
            (core::cmp::Ordering::Equal, core::cmp::Ordering::Less) => Some(Self::Down),
            _ => None,
        }
    }

    /// The number of quarter turns from `self` to `other`: 0 straight, 1 a bend, 2 a
    /// reversal.
    #[must_use]
    pub const fn turns_to(self, other: Self) -> u32 {
        match (other.index() + 4 - self.index()) % 4 {
            0 => 0,
            2 => 2,
            _ => 1,
        }
    }
}

/// A set of [`Direction`]s, such as the directions a port may be left in.
#[derive(Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub struct DirSet(u8);

impl DirSet {
    /// No direction.
    pub const NONE: Self = Self(0);
    /// Every direction.
    pub const ALL: Self = Self(0b1111);
    /// Only [`Direction::Right`].
    pub const RIGHT: Self = Self::only(Direction::Right);
    /// Only [`Direction::Up`].
    pub const UP: Self = Self::only(Direction::Up);
    /// Only [`Direction::Left`].
    pub const LEFT: Self = Self::only(Direction::Left);
    /// Only [`Direction::Down`].
    pub const DOWN: Self = Self::only(Direction::Down);

    /// The set containing only `dir`.
    #[must_use]
    pub const fn only(dir: Direction) -> Self {
        Self(1 << dir.index())
    }

    /// Whether `dir` is in the set.
    #[must_use]
    pub const fn contains(self, dir: Direction) -> bool {
        self.0 & (1 << dir.index()) != 0
    }

    /// Whether the set is empty.
    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// The set with every direction reversed.
    #[must_use]
    pub fn reversed(self) -> Self {
        self.iter().fold(Self::NONE, |set, dir| set | dir.reverse())
    }

    /// The directions in the set, in the order of [`Direction::ALL`].
    pub fn iter(self) -> impl Iterator<Item = Direction> {
        Direction::ALL
            .into_iter()
            .filter(move |&d| self.contains(d))
    }
}

impl BitOr for DirSet {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

impl BitOr<Direction> for DirSet {
    type Output = Self;

    fn bitor(self, rhs: Direction) -> Self {
        self | Self::only(rhs)
    }
}

impl BitOrAssign for DirSet {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

impl From<Direction> for DirSet {
    fn from(dir: Direction) -> Self {
        Self::only(dir)
    }
}

impl fmt::Debug for DirSet {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_set().entries(self.iter()).finish()
    }
}

/// A side of a rectangle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Side {
    /// The side at `max.x`.
    Right,
    /// The side at `max.y`.
    Top,
    /// The side at `min.x`.
    Left,
    /// The side at `min.y`.
    Bottom,
}

impl Side {
    /// All four sides, in the order of the [`Direction`]s pointing out of them.
    pub const ALL: [Self; 4] = [Self::Right, Self::Top, Self::Left, Self::Bottom];

    /// The direction pointing out of the rectangle through this side.
    #[must_use]
    pub const fn outward(self) -> Direction {
        match self {
            Self::Right => Direction::Right,
            Self::Top => Direction::Up,
            Self::Left => Direction::Left,
            Self::Bottom => Direction::Down,
        }
    }
}

/// A closed axis-aligned rectangle with positive width and height.
///
/// Routes may touch the boundary but never enter the open interior.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Rect {
    /// The corner with the smallest coordinates.
    pub min: Point,
    /// The corner with the largest coordinates; strictly greater than `min` on both axes.
    pub max: Point,
}

impl Rect {
    /// Creates the rectangle `[x0, x1] x [y0, y1]`, ordering the bounds.
    ///
    /// # Panics
    ///
    /// Panics if the rectangle has zero width or height. Use [`Rect::try_new`] to get
    /// `None` instead.
    ///
    /// # Examples
    ///
    /// ```
    /// use connector_routing::{Point, Rect};
    ///
    /// let r = Rect::new(10, 0, 0, 5);
    /// assert_eq!(r.min, Point::new(0, 0));
    /// assert_eq!(r.width(), 10);
    /// ```
    #[must_use]
    pub fn new(x0: i32, y0: i32, x1: i32, y1: i32) -> Self {
        Self::try_new(x0, y0, x1, y1).expect("a rectangle needs positive width and height")
    }

    /// Creates the rectangle `[x0, x1] x [y0, y1]`, or `None` if it is degenerate.
    #[must_use]
    pub fn try_new(x0: i32, y0: i32, x1: i32, y1: i32) -> Option<Self> {
        let min = Point::new(x0.min(x1), y0.min(y1));
        let max = Point::new(x0.max(x1), y0.max(y1));

        (min.x < max.x && min.y < max.y).then_some(Self { min, max })
    }

    /// The width, `max.x - min.x`.
    #[must_use]
    pub fn width(&self) -> u64 {
        abs_diff(self.max.x, self.min.x)
    }

    /// The height, `max.y - min.y`.
    #[must_use]
    pub fn height(&self) -> u64 {
        abs_diff(self.max.y, self.min.y)
    }

    /// The low and high bound along `axis`.
    #[must_use]
    pub(crate) fn range(&self, axis: Axis) -> (i32, i32) {
        (self.min.coordinate(axis), self.max.coordinate(axis))
    }

    /// The four corners, counter-clockwise from `min`.
    #[must_use]
    pub fn corners(&self) -> [Point; 4] {
        [
            self.min,
            Point::new(self.max.x, self.min.y),
            self.max,
            Point::new(self.min.x, self.max.y),
        ]
    }

    /// The centre, rounded towards negative infinity.
    #[must_use]
    pub fn center(&self) -> Point {
        Point::new(
            midpoint(self.min.x, self.max.x),
            midpoint(self.min.y, self.max.y),
        )
    }

    /// Whether `p` lies in the open interior.
    #[must_use]
    pub fn interior_contains(&self, p: Point) -> bool {
        self.min.x < p.x && p.x < self.max.x && self.min.y < p.y && p.y < self.max.y
    }

    /// Whether `p` lies in the closed rectangle.
    #[must_use]
    pub fn contains(&self, p: Point) -> bool {
        self.min.x <= p.x && p.x <= self.max.x && self.min.y <= p.y && p.y <= self.max.y
    }

    /// Whether `p` lies on the boundary.
    #[must_use]
    pub fn boundary_contains(&self, p: Point) -> bool {
        self.contains(p) && !self.interior_contains(p)
    }

    /// The rectangle grown by `amount` on every side.
    ///
    /// # Panics
    ///
    /// Panics if a negative `amount` makes the rectangle degenerate.
    #[must_use]
    pub fn buffer(&self, amount: i32) -> Self {
        Self::new(
            self.min.x - amount,
            self.min.y - amount,
            self.max.x + amount,
            self.max.y + amount,
        )
    }

    /// The smallest rectangle containing both.
    #[must_use]
    pub fn union(&self, other: &Self) -> Self {
        Self {
            min: Point::new(self.min.x.min(other.min.x), self.min.y.min(other.min.y)),
            max: Point::new(self.max.x.max(other.max.x), self.max.y.max(other.max.y)),
        }
    }

    /// Whether the open interiors of the two rectangles intersect.
    #[must_use]
    pub fn interiors_overlap(&self, other: &Self) -> bool {
        self.min.x < other.max.x
            && other.min.x < self.max.x
            && self.min.y < other.max.y
            && other.min.y < self.max.y
    }

    /// Whether the closed axis-parallel segment from `a` to `b` meets the open interior.
    ///
    /// `a` and `b` must share one coordinate.
    #[must_use]
    pub fn blocks_segment(&self, a: Point, b: Point) -> bool {
        let (lo, hi) = (
            Point::new(a.x.min(b.x), a.y.min(b.y)),
            Point::new(a.x.max(b.x), a.y.max(b.y)),
        );
        if lo.y == hi.y {
            return self.min.y < lo.y
                && lo.y < self.max.y
                && lo.x < self.max.x
                && self.min.x < hi.x;
        }

        self.min.x < lo.x && lo.x < self.max.x && lo.y < self.max.y && self.min.y < hi.y
    }

    /// The side that `p` lies on, if it lies on exactly one side (not a corner).
    #[must_use]
    pub fn side_of(&self, p: Point) -> Option<Side> {
        let on_x = p.x == self.min.x || p.x == self.max.x;
        let on_y = p.y == self.min.y || p.y == self.max.y;
        if !self.contains(p) || on_x == on_y {
            return None;
        }

        Some(
            match (p.x == self.max.x, p.x == self.min.x, p.y == self.max.y) {
                (true, _, _) => Side::Right,
                (_, true, _) => Side::Left,
                (_, _, true) => Side::Top,
                _ => Side::Bottom,
            },
        )
    }
}

/// `floor((a + b) / 2)` without overflow.
#[must_use]
pub(crate) fn midpoint(a: i32, b: i32) -> i32 {
    i32::try_from((i64::from(a) + i64::from(b)).div_euclid(2)).expect("between two i32")
}

#[cfg(test)]
mod tests {
    //! Unit tests for directions, sets and rectangle predicates.

    use super::*;

    /// Turning left and right are inverse, and reversal is two turns.
    #[test]
    fn turns_compose() {
        for d in Direction::ALL {
            assert_eq!(d.left().right(), d);
            assert_eq!(d.left().left(), d.reverse());
            assert_eq!(d.turns_to(d), 0);
            assert_eq!(d.turns_to(d.left()), 1);
            assert_eq!(d.turns_to(d.right()), 1);
            assert_eq!(d.turns_to(d.reverse()), 2);
        }

        assert_eq!(Direction::Right.left(), Direction::Up);
    }

    /// A segment along a boundary does not enter the interior; one through it does.
    #[test]
    fn segment_blocking_uses_open_interior() {
        let r = Rect::new(0, 0, 10, 10);
        assert!(!r.blocks_segment(Point::new(-5, 0), Point::new(15, 0)));
        assert!(!r.blocks_segment(Point::new(10, -5), Point::new(10, 15)));
        assert!(r.blocks_segment(Point::new(-5, 5), Point::new(1, 5)));
        assert!(!r.blocks_segment(Point::new(-5, 5), Point::new(0, 5)));
        assert!(r.blocks_segment(Point::new(5, 20), Point::new(5, 9)));
    }

    /// Reversing a set reverses every member.
    #[test]
    fn dirset_reverse() {
        let set = DirSet::UP | Direction::Left;
        assert_eq!(set.reversed(), DirSet::DOWN | Direction::Right);
        assert_eq!(DirSet::ALL.iter().count(), 4);
    }

    /// Sides are found for side points and not for corners.
    #[test]
    fn side_of_points() {
        let r = Rect::new(0, 0, 10, 4);
        assert_eq!(r.side_of(Point::new(10, 2)), Some(Side::Right));
        assert_eq!(r.side_of(Point::new(3, 4)), Some(Side::Top));
        assert_eq!(r.side_of(Point::new(0, 0)), None);
        assert_eq!(r.side_of(Point::new(5, 2)), None);
    }
}
