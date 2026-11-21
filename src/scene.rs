//! The [`Scene`]: obstacles, ports and their incrementally maintained orthogonal
//! visibility graph (Wybrow, Marriott and Stuckey 2009).
//!
//! The interesting points are the obstacle corners and the ports. Through every
//! interesting point outside all obstacle interiors runs a maximal obstacle-free
//! horizontal and vertical segment; the graph's vertices are the intersections of
//! these segments and its edges join consecutive intersections along a segment.
//!
//! ```mermaid
//! flowchart LR
//!     M[obstacle or port change] --> A[affected segments]
//!     A --> R[remove segments and their vertices]
//!     R --> U[apply the change]
//!     U --> G[recompute segments of affected generators]
//!     G --> I[insert segments, link intersections]
//! ```

mod segments;

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::error::Error;
use crate::geom::{DirSet, Point, Rect};
use crate::graph::{RoutingGraph, VertexId};

use segments::Segments;

/// An obstacle of a [`Scene`]. Stable until the obstacle is removed; ids of removed
/// obstacles are reused.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ObstacleId(pub(crate) u32);

impl ObstacleId {
    /// The slot index.
    #[must_use]
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// A port of a [`Scene`]. Stable until the port is removed; ids of removed ports are
/// reused.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct PortId(pub(crate) u32);

impl PortId {
    /// The slot index.
    #[must_use]
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// A connection point: where connectors start or end, and in which directions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Port {
    /// The position. Must not lie in the open interior of an obstacle.
    pub point: Point,
    /// The directions a connector may leave the port in. A connector ending here
    /// arrives moving opposite to one of them. Never empty.
    pub directions: DirSet,
}

impl Port {
    /// A port at `point` that may be left in any direction in `directions`.
    #[must_use]
    pub fn new(point: Point, directions: impl Into<DirSet>) -> Self {
        Self {
            point,
            directions: directions.into(),
        }
    }

    /// A port at `point` that may be left in any direction.
    #[must_use]
    pub fn free(point: Point) -> Self {
        Self::new(point, DirSet::ALL)
    }
}

/// A slot arena with stable indices and reuse of freed slots.
#[derive(Debug, Clone)]
pub(crate) struct Arena<T> {
    /// The slots; `None` for freed ones.
    slots: Vec<Option<T>>,
    /// Freed slot indices.
    free: Vec<u32>,
}

impl<T> Default for Arena<T> {
    fn default() -> Self {
        Self {
            slots: Vec::new(),
            free: Vec::new(),
        }
    }
}

impl<T> Arena<T> {
    /// Stores `value` and returns its slot.
    fn insert(&mut self, value: T) -> u32 {
        if let Some(index) = self.free.pop() {
            self.slots[index as usize] = Some(value);
            return index;
        }

        self.slots.push(Some(value));

        u32::try_from(self.slots.len() - 1).expect("fewer than 2^32 slots")
    }

    /// Removes and returns the value in slot `index`.
    fn remove(&mut self, index: u32) -> Option<T> {
        let value = self.slots.get_mut(index as usize)?.take()?;
        self.free.push(index);

        Some(value)
    }

    /// The value in slot `index`.
    pub(crate) fn get(&self, index: u32) -> Option<&T> {
        self.slots.get(index as usize)?.as_ref()
    }

    /// The live values with their slots, in slot order.
    pub(crate) fn iter(&self) -> impl Iterator<Item = (u32, &T)> + '_ {
        self.slots.iter().enumerate().filter_map(|(i, slot)| {
            slot.as_ref()
                .map(|value| (u32::try_from(i).expect("fewer than 2^32 slots"), value))
        })
    }

    /// One more than the largest slot index ever used.
    pub(crate) fn capacity(&self) -> usize {
        self.slots.len()
    }
}

/// Obstacles, ports and their orthogonal visibility graph, updated incrementally.
///
/// # Examples
///
/// ```
/// use connector_routing::{DirSet, Point, Port, Rect, Scene};
///
/// let mut scene = Scene::new();
/// let box_a = scene.add_obstacle(Rect::new(0, 0, 10, 10))?;
/// let port = scene.add_port(Port::new(Point::new(10, 5), DirSet::RIGHT))?;
/// assert!(scene.port_vertex(port).is_some());
///
/// // Moving the box updates the graph in place.
/// scene.move_obstacle(box_a, Rect::new(0, 0, 10, 12))?;
/// assert_eq!(scene.obstacle(box_a), Some(Rect::new(0, 0, 10, 12)));
/// # Ok::<(), connector_routing::Error>(())
/// ```
#[derive(Debug, Clone, Default)]
pub struct Scene {
    /// The obstacles.
    obstacles: Arena<Rect>,
    /// The ports.
    ports: Arena<Port>,
    /// How many obstacle corners and ports sit at each interesting point.
    generators: BTreeMap<Point, u32>,
    /// The interesting segments and the visibility graph.
    segments: Segments,
}

impl Scene {
    /// Creates an empty scene.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Builds a scene from all obstacles and ports at once, faster than adding them one
    /// by one. Ids are assigned in input order.
    ///
    /// # Errors
    ///
    /// [`Error::CoordinateOutOfRange`] for a coordinate beyond
    /// [`MAX_COORD`](crate::MAX_COORD), [`Error::PortWithoutDirections`] for a port that
    /// allows no direction.
    pub fn build(
        obstacles: impl IntoIterator<Item = Rect>,
        ports: impl IntoIterator<Item = Port>,
    ) -> Result<Self, Error> {
        let mut scene = Self::new();
        for rect in obstacles {
            check_rect(&rect)?;
            scene.obstacles.insert(rect);
            for corner in rect.corners() {
                *scene.generators.entry(corner).or_default() += 1;
            }
        }

        for port in ports {
            check_port(&port)?;
            scene.ports.insert(port);
            *scene.generators.entry(port.point).or_default() += 1;
        }

        let rects = scene.rects();
        let points: Vec<Point> = scene.generators.keys().copied().collect();
        scene.segments.build(&rects, &points);

        Ok(scene)
    }

    /// The routing graph: the orthogonal visibility graph of the scene.
    #[must_use]
    pub fn graph(&self) -> &RoutingGraph {
        self.segments.graph()
    }

    /// The obstacle `id`, if it exists.
    #[must_use]
    pub fn obstacle(&self, id: ObstacleId) -> Option<Rect> {
        self.obstacles.get(id.0).copied()
    }

    /// Every obstacle with its id, in id order.
    pub fn obstacles(&self) -> impl Iterator<Item = (ObstacleId, Rect)> + '_ {
        self.obstacles.iter().map(|(i, r)| (ObstacleId(i), *r))
    }

    /// One more than the largest obstacle index ever used.
    #[must_use]
    pub fn obstacle_capacity(&self) -> usize {
        self.obstacles.capacity()
    }

    /// The rectangles of all obstacles.
    #[must_use]
    pub fn rects(&self) -> Vec<Rect> {
        self.obstacles.iter().map(|(_, r)| *r).collect()
    }

    /// The port `id`, if it exists.
    #[must_use]
    pub fn port(&self, id: PortId) -> Option<Port> {
        self.ports.get(id.0).copied()
    }

    /// Every port with its id, in id order.
    pub fn ports(&self) -> impl Iterator<Item = (PortId, Port)> + '_ {
        self.ports.iter().map(|(i, p)| (PortId(i), *p))
    }

    /// The graph vertex of port `id`, or `None` if the port does not exist or lies
    /// inside an obstacle.
    #[must_use]
    pub fn port_vertex(&self, id: PortId) -> Option<VertexId> {
        self.graph().vertex_at(self.port(id)?.point)
    }

    /// Whether `p` lies in the open interior of some obstacle.
    #[must_use]
    pub fn is_blocked(&self, p: Point) -> bool {
        self.obstacles.iter().any(|(_, r)| r.interior_contains(p))
    }

    /// Adds an obstacle and updates the graph.
    ///
    /// # Errors
    ///
    /// [`Error::CoordinateOutOfRange`] for a coordinate beyond
    /// [`MAX_COORD`](crate::MAX_COORD).
    pub fn add_obstacle(&mut self, rect: Rect) -> Result<ObstacleId, Error> {
        check_rect(&rect)?;
        let mut id = ObstacleId(0);
        self.change(&[rect], |scene| {
            id = ObstacleId(scene.obstacles.insert(rect));
            scene.add_generators(&rect.corners());
        });

        Ok(id)
    }

    /// Removes an obstacle and updates the graph. Returns its rectangle.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownObstacle`] if `id` names no obstacle.
    pub fn remove_obstacle(&mut self, id: ObstacleId) -> Result<Rect, Error> {
        let rect = self.obstacle(id).ok_or(Error::UnknownObstacle)?;
        self.change(&[rect], |scene| {
            scene.obstacles.remove(id.0);
            scene.remove_generators(&rect.corners());
        });

        Ok(rect)
    }

    /// Moves or resizes an obstacle and updates the graph.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownObstacle`] if `id` names no obstacle,
    /// [`Error::CoordinateOutOfRange`] for a coordinate beyond
    /// [`MAX_COORD`](crate::MAX_COORD).
    pub fn move_obstacle(&mut self, id: ObstacleId, rect: Rect) -> Result<(), Error> {
        let old = self.obstacle(id).ok_or(Error::UnknownObstacle)?;
        check_rect(&rect)?;
        self.change(&[old, rect], |scene| {
            scene.obstacles.slots[id.index()] = Some(rect);
            scene.remove_generators(&old.corners());
            scene.add_generators(&rect.corners());
        });

        Ok(())
    }

    /// Adds a port and updates the graph.
    ///
    /// A port inside an obstacle is accepted (the obstacle may move away later) but has
    /// no vertex until then.
    ///
    /// # Errors
    ///
    /// [`Error::CoordinateOutOfRange`] for a coordinate beyond
    /// [`MAX_COORD`](crate::MAX_COORD), [`Error::PortWithoutDirections`] for a port that
    /// allows no direction.
    pub fn add_port(&mut self, port: Port) -> Result<PortId, Error> {
        check_port(&port)?;
        let id = PortId(self.ports.insert(port));
        self.add_point_generator(port.point);

        Ok(id)
    }

    /// Removes a port and updates the graph. Returns it.
    ///
    /// # Errors
    ///
    /// [`Error::UnknownPort`] if `id` names no port.
    pub fn remove_port(&mut self, id: PortId) -> Result<Port, Error> {
        let port = self.ports.remove(id.0).ok_or(Error::UnknownPort)?;
        self.remove_point_generator(port.point);

        Ok(port)
    }

    /// Applies an obstacle change whose old and new rectangles are `regions`.
    ///
    /// Removes the segments the regions affect, runs `apply`, and recomputes the
    /// segments of every generator that lost its segment.
    fn change(&mut self, regions: &[Rect], apply: impl FnOnce(&mut Self)) {
        let mut affected = self.segments.take_affected(regions);
        apply(self);
        affected.extend(
            self.generators
                .keys()
                .copied()
                .filter(|p| regions.iter().any(|r| r.contains(*p))),
        );

        let rects = self.rects();
        affected.retain(|p| self.generators.contains_key(p));
        affected.sort_unstable();
        affected.dedup();
        self.segments.restore(&rects, &affected);
    }

    /// Counts one more use of each point as an interesting point.
    fn add_generators(&mut self, points: &[Point]) {
        for &p in points {
            *self.generators.entry(p).or_default() += 1;
        }
    }

    /// Counts one use less of each point; a point at zero is no longer interesting.
    fn remove_generators(&mut self, points: &[Point]) {
        for p in points {
            let count = self.generators.get_mut(p).expect("a counted generator");
            *count -= 1;
            if *count == 0 {
                self.generators.remove(p);
            }
        }
    }

    /// Adds one use of `p` as an interesting point, creating its segments if new.
    fn add_point_generator(&mut self, p: Point) {
        let count = self.generators.entry(p).or_default();
        *count += 1;
        if *count == 1 {
            let rects = self.rects();
            self.segments.restore(&rects, &[p]);
        }
    }

    /// Removes one use of `p`; drops it from its segments when the last use goes.
    fn remove_point_generator(&mut self, p: Point) {
        self.remove_generators(&[p]);
        if !self.generators.contains_key(&p) {
            self.segments.drop_generator(p);
        }
    }
}

/// Checks that both corners of `rect` are in range.
fn check_rect(rect: &Rect) -> Result<(), Error> {
    for p in [rect.min, rect.max] {
        if !p.is_in_range() {
            return Err(Error::CoordinateOutOfRange(p));
        }
    }

    Ok(())
}

/// Checks that a port is in range and allows a direction.
fn check_port(port: &Port) -> Result<(), Error> {
    if !port.point.is_in_range() {
        return Err(Error::CoordinateOutOfRange(port.point));
    }

    if port.directions.is_empty() {
        return Err(Error::PortWithoutDirections(port.point));
    }

    Ok(())
}
