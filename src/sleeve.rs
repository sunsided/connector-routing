//! Sleeve routing (Nachmanson and Chen, arXiv 2605.17498, section 4): smooth any-angle
//! routes through a constrained Delaunay triangulation of the free space.
//!
//! Routing a connector has a combinatorial step and a geometric one. Dijkstra over the
//! dual graph of the triangulation (one node per triangle outside the obstacles,
//! weighted by centroid distances) picks a *sleeve*: a corridor of triangles from the
//! source to the target. The funnel algorithm then pulls the route taut inside the
//! sleeve, so it bends only at obstacle corners. Before searching, the straight segment
//! is tried. Many connectors are batched: a greedy vertex cover of the demand graph picks
//! roots, and one Dijkstra tree per root serves every connector at that root.
//!
//! The geometry is exact: route vertices are input points, and all predicates are exact
//! integer tests. Path costs use integer square roots, so the sleeve choice among nearly
//! equal corridors may differ from the Euclidean optimum by rounding. Not checked against
//! an original funnel description (Lee and Preparata; Chazelle).

pub(crate) mod cdt;
mod funnel;

use alloc::collections::{BTreeMap, BinaryHeap};
use alloc::vec::Vec;
use core::cmp::Reverse;

use crate::error::Error;
use crate::geom::{Point, Rect};

use cdt::{Cdt, NONE};

/// A router over the constrained Delaunay triangulation of a scene.
///
/// # Examples
///
/// ```
/// use connector_routing::{Point, Rect, sleeve::SleeveRouter, check};
///
/// let obstacles = [Rect::new(10, -10, 20, 10)];
/// let (a, b) = (Point::new(0, 0), Point::new(30, 0));
/// let router = SleeveRouter::new(&obstacles, &[a, b], 5)?;
/// let route = router.route(a, b).expect("a route around the box");
/// // Taut over two corners of the box.
/// assert_eq!(route.len(), 4);
/// assert!(check::polyline(&route, &obstacles).is_ok());
/// # Ok::<(), connector_routing::Error>(())
/// ```
#[derive(Debug, Clone)]
pub struct SleeveRouter {
    /// The triangulation of the frame.
    cdt: Cdt,
    /// The obstacles.
    obstacles: Vec<Rect>,
    /// Whether each triangle lies inside an obstacle.
    blocked: Vec<bool>,
}

impl SleeveRouter {
    /// Triangulates the free space around `obstacles` with `endpoints` as extra
    /// vertices, in a frame grown by `margin` around everything.
    ///
    /// # Errors
    ///
    /// [`Error::CoordinateOutOfRange`] beyond [`MAX_COORD`](crate::MAX_COORD),
    /// [`Error::OverlappingObstacles`] if two obstacle interiors overlap (touching is
    /// fine), [`Error::PortBlocked`] for an endpoint inside an obstacle,
    /// [`Error::TriangulationFailed`] if constraint recovery fails (a bug guard).
    ///
    /// # Panics
    ///
    /// Panics if the triangulation would exceed `2^32` triangles.
    pub fn new(obstacles: &[Rect], endpoints: &[Point], margin: u32) -> Result<Self, Error> {
        validate(obstacles, endpoints)?;
        let margin = i32::try_from(margin.max(1)).map_err(|_| Error::CostOverflow)?;
        let frame = frame(obstacles, endpoints, margin)?;
        let mut points: Vec<Point> = obstacles.iter().flat_map(Rect::corners).collect();
        points.extend_from_slice(endpoints);
        let constraints: Vec<(Point, Point)> = obstacles
            .iter()
            .flat_map(|r| {
                let c = r.corners();
                [(c[0], c[1]), (c[1], c[2]), (c[2], c[3]), (c[3], c[0])]
            })
            .collect();
        let cdt = Cdt::new(frame, &points, &constraints)?;
        let blocked = (0..cdt.triangles.len())
            .map(|t| inside_obstacle(&cdt, u32::try_from(t).expect("few triangles"), obstacles))
            .collect();

        Ok(Self {
            cdt,
            obstacles: obstacles.to_vec(),
            blocked,
        })
    }

    /// The number of triangles.
    #[must_use]
    pub fn triangle_count(&self) -> usize {
        self.cdt.triangles.len()
    }

    /// The taut route from `from` to `to` (both endpoints given to [`SleeveRouter::new`]),
    /// or `None` if they are not connected or not vertices.
    #[must_use]
    pub fn route(&self, from: Point, to: Point) -> Option<Vec<Point>> {
        if from == to {
            return Some(alloc::vec![from]);
        }

        if self.straight_is_free(from, to) {
            return Some(alloc::vec![from, to]);
        }

        let tree = self.dijkstra(&self.exits(self.cdt.vertex(from)?));
        self.extract(&tree, to)
    }

    /// Routes every pair, batching the searches with a greedy vertex cover of the demand
    /// graph (one Dijkstra tree per cover vertex). Results are in input order.
    #[must_use]
    pub fn route_all(&self, pairs: &[(Point, Point)]) -> Vec<Option<Vec<Point>>> {
        let roots = vertex_cover(pairs);
        let mut out = alloc::vec![None; pairs.len()];
        let mut by_root: BTreeMap<Point, Vec<usize>> = BTreeMap::new();
        for (i, &(a, b)) in pairs.iter().enumerate() {
            if a == b || self.straight_is_free(a, b) {
                out[i] = Some(if a == b {
                    alloc::vec![a]
                } else {
                    alloc::vec![a, b]
                });
                continue;
            }

            let root = if roots.contains(&a) { a } else { b };
            by_root.entry(root).or_default().push(i);
        }

        for (root, list) in by_root {
            let Some(v) = self.cdt.vertex(root) else {
                continue;
            };
            let tree = self.dijkstra(&self.exits(v));
            for i in list {
                let (a, b) = pairs[i];
                out[i] = if root == a {
                    self.extract(&tree, b)
                } else {
                    self.extract(&tree, a).map(|mut r| {
                        r.reverse();
                        r
                    })
                };
            }
        }

        out
    }

    /// Whether the straight segment stays out of every obstacle interior, by separating
    /// axes (the router's own test, independent of [`crate::check`]).
    fn straight_is_free(&self, a: Point, b: Point) -> bool {
        self.obstacles.iter().all(|r| separated(a, b, r))
    }

    /// The ways out of vertex `v` into free space: `v` itself if a free triangle touches
    /// it, else the nearest vertices with one along *walls*, obstacle sides shared by two
    /// touching obstacles (a slit without area, which no triangle covers). Each exit
    /// comes with the path to it from `v` and its scaled length.
    fn exits(&self, v: u32) -> Vec<Exit> {
        let free = |u: u32| {
            self.cdt
                .around(u)
                .iter()
                .any(|&t| !self.blocked[t as usize])
        };
        let point = |u: u32| self.cdt.points[u as usize];
        if free(v) {
            return alloc::vec![Exit {
                vertex: v,
                path: alloc::vec![point(v)],
                cost: 0
            }];
        }

        // Dijkstra along walls.
        let mut best: BTreeMap<u32, (u64, u32)> = BTreeMap::new();
        let mut heap = BinaryHeap::from([Reverse((0_u64, v, v))]);
        let mut exits = Vec::new();
        while let Some(Reverse((cost, u, parent))) = heap.pop() {
            if best.contains_key(&u) {
                continue;
            }

            best.insert(u, (cost, parent));
            if free(u) {
                let mut path = alloc::vec![point(u)];
                let mut at = u;
                while at != v {
                    at = best[&at].1;
                    path.push(point(at));
                }

                path.reverse();
                exits.push(Exit {
                    vertex: u,
                    path,
                    cost,
                });
                continue;
            }

            for w in self.wall_neighbours(u) {
                if !best.contains_key(&w) {
                    let step = distance(scaled(point(u)), scaled(point(w)));
                    heap.push(Reverse((cost.saturating_add(step), w, u)));
                }
            }
        }

        exits
    }

    /// The vertices joined to `u` by a wall: a constrained edge between two triangles
    /// inside obstacles.
    fn wall_neighbours(&self, u: u32) -> Vec<u32> {
        let mut out = Vec::new();
        for t in self.cdt.around(u) {
            let tri = self.cdt.triangles[t as usize];
            for k in 0..3 {
                let (a, b) = (tri.vertices[(k + 1) % 3], tri.vertices[(k + 2) % 3]);
                let n = tri.neighbors[k];
                let wall = tri.constrained[k]
                    && self.blocked[t as usize]
                    && n != NONE
                    && self.blocked[n as usize];
                if wall && (a == u || b == u) {
                    out.push(if a == u { b } else { a });
                }
            }
        }

        out.sort_unstable();
        out.dedup();

        out
    }

    /// Dijkstra over free triangles from the triangles around the `exits`. Returns per
    /// triangle the cost, the parent (`NONE` for roots) and the exit it grows from;
    /// `u64::MAX` if unreached.
    fn dijkstra(&self, exits: &[Exit]) -> Tree {
        let n = self.cdt.triangles.len();
        let mut best = alloc::vec![(u64::MAX, NONE, 0_usize); n];
        let mut heap = BinaryHeap::new();
        for (e, exit) in exits.iter().enumerate() {
            let s = self.cdt.points[exit.vertex as usize];
            for t in self.cdt.around(exit.vertex) {
                if !self.blocked[t as usize] {
                    let cost = exit
                        .cost
                        .saturating_add(distance(scaled(s), self.centroid3(t)));
                    heap.push(Reverse((cost, t, NONE, e)));
                }
            }
        }

        while let Some(Reverse((cost, t, parent, root))) = heap.pop() {
            if best[t as usize].0 != u64::MAX {
                continue;
            }

            best[t as usize] = (cost, parent, root);
            let tri = self.cdt.triangles[t as usize];
            for (k, &next) in tri.neighbors.iter().enumerate() {
                if next == NONE
                    || tri.constrained[k]
                    || self.blocked[next as usize]
                    || best[next as usize].0 != u64::MAX
                {
                    continue;
                }

                let step = distance(self.centroid3(t), self.centroid3(next));
                heap.push(Reverse((cost.saturating_add(step), next, t, root)));
            }
        }

        Tree {
            best,
            exits: exits.to_vec(),
        }
    }

    /// The route to `to` through the cheapest sleeve of `tree`, from the tree's exit
    /// the sleeve grows from.
    fn extract(&self, tree: &Tree, to: Point) -> Option<Vec<Point>> {
        let target = self.cdt.vertex(to)?;
        let best = &tree.best;
        let (end, arrival) = self
            .exits(target)
            .into_iter()
            .flat_map(|exit| {
                let p = scaled(self.cdt.points[exit.vertex as usize]);
                self.cdt
                    .around(exit.vertex)
                    .into_iter()
                    .filter(|&t| best[t as usize].0 != u64::MAX)
                    .map(move |t| (t, exit.clone(), p))
                    .collect::<Vec<_>>()
            })
            .min_by_key(|(t, exit, p)| {
                let total = best[*t as usize]
                    .0
                    .saturating_add(distance(self.centroid3(*t), *p))
                    .saturating_add(exit.cost);
                (total, *t)
            })
            .map(|(t, exit, _)| (t, exit))?;

        let mut sleeve = alloc::vec![end];
        while let Some(&last) = sleeve.last() {
            let parent = best[last as usize].1;
            if parent == NONE {
                break;
            }

            sleeve.push(parent);
        }

        sleeve.reverse();
        let portals = self.portals(&sleeve);
        let departure = &tree.exits[best[end as usize].2];
        let start = *departure
            .path
            .last()
            .expect("an exit path ends at the exit");
        let finish = *arrival.path.last().expect("an exit path ends at the exit");
        let mut route = departure.path.clone();
        route.extend(funnel::funnel(start, finish, &portals).into_iter().skip(1));
        route.extend(arrival.path.iter().rev().skip(1));

        Some(route)
    }

    /// The `(left, right)` portals between consecutive triangles of a sleeve.
    fn portals(&self, sleeve: &[u32]) -> Vec<(Point, Point)> {
        sleeve
            .windows(2)
            .map(|pair| {
                let tri = self.cdt.triangles[pair[0] as usize];
                let k = tri
                    .neighbors
                    .iter()
                    .position(|&n| n == pair[1])
                    .expect("adjacent");
                // Leaving a counter-clockwise triangle through the edge opposite vertex
                // k, vertex k + 2 is on the left and vertex k + 1 on the right.
                let left = self.cdt.points[tri.vertices[(k + 2) % 3] as usize];
                let right = self.cdt.points[tri.vertices[(k + 1) % 3] as usize];
                (left, right)
            })
            .collect()
    }

    /// Three times the centroid of triangle `t`, exact.
    fn centroid3(&self, t: u32) -> (i64, i64) {
        let [a, b, c] = self.cdt.corners(t);

        (
            i64::from(a.x) + i64::from(b.x) + i64::from(c.x),
            i64::from(a.y) + i64::from(b.y) + i64::from(c.y),
        )
    }
}

/// A way from an endpoint into free space.
#[derive(Debug, Clone)]
struct Exit {
    /// The first vertex with a free triangle around it.
    vertex: u32,
    /// The path from the endpoint to that vertex along walls (just the endpoint if it
    /// touches free space itself).
    path: Vec<Point>,
    /// The scaled length of the path.
    cost: u64,
}

/// A Dijkstra tree over triangles.
struct Tree {
    /// Per triangle: cost, parent (`NONE` for roots), index of the exit it grows from.
    best: Vec<(u64, u32, usize)>,
    /// The exits the tree grows from.
    exits: Vec<Exit>,
}

/// Three times `p`, in the units of [`SleeveRouter::centroid3`].
fn scaled(p: Point) -> (i64, i64) {
    (3 * i64::from(p.x), 3 * i64::from(p.y))
}

/// The Euclidean distance between two scaled points, rounded down by `isqrt`.
fn distance(a: (i64, i64), b: (i64, i64)) -> u64 {
    let dx = u128::from(a.0.abs_diff(b.0));
    let dy = u128::from(a.1.abs_diff(b.1));

    u64::try_from((dx * dx + dy * dy).isqrt()).unwrap_or(u64::MAX)
}

/// Whether triangle `t` lies inside one of `obstacles` (its centroid does; triangles
/// never straddle an obstacle side, which is a constraint).
fn inside_obstacle(cdt: &Cdt, t: u32, obstacles: &[Rect]) -> bool {
    let [a, b, c] = cdt.corners(t);
    let sx = i64::from(a.x) + i64::from(b.x) + i64::from(c.x);
    let sy = i64::from(a.y) + i64::from(b.y) + i64::from(c.y);
    obstacles.iter().any(|r| {
        3 * i64::from(r.min.x) < sx
            && sx < 3 * i64::from(r.max.x)
            && 3 * i64::from(r.min.y) < sy
            && sy < 3 * i64::from(r.max.y)
    })
}

/// Whether the closed segment `a b` and the open interior of `r` are disjoint: some
/// axis of the rectangle or the segment's own line separates them.
pub(crate) fn separated(a: Point, b: Point, r: &Rect) -> bool {
    let (x0, x1) = (a.x.min(b.x), a.x.max(b.x));
    let (y0, y1) = (a.y.min(b.y), a.y.max(b.y));
    if x1 <= r.min.x || x0 >= r.max.x || y1 <= r.min.y || y0 >= r.max.y {
        return true;
    }

    let sides = r.corners().map(|c| cdt::orient(a, b, c).signum());

    sides.iter().all(|&s| s >= 0) || sides.iter().all(|&s| s <= 0)
}

/// Checks coordinates, overlaps and endpoints.
fn validate(obstacles: &[Rect], endpoints: &[Point]) -> Result<(), Error> {
    for p in obstacles
        .iter()
        .flat_map(|r| [r.min, r.max])
        .chain(endpoints.iter().copied())
    {
        if !p.is_in_range() {
            return Err(Error::CoordinateOutOfRange(p));
        }
    }

    for (i, a) in obstacles.iter().enumerate() {
        if obstacles[i + 1..].iter().any(|b| a.interiors_overlap(b)) {
            return Err(Error::OverlappingObstacles);
        }
    }

    match endpoints
        .iter()
        .find(|p| obstacles.iter().any(|r| r.interior_contains(**p)))
    {
        Some(&p) => Err(Error::PortBlocked(p)),
        None => Ok(()),
    }
}

/// The bounding box of everything grown by `margin`; its corners must stay within
/// twice [`MAX_COORD`](crate::MAX_COORD), where the in-circle test is still exact.
fn frame(obstacles: &[Rect], endpoints: &[Point], margin: i32) -> Result<Rect, Error> {
    let mut points = obstacles
        .iter()
        .flat_map(|r| [r.min, r.max])
        .chain(endpoints.iter().copied());
    let first = points.next().unwrap_or_default();
    let (lo, hi) = points.fold((first, first), |(lo, hi), p| {
        (
            Point::new(lo.x.min(p.x), lo.y.min(p.y)),
            Point::new(hi.x.max(p.x), hi.y.max(p.y)),
        )
    });
    let limit = 2 * i64::from(crate::MAX_COORD);
    let grow = |c: i32, sign: i64| {
        let value = i64::from(c) + sign * i64::from(margin);
        if value.abs() > limit {
            Err(Error::CoordinateOutOfRange(Point::new(c, c)))
        } else {
            Ok(i32::try_from(value).expect("within twice MAX_COORD"))
        }
    };

    Ok(Rect::new(
        grow(lo.x, -1)?,
        grow(lo.y, -1)?,
        grow(hi.x, 1)?,
        grow(hi.y, 1)?,
    ))
}

/// A greedy vertex cover of the demand graph (paper, section 4.1): repeatedly take an
/// endpoint of maximum remaining degree (ties by point order).
fn vertex_cover(pairs: &[(Point, Point)]) -> Vec<Point> {
    let mut remaining: Vec<(Point, Point)> =
        pairs.iter().copied().filter(|(a, b)| a != b).collect();
    let mut cover = Vec::new();
    while !remaining.is_empty() {
        let mut degree: BTreeMap<Point, usize> = BTreeMap::new();
        for &(a, b) in &remaining {
            *degree.entry(a).or_default() += 1;
            *degree.entry(b).or_default() += 1;
        }

        let (&best, _) = degree
            .iter()
            .max_by(|x, y| x.1.cmp(y.1).then(y.0.cmp(x.0)))
            .expect("non-empty");
        cover.push(best);
        remaining.retain(|&(a, b)| a != best && b != best);
    }

    cover
}
