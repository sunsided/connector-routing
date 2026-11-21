//! An exact constrained Delaunay triangulation of a rectangular frame with points and
//! constraint segments.
//!
//! Points are inserted one by one into a Delaunay triangulation (point location by
//! walking, Lawson flips to restore the empty-circle property). Constraint segments are
//! then recovered by flipping the edges that cross them (Sloan's method) and the
//! remaining unconstrained edges are made Delaunay again. All predicates are exact in
//! `i128`; coordinates must lie within [`MAX_COORD`](crate::MAX_COORD) (plus a small
//! frame margin).
//!
//! Not checked against an original description (Lawson, Sloan); tested for the
//! empty-circle property, constraint presence and area coverage.

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::geom::{Point, Rect};

/// "No triangle" in [`Triangle::neighbors`].
pub(crate) const NONE: u32 = u32::MAX;

/// One triangle: counter-clockwise vertices, and per vertex the neighbour across the
/// opposite edge and whether that edge is constrained.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Triangle {
    /// Vertex indices, counter-clockwise.
    pub(crate) vertices: [u32; 3],
    /// The neighbour across the edge opposite each vertex, or [`NONE`] on the frame.
    pub(crate) neighbors: [u32; 3],
    /// Whether the edge opposite each vertex is a constraint.
    pub(crate) constrained: [bool; 3],
}

/// A constrained Delaunay triangulation.
#[derive(Debug, Clone)]
pub(crate) struct Cdt {
    /// The vertices.
    pub(crate) points: Vec<Point>,
    /// The triangles.
    pub(crate) triangles: Vec<Triangle>,
    /// The vertex index of each point.
    index: BTreeMap<Point, u32>,
    /// The triangle the last point location ended in, to start the next walk.
    last: u32,
}

/// Twice the signed area of `a b c`: positive if counter-clockwise.
pub(crate) fn orient(a: Point, b: Point, c: Point) -> i128 {
    let (ax, ay) = (i128::from(a.x), i128::from(a.y));
    (i128::from(b.x) - ax) * (i128::from(c.y) - ay)
        - (i128::from(b.y) - ay) * (i128::from(c.x) - ax)
}

/// Positive if `d` lies strictly inside the circumcircle of the counter-clockwise
/// triangle `a b c`, zero if on it.
fn in_circle(a: Point, b: Point, c: Point, d: Point) -> i128 {
    let rel = |p: Point| {
        (
            i128::from(p.x) - i128::from(d.x),
            i128::from(p.y) - i128::from(d.y),
        )
    };
    let (ax, ay) = rel(a);
    let (bx, by) = rel(b);
    let (cx, cy) = rel(c);
    let (a2, b2, c2) = (ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy);

    ax * (by * c2 - b2 * cy) - ay * (bx * c2 - b2 * cx) + a2 * (bx * cy - by * cx)
}

impl Cdt {
    /// Triangulates `frame` with `points` inside or on it and the `constraints` (pairs
    /// of points), which must not cross each other except at shared points.
    pub(crate) fn new(frame: Rect, points: &[Point], constraints: &[(Point, Point)]) -> Self {
        let corners = frame.corners();
        let mut cdt = Self {
            points: corners.to_vec(),
            triangles: alloc::vec![
                Triangle {
                    vertices: [0, 1, 2],
                    neighbors: [NONE, 1, NONE],
                    constrained: [false; 3]
                },
                Triangle {
                    vertices: [0, 2, 3],
                    neighbors: [NONE, NONE, 0],
                    constrained: [false; 3]
                },
            ],
            index: BTreeMap::new(),
            last: 0,
        };
        for (i, &c) in corners.iter().enumerate() {
            cdt.index.insert(c, u32::try_from(i).expect("four corners"));
        }

        for &p in points {
            cdt.insert(p);
        }

        for &(a, b) in constraints {
            cdt.constrain(a, b);
        }

        cdt
    }

    /// The vertex index of `p`, if it is a vertex.
    pub(crate) fn vertex(&self, p: Point) -> Option<u32> {
        self.index.get(&p).copied()
    }

    /// The points of triangle `t`.
    pub(crate) fn corners(&self, t: u32) -> [Point; 3] {
        self.triangles[t as usize]
            .vertices
            .map(|v| self.points[v as usize])
    }

    /// The slot in `t` of the edge shared with neighbour `n`.
    fn edge_to(&self, t: u32, n: u32) -> usize {
        self.triangles[t as usize]
            .neighbors
            .iter()
            .position(|&m| m == n)
            .expect("adjacent triangles")
    }

    /// Finds a triangle containing `p` (inside or on its boundary) by walking.
    #[expect(
        clippy::many_single_char_names,
        reason = "triangle and vertex names follow the usual geometry notation"
    )]
    fn locate(&self, p: Point) -> u32 {
        let mut t = self
            .last
            .min(u32::try_from(self.triangles.len() - 1).expect("few triangles"));
        // A walk in a Delaunay triangulation never cycles; the bound only guards bugs.
        for _ in 0..=4 * self.triangles.len() {
            let tri = self.triangles[t as usize];
            let corners = self.corners(t);
            let mut moved = false;
            for i in 0..3 {
                let (a, b) = (corners[(i + 1) % 3], corners[(i + 2) % 3]);
                if orient(a, b, p) < 0 && tri.neighbors[i] != NONE {
                    t = tri.neighbors[i];
                    moved = true;
                    break;
                }
            }

            if !moved {
                return t;
            }
        }

        // Fall back to a scan.
        (0..self.triangles.len())
            .map(|i| u32::try_from(i).expect("few triangles"))
            .find(|&t| {
                let [a, b, c] = self.corners(t);
                orient(a, b, p) >= 0 && orient(b, c, p) >= 0 && orient(c, a, p) >= 0
            })
            .expect("the point lies in the frame")
    }

    /// Inserts point `p` (ignored if already a vertex) and restores the Delaunay
    /// property.
    fn insert(&mut self, p: Point) {
        if self.index.contains_key(&p) {
            return;
        }

        let v = u32::try_from(self.points.len()).expect("few points");
        self.points.push(p);
        self.index.insert(p, v);
        let t = self.locate(p);
        let corners = self.corners(t);
        let on_edge = (0..3).find(|&i| orient(corners[(i + 1) % 3], corners[(i + 2) % 3], p) == 0);
        let mut stack = Vec::new();
        match on_edge {
            Some(i) => self.split_edge(t, i, v, &mut stack),
            None => self.split_triangle(t, v, &mut stack),
        }

        self.legalize(&mut stack);
    }

    /// Replaces the neighbour link `old` of triangle `t` by `new`.
    fn relink(&mut self, t: u32, old: u32, new: u32) {
        if t == NONE {
            return;
        }

        let slot = self.edge_to(t, old);
        self.triangles[t as usize].neighbors[slot] = new;
    }

    /// Splits triangle `t` at interior vertex `v` into three.
    #[expect(
        clippy::many_single_char_names,
        reason = "triangle and vertex names follow the usual geometry notation"
    )]
    fn split_triangle(&mut self, t: u32, v: u32, stack: &mut Vec<(u32, usize)>) {
        let old = self.triangles[t as usize];
        let [a, b, c] = old.vertices;
        let t1 = u32::try_from(self.triangles.len()).expect("few triangles");
        let t2 = t1 + 1;
        // t: (v, b, c), t1: (a, v, c), t2: (a, b, v); the edge opposite v keeps its
        // neighbour and constraint.
        self.triangles[t as usize] = Triangle {
            vertices: [v, b, c],
            neighbors: [old.neighbors[0], t1, t2],
            constrained: [old.constrained[0], false, false],
        };
        self.triangles.push(Triangle {
            vertices: [a, v, c],
            neighbors: [t, old.neighbors[1], t2],
            constrained: [false, old.constrained[1], false],
        });
        self.triangles.push(Triangle {
            vertices: [a, b, v],
            neighbors: [t, t1, old.neighbors[2]],
            constrained: [false, false, old.constrained[2]],
        });
        self.relink(old.neighbors[1], t, t1);
        self.relink(old.neighbors[2], t, t2);
        self.last = t;
        stack.extend([(t, 0), (t1, 1), (t2, 2)]);
    }

    /// Splits the edge opposite slot `i` of triangle `t` at vertex `v` on it.
    fn split_edge(&mut self, t: u32, i: usize, v: u32, stack: &mut Vec<(u32, usize)>) {
        let other = self.triangles[t as usize].neighbors[i];
        let constrained = self.triangles[t as usize].constrained[i];
        let first = self.split_half(t, i, v, stack);
        if other == NONE {
            return;
        }

        let j = self.edge_to(other, t);
        let second = self.split_half(other, j, v, stack);
        // Link the halves across the split edge: each half of one side meets the half
        // of the other side that shares its original edge end.
        self.link_halves(first, second, constrained);
    }

    /// Splits triangle `t` by the segment from vertex `v` (on the edge opposite slot
    /// `i`) to the vertex in slot `i`. Returns the two halves with the vertex each
    /// keeps of the split edge: `[(half, end), (half, end)]`.
    #[expect(
        clippy::many_single_char_names,
        reason = "triangle and vertex names follow the usual geometry notation"
    )]
    fn split_half(
        &mut self,
        t: u32,
        i: usize,
        v: u32,
        stack: &mut Vec<(u32, usize)>,
    ) -> [(u32, u32); 2] {
        let old = self.triangles[t as usize];
        let apex = old.vertices[i];
        let p = old.vertices[(i + 1) % 3];
        let q = old.vertices[(i + 2) % 3];
        let half = u32::try_from(self.triangles.len()).expect("few triangles");
        // t: (apex, p, v), half: (apex, v, q).
        self.triangles[t as usize] = Triangle {
            vertices: [apex, p, v],
            neighbors: [NONE, half, old.neighbors[(i + 2) % 3]],
            constrained: [false, false, old.constrained[(i + 2) % 3]],
        };
        self.triangles.push(Triangle {
            vertices: [apex, v, q],
            neighbors: [NONE, old.neighbors[(i + 1) % 3], t],
            constrained: [false, old.constrained[(i + 1) % 3], false],
        });
        self.relink(old.neighbors[(i + 1) % 3], t, half);
        self.last = t;
        stack.extend([(t, 2), (half, 1)]);

        [(t, p), (half, q)]
    }

    /// Links the halves of two triangles split along a shared edge.
    fn link_halves(&mut self, first: [(u32, u32); 2], second: [(u32, u32); 2], constrained: bool) {
        for &(a, end) in &first {
            let &(b, _) = second
                .iter()
                .find(|&&(_, e)| e == end)
                .expect("matching halves");
            // In each half, the split edge is opposite the apex, slot 0.
            self.triangles[a as usize].neighbors[0] = b;
            self.triangles[b as usize].neighbors[0] = a;
            self.triangles[a as usize].constrained[0] = constrained;
            self.triangles[b as usize].constrained[0] = constrained;
        }
    }

    /// Flips edges from `stack` until every unconstrained edge is locally Delaunay.
    #[expect(
        clippy::many_single_char_names,
        reason = "triangle and vertex names follow the usual geometry notation"
    )]
    fn legalize(&mut self, stack: &mut Vec<(u32, usize)>) {
        while let Some((t, i)) = stack.pop() {
            let tri = self.triangles[t as usize];
            let n = tri.neighbors[i];
            if n == NONE || tri.constrained[i] {
                continue;
            }

            let j = self.edge_to(n, t);
            let far = self.points[self.triangles[n as usize].vertices[j] as usize];
            let [a, b, c] = self.corners(t);
            if in_circle(a, b, c, far) > 0 {
                let (x, y) = self.flip(t, i);
                stack.extend([(x, 0), (x, 1), (y, 0), (y, 1)]);
            }
        }
    }

    /// Flips the edge opposite slot `i` of `t` to the other diagonal of the two
    /// triangles' quadrilateral. Returns both new triangles; in each, the new diagonal
    /// is opposite slot 2 and the outer edges are opposite slots 0 and 1.
    #[expect(
        clippy::many_single_char_names,
        reason = "triangle and vertex names follow the usual geometry notation"
    )]
    fn flip(&mut self, t: u32, i: usize) -> (u32, u32) {
        let tri = self.triangles[t as usize];
        let n = tri.neighbors[i];
        let j = self.edge_to(n, t);
        let other = self.triangles[n as usize];
        let p = tri.vertices[i];
        let a = tri.vertices[(i + 1) % 3];
        let b = tri.vertices[(i + 2) % 3];
        let q = other.vertices[j];
        // Old: t = (p, a, b) and n = (q, b, a) share edge a b. New: t = (q, p, a) and
        // n = (p, q, b) share edge p q.
        let t_pa = tri.neighbors[(i + 2) % 3]; // across edge p a (opposite b)
        let t_bp = tri.neighbors[(i + 1) % 3]; // across edge b p (opposite a)
        let n_qb = other.neighbors[(j + 2) % 3]; // across edge q b (opposite a)
        let n_aq = other.neighbors[(j + 1) % 3]; // across edge a q (opposite b)
        let c_pa = tri.constrained[(i + 2) % 3];
        let c_bp = tri.constrained[(i + 1) % 3];
        let c_qb = other.constrained[(j + 2) % 3];
        let c_aq = other.constrained[(j + 1) % 3];

        // t = (q, p, a): opposite q is edge p a, opposite p is edge a q, opposite a is p q.
        self.triangles[t as usize] = Triangle {
            vertices: [q, p, a],
            neighbors: [t_pa, n_aq, n],
            constrained: [c_pa, c_aq, false],
        };
        // n = (p, q, b): opposite p is edge q b, opposite q is edge b p, opposite b is p q.
        self.triangles[n as usize] = Triangle {
            vertices: [p, q, b],
            neighbors: [n_qb, t_bp, t],
            constrained: [c_qb, c_bp, false],
        };
        self.relink(n_aq, n, t);
        self.relink(t_bp, t, n);

        (t, n)
    }

    /// Makes the segment `a b` an edge and marks it constrained, splitting it at the
    /// vertices that lie on it.
    fn constrain(&mut self, a: Point, b: Point) {
        let (Some(va), Some(vb)) = (self.vertex(a), self.vertex(b)) else {
            return;
        };
        // Vertices strictly inside the segment split it.
        let mut on: Vec<Point> = self
            .points
            .iter()
            .copied()
            .filter(|&p| p != a && p != b && orient(a, b, p) == 0 && between(a, b, p))
            .collect();
        on.sort_by_key(|p| a.distance_squared(*p));
        let mut chain = alloc::vec![va];
        chain.extend(on.iter().map(|p| self.vertex(*p).expect("a vertex")));
        chain.push(vb);
        for pair in chain.windows(2) {
            self.recover(pair[0], pair[1]);
        }
    }

    /// Recovers the edge between vertices `u` and `w` (no vertex lies strictly between
    /// them) by flipping the edges that cross it, then marks it constrained.
    fn recover(&mut self, u: u32, w: u32) {
        let (pu, pw) = (self.points[u as usize], self.points[w as usize]);
        for _ in 0..=16 * self.triangles.len() {
            if self.mark_edge(u, w) {
                self.relegalize();
                return;
            }

            let Some((t, i)) = self.first_crossing(pu, pw) else {
                return;
            };
            if self.is_convex(t, i) {
                self.flip(t, i);
            } else {
                // Flip a crossing edge further along first; Sloan's queue order.
                self.flip_any_convex(pu, pw);
            }
        }
    }

    /// Marks the edge between `u` and `w` constrained in both triangles, if it exists.
    fn mark_edge(&mut self, u: u32, w: u32) -> bool {
        let mut found = false;
        for t in 0..self.triangles.len() {
            let tri = self.triangles[t];
            for i in 0..3 {
                let (x, y) = (tri.vertices[(i + 1) % 3], tri.vertices[(i + 2) % 3]);
                if (x == u && y == w) || (x == w && y == u) {
                    self.triangles[t].constrained[i] = true;
                    found = true;
                }
            }
        }

        found
    }

    /// Some edge that properly crosses the segment `a b`, as `(triangle, slot)`.
    fn crossings(&self, a: Point, b: Point) -> Vec<(u32, usize)> {
        let mut out = Vec::new();
        for (t, tri) in self.triangles.iter().enumerate() {
            for i in 0..3 {
                if tri.neighbors[i] == NONE {
                    continue;
                }

                let p = self.points[tri.vertices[(i + 1) % 3] as usize];
                let q = self.points[tri.vertices[(i + 2) % 3] as usize];
                if properly_cross(a, b, p, q) {
                    out.push((u32::try_from(t).expect("few triangles"), i));
                }
            }
        }

        out
    }

    /// The first edge crossing `a b`, if any.
    fn first_crossing(&self, a: Point, b: Point) -> Option<(u32, usize)> {
        self.crossings(a, b).into_iter().next()
    }

    /// Flips some crossing edge of `a b` whose quadrilateral is convex.
    fn flip_any_convex(&mut self, a: Point, b: Point) {
        if let Some(&(t, i)) = self
            .crossings(a, b)
            .iter()
            .find(|&&(t, i)| self.is_convex(t, i))
        {
            self.flip(t, i);
        }
    }

    /// Whether the quadrilateral of the two triangles at edge slot `i` of `t` is
    /// strictly convex, so the edge can be flipped.
    #[expect(
        clippy::many_single_char_names,
        reason = "triangle and vertex names follow the usual geometry notation"
    )]
    fn is_convex(&self, t: u32, i: usize) -> bool {
        let tri = self.triangles[t as usize];
        let n = tri.neighbors[i];
        if n == NONE || tri.constrained[i] {
            return false;
        }

        let j = self.edge_to(n, t);
        let p = self.points[tri.vertices[i] as usize];
        let a = self.points[tri.vertices[(i + 1) % 3] as usize];
        let b = self.points[tri.vertices[(i + 2) % 3] as usize];
        let q = self.points[self.triangles[n as usize].vertices[j] as usize];

        orient(p, a, q) > 0 && orient(q, b, p) > 0
    }

    /// Restores the Delaunay property of all unconstrained edges.
    fn relegalize(&mut self) {
        let mut stack: Vec<(u32, usize)> = (0..self.triangles.len())
            .flat_map(|t| (0..3).map(move |i| (u32::try_from(t).expect("few triangles"), i)))
            .collect();
        self.legalize(&mut stack);
    }

    /// The triangles around vertex `v`.
    pub(crate) fn around(&self, v: u32) -> Vec<u32> {
        (0..self.triangles.len())
            .filter(|&t| self.triangles[t].vertices.contains(&v))
            .map(|t| u32::try_from(t).expect("few triangles"))
            .collect()
    }
}

/// Whether `p`, collinear with `a b`, lies between them.
fn between(a: Point, b: Point, p: Point) -> bool {
    a.x.min(b.x) <= p.x && p.x <= a.x.max(b.x) && a.y.min(b.y) <= p.y && p.y <= a.y.max(b.y)
}

/// Whether segments `a b` and `p q` cross at a point interior to both.
fn properly_cross(a: Point, b: Point, p: Point, q: Point) -> bool {
    let d1 = orient(a, b, p).signum();
    let d2 = orient(a, b, q).signum();
    let d3 = orient(p, q, a).signum();
    let d4 = orient(p, q, b).signum();

    d1 * d2 < 0 && d3 * d4 < 0
}

#[cfg(test)]
mod tests {
    //! The triangulation against its defining properties.

    extern crate std;

    use super::*;

    /// Checks adjacency symmetry, positive orientation, area, the empty-circle property
    /// of unconstrained edges, and the presence of every constraint.
    fn validate(cdt: &Cdt, frame: Rect, constraints: &[(Point, Point)]) {
        let mut area = 0;
        for (t, tri) in cdt.triangles.iter().enumerate() {
            let [a, b, c] = cdt.corners(u32::try_from(t).unwrap());
            assert!(
                orient(a, b, c) > 0,
                "triangle {t} is counter-clockwise and not degenerate"
            );
            area += orient(a, b, c);
            for i in 0..3 {
                let n = tri.neighbors[i];
                if n == NONE {
                    continue;
                }

                let back = cdt.edge_to(n, u32::try_from(t).unwrap());
                assert_eq!(
                    cdt.triangles[n as usize].constrained[back],
                    tri.constrained[i]
                );
                if !tri.constrained[i] {
                    let far = cdt.points[cdt.triangles[n as usize].vertices[back] as usize];
                    assert!(
                        in_circle(a, b, c, far) <= 0,
                        "edge of triangle {t} is Delaunay"
                    );
                }
            }
        }

        assert_eq!(
            area,
            2 * i128::from(frame.width()) * i128::from(frame.height())
        );
        for &(a, b) in constraints {
            // Every piece of the constraint is an edge.
            let (ua, ub) = (cdt.vertex(a).unwrap(), cdt.vertex(b).unwrap());
            let mut on: std::vec::Vec<Point> = cdt
                .points
                .iter()
                .copied()
                .filter(|&p| orient(a, b, p) == 0 && between(a, b, p))
                .collect();
            on.sort_by_key(|p| a.distance_squared(*p));
            let _ = (ua, ub);
            for pair in on.windows(2) {
                let (u, w) = (cdt.vertex(pair[0]).unwrap(), cdt.vertex(pair[1]).unwrap());
                let present = cdt.triangles.iter().any(|tri| {
                    (0..3).any(|i| {
                        let (x, y) = (tri.vertices[(i + 1) % 3], tri.vertices[(i + 2) % 3]);
                        ((x == u && y == w) || (x == w && y == u)) && tri.constrained[i]
                    })
                });
                assert!(present, "constraint piece {:?} {:?}", pair[0], pair[1]);
            }
        }
    }

    /// Rectangles in a frame: corners, sides as constraints, co-circular corners.
    #[test]
    fn rectangles_in_frame() {
        let frame = Rect::new(-10, -10, 50, 40);
        let rects = [
            Rect::new(0, 0, 10, 10),
            Rect::new(10, 20, 30, 25),
            Rect::new(35, 0, 45, 30),
        ];
        let mut points: std::vec::Vec<Point> = rects.iter().flat_map(Rect::corners).collect();
        points.push(Point::new(10, 5));
        let constraints: std::vec::Vec<(Point, Point)> = rects
            .iter()
            .flat_map(|r| {
                let c = r.corners();
                [(c[0], c[1]), (c[1], c[2]), (c[2], c[3]), (c[3], c[0])]
            })
            .collect();
        let cdt = Cdt::new(frame, &points, &constraints);
        validate(&cdt, frame, &constraints);
    }

    /// A grid of collinear and co-circular points, a worst case for degeneracies.
    #[test]
    fn grid_points() {
        let frame = Rect::new(0, 0, 12, 12);
        let points: std::vec::Vec<Point> = (0..=12)
            .step_by(3)
            .flat_map(|x| (0..=12).step_by(3).map(move |y| Point::new(x, y)))
            .collect();
        let constraints = [
            (Point::new(3, 3), Point::new(9, 3)),
            (Point::new(9, 3), Point::new(9, 9)),
        ];
        let cdt = Cdt::new(frame, &points, &constraints);
        validate(&cdt, frame, &constraints);
    }

    /// Random rectangles (disjoint by construction on a lattice) with constraints.
    #[test]
    fn random_lattices() {
        let mut seed: u64 = 7;
        let mut next = |m: i32| {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            i32::try_from((seed >> 33) % u64::try_from(m).unwrap()).unwrap()
        };
        for _ in 0..40 {
            let mut rects = std::vec::Vec::new();
            for i in 0..4 {
                for j in 0..4 {
                    if next(3) == 0 {
                        continue;
                    }

                    let (x, y) = (i * 20 + next(5), j * 20 + next(5));
                    rects.push(Rect::new(x, y, x + 3 + next(10), y + 3 + next(10)));
                }
            }

            let frame = Rect::new(-5, -5, 90, 90);
            let points: std::vec::Vec<Point> = rects.iter().flat_map(Rect::corners).collect();
            let constraints: std::vec::Vec<(Point, Point)> = rects
                .iter()
                .flat_map(|r| {
                    let c = r.corners();
                    [(c[0], c[1]), (c[1], c[2]), (c[2], c[3]), (c[3], c[0])]
                })
                .collect();
            let cdt = Cdt::new(frame, &points, &constraints);
            validate(&cdt, frame, &constraints);
        }
    }
}
