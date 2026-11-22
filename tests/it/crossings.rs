//! Required crossings, crossing reduction, and the guarantees of Wybrow et al. (2009,
//! Theorems 3 and 4) for the ordering.

use connector_routing::global::{Config, route_all};
use connector_routing::{
    Connector, DirSet, Direction, Objective, Point, Port, Rect, Route, RoutingGraph, Scene,
    VertexId, check, crossing, nudge,
};
use proptest::prelude::*;

use crate::support::rect;

/// The vertex path along the polyline `points` (which must follow graph edges).
fn path(graph: &RoutingGraph, points: &[(i32, i32)]) -> Vec<VertexId> {
    let pts: Vec<Point> = points.iter().map(|&(x, y)| Point::new(x, y)).collect();
    let mut v = graph.vertex_at(pts[0]).unwrap();
    let mut out = vec![v];
    for pair in pts.windows(2) {
        let dir = Direction::between(pair[0], pair[1]).unwrap();
        while graph.point(v) != pair[1] {
            v = graph.neighbor(v, dir).unwrap();
            out.push(v);
        }
    }

    out
}

/// A route crossing another twice is rerouted along the other's section and crosses at
/// most once afterwards.
#[test]
fn reduction_removes_double_crossing() {
    let ports = [(0, 5), (30, 5), (10, 0), (20, 0), (10, 10), (20, 10)]
        .map(|(x, y)| Port::free(Point::new(x, y)));
    let scene = Scene::build([], ports).unwrap();
    let g = scene.graph();
    let a = Route::from_vertices(g, path(g, &[(0, 5), (30, 5)]), 30);
    let b = Route::from_vertices(g, path(g, &[(10, 0), (10, 10), (20, 10), (20, 0)]), 40);
    let mut routes = vec![a, b];
    assert_eq!(crossing::total_crossings(g, &routes), 2);

    let weights = Objective::default().weights(g).unwrap();
    assert!(crossing::reduce_crossings(g, &mut routes, weights, 4) >= 1);
    assert!(crossing::total_crossings(g, &routes) <= 1);
    for r in &routes {
        check::orthogonal(&r.points, &[]).unwrap();
    }
}

/// Theorem 4: when the shared-edge graph is planar (the connectors can be drawn
/// without crossings), ordering and nudging produce no crossing. A bundle of nested
/// U-shaped connectors around a box shares the box's top side.
#[test]
fn planar_bundle_has_no_crossings() {
    let rects = [Rect::new(0, 0, 40, 20)];
    let mut scene = Scene::build(rects, []).unwrap();
    let mut connectors = Vec::new();
    for i in 0..4 {
        let source = scene
            .add_port(Port::new(Point::new(4 + 4 * i, 20), DirSet::UP))
            .unwrap();
        let target = scene
            .add_port(Port::new(Point::new(36 - 4 * i, 20), DirSet::UP))
            .unwrap();
        connectors.push(Connector { source, target });
    }

    let routing = route_all(&scene, &connectors, &Config::default()).unwrap();
    assert_eq!(crossing::total_crossings(scene.graph(), &routing.routes), 0);
    let nudged = nudge::nudge(
        scene.graph(),
        &rects,
        &routing.routes,
        &nudge::Config::default(),
    );
    for (i, a) in nudged.routes.iter().enumerate() {
        check::orthogonal(a, &rects).unwrap();
        for b in &nudged.routes[i + 1..] {
            assert_eq!(check::orthogonal_crossings(a, b), 0);
        }
    }

    assert_eq!(nudged.overlaps, 0);
}

/// Theorem 3: a necessary crossing happens once, at an end of the shared stretch, not
/// inside it. Two connectors swap sides along a shared channel.
#[test]
fn necessary_crossing_happens_once() {
    let rects = [Rect::new(0, 0, 60, 10), Rect::new(0, 30, 60, 40)];
    let mut scene = Scene::build(rects, []).unwrap();
    let mut add = |a: (i32, i32), da: DirSet, b: (i32, i32), db: DirSet| {
        let source = scene.add_port(Port::new(Point::new(a.0, a.1), da)).unwrap();
        let target = scene.add_port(Port::new(Point::new(b.0, b.1), db)).unwrap();
        Connector { source, target }
    };
    // One enters from the lower box and leaves into the upper, the other the reverse.
    let connectors = vec![
        add((10, 10), DirSet::UP, (50, 30), DirSet::DOWN),
        add((14, 30), DirSet::DOWN, (46, 10), DirSet::UP),
    ];
    let routing = route_all(&scene, &connectors, &Config::default()).unwrap();
    let required = crossing::total_crossings(scene.graph(), &routing.routes);
    let nudged = nudge::nudge(
        scene.graph(),
        &rects,
        &routing.routes,
        &nudge::Config::default(),
    );
    let geometric = check::orthogonal_crossings(&nudged.routes[0], &nudged.routes[1]);
    assert_eq!(u32::try_from(geometric).unwrap(), required);
    assert!(required <= 1);
}

/// Regression for the fixed look directions of arXiv 2309.01671: two routes share an
/// L-shaped stretch whose ends disagree. Horizontal edges looking left and vertical edges
/// looking down decide the two legs from different ends, which flips the order at the
/// shared corner: three crossings instead of one. Pseudo directions (Wybrow et al. 2009)
/// make both legs look towards the same end.
#[test]
fn shared_corner_crosses_once() {
    let ports = [(8, 32), (48, 4), (4, 36), (16, 8)].map(|(x, y)| Port::free(Point::new(x, y)));
    let scene = Scene::build([], ports).unwrap();
    let g = scene.graph();
    let a = Route::from_vertices(g, path(g, &[(8, 32), (4, 32), (4, 4), (48, 4)]), 0);
    let b = Route::from_vertices(g, path(g, &[(4, 36), (4, 4), (16, 4), (16, 8)]), 0);
    let routes = [a, b];
    assert_eq!(crossing::total_crossings(g, &routes), 1);
    let config = nudge::Config {
        separation: 1,
        max_separation: 2,
        ..nudge::Config::default()
    };
    let nudged = nudge::nudge(g, &[], &routes, &config);
    assert_eq!(nudged.overlaps, 0);
    assert_eq!(
        check::orthogonal_crossings(&nudged.routes[0], &nudged.routes[1]),
        1
    );
}

/// Routes for random free-port connectors among random boxes, or `None` if a port is
/// walled in (the only expected error).
fn random_routing(
    rects: &[Rect],
    pairs: Vec<((i32, i32), (i32, i32))>,
) -> Option<(Scene, Vec<Connector<connector_routing::PortId>>, Vec<Route>)> {
    let blocked = |p: Point| rects.iter().any(|r| r.interior_contains(p));
    let mut scene = Scene::build(rects.iter().copied(), []).unwrap();
    let mut connectors = Vec::new();
    for ((ax, ay), (bx, by)) in pairs {
        let (a, b) = (Point::new(ax, ay), Point::new(bx, by));
        if blocked(a) || blocked(b) {
            continue;
        }

        let source = scene.add_port(Port::free(a)).unwrap();
        let target = scene.add_port(Port::free(b)).unwrap();
        connectors.push(Connector { source, target });
    }

    match route_all(&scene, &connectors, &Config::default()) {
        Ok(routing) => Some((scene, connectors, routing.routes)),
        Err(connector_routing::Error::NoRoute { .. }) => None,
        Err(e) => panic!("{e}"),
    }
}

proptest! {
    #![proptest_config(crate::support::cases(40))]

    /// Required crossings are symmetric and sum to the total; crossing reduction never
    /// raises the total, and every rerouted connector stays valid between its ports.
    #[test]
    fn reduction_never_adds_crossings(
        rects in proptest::collection::vec(rect(30), 0..8),
        pairs in proptest::collection::vec(((0..=30, 0..=30), (0..=30, 0..=30)), 2..10),
    ) {
        let Some((scene, connectors, mut routes)) = random_routing(&rects, pairs) else {
            return Ok(());
        };
        let g = scene.graph();
        let mut pairwise = 0;
        for (i, a) in routes.iter().enumerate() {
            for b in &routes[i + 1..] {
                let ab = crossing::required_crossings(g, &a.vertices, &b.vertices);
                prop_assert_eq!(ab, crossing::required_crossings(g, &b.vertices, &a.vertices));
                pairwise += ab;
            }
        }

        let before = crossing::total_crossings(g, &routes);
        prop_assert_eq!(before, pairwise);

        let weights = Objective::default().weights(g).unwrap();
        crossing::reduce_crossings(g, &mut routes, weights, 8);
        prop_assert!(crossing::total_crossings(g, &routes) <= before);
        for (c, r) in connectors.iter().zip(&routes) {
            check::orthogonal(&r.points, &rects).unwrap();
            prop_assert_eq!(r.points.first(), Some(&scene.port(c.source).unwrap().point));
            prop_assert_eq!(r.points.last(), Some(&scene.port(c.target).unwrap().point));
        }
    }
}

/// `crossing::iter_points` agrees with `check::orthogonal_crossings` pair by pair, and
/// every point lies inside both of its segments.
#[test]
fn crossing_points_match_counts_and_segments() {
    let p = |pts: &[(i32, i32)]| {
        pts.iter()
            .map(|&(x, y)| Point::new(x, y))
            .collect::<Vec<_>>()
    };
    let routes = [
        p(&[(0, 5), (20, 5), (20, 15), (0, 15)]),
        p(&[(5, 0), (5, 20)]),
        p(&[(10, 0), (10, 5), (30, 5)]),
        p(&[(15, -5), (15, 25)]),
        p(&[(0, 0), (8, 8)]),
    ];
    let points = crossing::points(&routes);
    assert_eq!(points, crossing::iter_points(&routes).collect::<Vec<_>>());

    let expected: usize = (0..routes.len())
        .flat_map(|i| (i + 1..routes.len()).map(move |j| (i, j)))
        .map(|(i, j)| check::orthogonal_crossings(&routes[i], &routes[j]))
        .sum();
    // Route 0 crosses 1 and 3 twice each and route 2 crosses 3 once. Route 2 only
    // touches or overlaps route 0, and the diagonal route 4 is skipped.
    assert_eq!(expected, 5);
    assert_eq!(points.len(), expected);

    for c in &points {
        let (hr, hs) = c.horizontal;
        let (vr, vs) = c.vertical;
        assert_ne!(hr, vr);
        let (h0, h1) = (routes[hr][hs], routes[hr][hs + 1]);
        let (v0, v1) = (routes[vr][vs], routes[vr][vs + 1]);
        assert_eq!((h0.y, h1.y, v0.x, v1.x), (c.at.y, c.at.y, c.at.x, c.at.x));
        assert!(h0.x.min(h1.x) < c.at.x && c.at.x < h0.x.max(h1.x));
        assert!(v0.y.min(v1.y) < c.at.y && c.at.y < v0.y.max(v1.y));
    }
}
