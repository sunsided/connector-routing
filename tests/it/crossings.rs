//! Required crossings, crossing reduction, and the guarantees of Wybrow et al. (2009,
//! Theorems 3 and 4) for the ordering.

use connector_routing::global::{Config, route_all};
use connector_routing::{
    Connector, DirSet, Direction, Objective, Point, Port, Rect, Route, RoutingGraph, Scene,
    VertexId, check, crossing, nudge,
};

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
    };
    let nudged = nudge::nudge(g, &[], &routes, &config);
    assert_eq!(nudged.overlaps, 0);
    assert_eq!(
        check::orthogonal_crossings(&nudged.routes[0], &nudged.routes[1]),
        1
    );
}
