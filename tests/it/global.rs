//! Negotiated congestion: convergence on solvable cases, validity, determinism.

use connector_routing::global::{Config, route_all};
use connector_routing::{Connector, DirSet, Point, Port, PortId, Rect, Scene, check};
use proptest::prelude::*;

use crate::support::rect;

/// A corridor between two long boxes, `height` units high, at `y` from 9 upwards.
fn corridor(height: i32) -> Vec<Rect> {
    vec![Rect::new(0, -20, 40, 9), Rect::new(0, 9 + height, 40, 40)]
}

/// Adds a connector from `(x0, y)` leaving right to `(x1, y)` arriving from the left.
fn straight(scene: &mut Scene, y: i32, x0: i32, x1: i32) -> Connector<PortId> {
    let source = scene
        .add_port(Port::new(Point::new(x0, y), DirSet::RIGHT))
        .unwrap();
    let target = scene
        .add_port(Port::new(Point::new(x1, y), DirSet::LEFT))
        .unwrap();

    Connector { source, target }
}

/// Whether a route runs through the corridor (strictly between the boxes' x range).
fn through_corridor(points: &[Point], low: i32, high: i32) -> bool {
    points.windows(2).any(|w| {
        w[0].y == w[1].y
            && (low..=high).contains(&w[0].y)
            && w[0].x.min(w[1].x) < 20
            && w[0].x.max(w[1].x) > 20
    })
}

/// Three connectors want a corridor of two tracks; going around is feasible, so the
/// overuse must reach zero with exactly two inside.
#[test]
fn corridor_overuse_goes_to_zero() {
    let rects = corridor(4);
    let mut scene = Scene::new();
    for r in &rects {
        scene.add_obstacle(*r).unwrap();
    }

    let connectors: Vec<_> = (0..3)
        .map(|i| straight(&mut scene, 10 + i, -10 - i, 50 + i))
        .collect();
    let config = Config::default();
    let routing = route_all(&scene, &connectors, &config).unwrap();

    assert_eq!(
        routing.overuse, 0,
        "after {} iterations",
        routing.iterations
    );
    assert!(
        routing.iterations > 1,
        "the first iteration overloads the corridor"
    );
    let inside = routing
        .routes
        .iter()
        .filter(|r| through_corridor(&r.points, 9, 13))
        .count();
    assert_eq!(inside, 2);
    for r in &routing.routes {
        check::orthogonal(&r.points, &rects).unwrap();
    }
}

/// The same request routes identically twice.
#[test]
fn deterministic() {
    let rects = corridor(4);
    let mut scene = Scene::build(rects.iter().copied(), []).unwrap();
    let connectors: Vec<_> = (0..4)
        .map(|i| straight(&mut scene, 10 + i, -10 - i, 50 + i))
        .collect();
    let a = route_all(&scene, &connectors, &Config::default()).unwrap();
    let b = route_all(&scene, &connectors, &Config::default()).unwrap();
    assert_eq!(a, b);
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(40))]

    /// Random scenes and connectors: every route is valid and connects its ports.
    #[test]
    fn routes_are_valid(
        rects in proptest::collection::vec(rect(40), 0..10),
        pairs in proptest::collection::vec(((0..=40, 0..=40), (0..=40, 0..=40)), 1..12),
    ) {
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

        let routing = route_all(&scene, &connectors, &Config::default()).unwrap();
        for (c, r) in connectors.iter().zip(&routing.routes) {
            check::orthogonal(&r.points, &rects).unwrap();
            prop_assert_eq!(r.points.first(), Some(&scene.port(c.source).unwrap().point));
            prop_assert_eq!(r.points.last(), Some(&scene.port(c.target).unwrap().point));
        }
    }
}

/// Moving an obstacle onto a route reports it; removing the obstacle a route detours
/// around reports it as improvable, and rerouting it is indeed cheaper.
#[test]
fn incremental_reports() {
    use connector_routing::{Objective, Search, incremental, route};

    let mut scene = Scene::new();
    let block = scene.add_obstacle(Rect::new(10, 0, 20, 10)).unwrap();
    let a = scene
        .add_port(Port::new(Point::new(0, 5), DirSet::RIGHT))
        .unwrap();
    let b = scene
        .add_port(Port::new(Point::new(30, 5), DirSet::LEFT))
        .unwrap();
    let objective = Objective::Weighted {
        length: 1,
        bend: 10,
    };
    let around = route(&scene, a, b, objective, Search::Ovg).unwrap();
    let weights = objective.weights(scene.graph()).unwrap();

    scene.remove_obstacle(block).unwrap();
    assert_eq!(
        incremental::may_improve(
            std::slice::from_ref(&around),
            Rect::new(10, 0, 20, 10),
            weights
        ),
        [0]
    );
    let straight = route(&scene, a, b, objective, Search::Ovg).unwrap();
    assert!(straight.cost < around.cost);

    assert_eq!(
        incremental::invalidated_by(&[straight], Rect::new(10, 0, 20, 10)),
        [0]
    );
    assert_eq!(
        incremental::invalidated_by(std::slice::from_ref(&around), Rect::new(10, 0, 20, 10)),
        Vec::<usize>::new()
    );
    assert_eq!(
        incremental::may_improve(&[around], Rect::new(100, 100, 110, 110), weights),
        Vec::<usize>::new()
    );
}
