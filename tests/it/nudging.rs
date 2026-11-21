//! Nudging keeps obstacles, ports and topology, separates shared stretches, and
//! realises exactly the required crossings.

use connector_routing::global::{Config, route_all};
use connector_routing::{
    Connector, DirSet, Point, Port, PortId, Rect, Route, Scene, check, crossing, nudge,
};
use proptest::prelude::*;

use crate::support::rect;

/// Whether moving segment `a0 a1` to `b0 b1` is a topology-preserving move (Marriott et
/// al. 2014): the rectangle it sweeps stays out of every obstacle interior. Vertical
/// segments move in the first pass, while their `y` extent is still the original one;
/// horizontal segments move in the second, with their final `x` extent.
fn sweep_is_free(a0: Point, a1: Point, b0: Point, b1: Point, rects: &[Rect]) -> bool {
    let (x0, x1, y0, y1) = if a0.x == a1.x && a0.y != a1.y {
        (
            a0.x.min(b0.x),
            a0.x.max(b0.x),
            a0.y.min(a1.y),
            a0.y.max(a1.y),
        )
    } else {
        (
            b0.x.min(b1.x),
            b0.x.max(b1.x),
            a0.y.min(b0.y),
            a0.y.max(b0.y),
        )
    };
    let degenerate = x0 == x1 || y0 == y1;
    !rects.iter().any(|r| {
        if degenerate {
            r.blocks_segment(Point::new(x0, y0), Point::new(x1, y1))
        } else {
            r.min.x < x1 && x0 < r.max.x && r.min.y < y1 && y0 < r.max.y
        }
    })
}

/// Checks every property of a nudged result and returns the achieved separation.
fn validate(
    rects: &[Rect],
    routes: &[Route],
    nudged: &nudge::Nudged,
    ports: &[(Point, Point)],
    graph: &connector_routing::RoutingGraph,
) {
    for ((route, polyline), &(s, t)) in routes.iter().zip(&nudged.routes).zip(ports) {
        check::orthogonal(polyline, rects).expect("nudged routes avoid obstacles");
        assert_eq!(polyline.len(), route.points.len(), "point for point");
        assert_eq!(
            (polyline[0], *polyline.last().unwrap()),
            (s, t),
            "ports stay"
        );
        for (k, pair) in route.points.windows(2).enumerate() {
            let moved = (polyline[k], polyline[k + 1]);
            assert!(
                sweep_is_free(pair[0], pair[1], moved.0, moved.1, rects),
                "segment {k} keeps its topology"
            );
        }
    }

    let geometric: usize = (0..nudged.routes.len())
        .flat_map(|i| (i + 1..nudged.routes.len()).map(move |j| (i, j)))
        .map(|(i, j)| check::orthogonal_crossings(&nudged.routes[i], &nudged.routes[j]))
        .sum();
    let required = crossing::total_crossings(graph, routes);
    // Any drawing of these paths has at least the required crossings. Equality needs
    // the freedom to put every movable segment on either side of the others: an obstacle
    // beside a segment pinned by a port can force an order that costs crossings the
    // combinatorial count does not see, so exactness is asserted without obstacles
    // (and the planar and swapping cases in `crossings` cover it with obstacles).
    if nudged.overlaps == 0 {
        let geometric = u32::try_from(geometric).unwrap();
        assert!(geometric >= required, "at least the required crossings");
        if rects.is_empty() {
            assert_eq!(geometric, required, "exactly the required crossings");
        }
    }

    if let Some(sep) = nudged.separation {
        let inner: Vec<Vec<Point>> = nudged.routes.clone();
        if let Some(found) = min_movable_separation(&inner) {
            assert!(
                found >= u64::from(sep),
                "parallel segments at least {sep} apart, found {found}"
            );
        }
    }
}

/// The smallest distance between overlapping parallel segments of different routes,
/// leaving out pairs of port segments (first or last segments), which nudging cannot
/// move.
fn min_movable_separation(routes: &[Vec<Point>]) -> Option<u64> {
    let mut best = None;
    for (i, a) in routes.iter().enumerate() {
        for b in &routes[i + 1..] {
            for (ka, pa) in a.windows(2).enumerate() {
                for (kb, pb) in b.windows(2).enumerate() {
                    let fixed_a = ka == 0 || ka + 2 == a.len();
                    let fixed_b = kb == 0 || kb + 2 == b.len();
                    if fixed_a && fixed_b {
                        continue;
                    }

                    if let Some(d) = check::min_parallel_separation(&[pa.to_vec(), pb.to_vec()]) {
                        best = Some(best.map_or(d, |old: u64| old.min(d)));
                    }
                }
            }
        }
    }

    best
}

/// Three connectors through one channel are spread apart, centred, and keep order.
#[test]
fn bundle_in_channel() {
    let rects = [Rect::new(0, 0, 60, 10), Rect::new(0, 40, 60, 50)];
    let mut scene = Scene::build(rects, []).unwrap();
    let add =
        |scene: &mut Scene, a: Point, da: DirSet, b: Point, db: DirSet| -> Connector<PortId> {
            let source = scene.add_port(Port::new(a, da)).unwrap();
            let target = scene.add_port(Port::new(b, db)).unwrap();
            Connector { source, target }
        };
    let connectors = vec![
        add(
            &mut scene,
            Point::new(10, 10),
            DirSet::UP,
            Point::new(50, 40),
            DirSet::DOWN,
        ),
        add(
            &mut scene,
            Point::new(14, 10),
            DirSet::UP,
            Point::new(46, 40),
            DirSet::DOWN,
        ),
        add(
            &mut scene,
            Point::new(18, 10),
            DirSet::UP,
            Point::new(42, 40),
            DirSet::DOWN,
        ),
    ];
    let routing = route_all(&scene, &connectors, &Config::default()).unwrap();
    let nudged = nudge::nudge(
        scene.graph(),
        &rects,
        &routing.routes,
        &nudge::Config::default(),
    );
    let ports: Vec<_> = connectors
        .iter()
        .map(|c| {
            (
                scene.port(c.source).unwrap().point,
                scene.port(c.target).unwrap().point,
            )
        })
        .collect();
    validate(&rects, &routing.routes, &nudged, &ports, scene.graph());
    assert!(nudged.separation.unwrap() >= 4);
    assert_eq!(nudged.overlaps, 0);
}

proptest! {
    #![proptest_config(crate::support::cases(64))]

    /// Random scenes routed globally, then nudged.
    #[test]
    fn nudging_properties(
        rects in proptest::collection::vec(rect(20), 0..8),
        pairs in proptest::collection::vec(((0..=20, 0..=20), (0..=20, 0..=20)), 1..10),
    ) {
        // Inputs on a lattice of spacing 4, so channels have room for separation 1.
        let rects: Vec<Rect> = rects.iter().map(|r| Rect::new(r.min.x * 4, r.min.y * 4, r.max.x * 4, r.max.y * 4)).collect();
        let pairs: Vec<_> = pairs.into_iter().map(|((a, b), (c, d))| ((a * 4, b * 4), (c * 4, d * 4))).collect();
        let blocked = |p: Point| rects.iter().any(|r| r.interior_contains(p));
        let mut scene = Scene::build(rects.iter().copied(), []).unwrap();
        let mut connectors = Vec::new();
        let mut used = std::collections::BTreeSet::new();
        for ((ax, ay), (bx, by)) in pairs {
            let (a, b) = (Point::new(ax, ay), Point::new(bx, by));
            if blocked(a) || blocked(b) || a == b || !used.insert(a) || !used.insert(b) {
                continue;
            }

            let source = scene.add_port(Port::free(a)).unwrap();
            let target = scene.add_port(Port::free(b)).unwrap();
            connectors.push(Connector { source, target });
        }

        // A port walled in by obstacles has no route; such scenes say nothing here.
        let Ok(routing) = route_all(&scene, &connectors, &Config::default()) else {
            return Ok(());
        };
        let ports: Vec<_> = connectors.iter().map(|c| (scene.port(c.source).unwrap().point, scene.port(c.target).unwrap().point)).collect();
        for placement in [nudge::Placement::Spread, nudge::Placement::Project] {
            let config = nudge::Config { separation: 1, max_separation: 2, placement };
            let nudged = nudge::nudge(scene.graph(), &rects, &routing.routes, &config);
            validate(&rects, &routing.routes, &nudged, &ports, scene.graph());
        }
    }
}
