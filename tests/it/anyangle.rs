//! Any-angle routing: poly-line routes against an independent visibility oracle, their
//! incremental maintenance, and sleeve routes.

use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap};

use connector_routing::polyline::{Config, PolylineRouter};
use connector_routing::sleeve::SleeveRouter;
use connector_routing::{Point, Rect, check};
use proptest::prelude::*;

/// The fixed-point length the poly-line router uses, rounded up.
fn length_up(a: Point, b: Point) -> u64 {
    let squared = a.distance_squared(b) << 16;
    let root = squared.isqrt();
    u64::try_from(if root * root == squared {
        root
    } else {
        root + 1
    })
    .unwrap()
}

/// The shortest obstacle-avoiding route cost from `a` to `b` by Dijkstra over corners,
/// with visibility decided by the independent checker.
fn oracle(rects: &[Rect], a: Point, b: Point) -> Option<u64> {
    let mut nodes: Vec<Point> = rects.iter().flat_map(Rect::corners).collect();
    nodes.extend([a, b]);
    nodes.sort();
    nodes.dedup();
    let mut best: BTreeMap<Point, u64> = BTreeMap::new();
    let mut heap = BinaryHeap::from([Reverse((0_u64, a))]);
    while let Some(Reverse((cost, p))) = heap.pop() {
        if best.contains_key(&p) {
            continue;
        }

        best.insert(p, cost);
        if p == b {
            return Some(cost);
        }

        for &q in &nodes {
            if !best.contains_key(&q) && check::polyline(&[p, q], rects).is_ok() {
                heap.push(Reverse((cost + length_up(p, q), q)));
            }
        }
    }

    None
}

/// The Euclidean length of a polyline.
fn euclid(route: &[Point]) -> f64 {
    route
        .windows(2)
        .map(|w| f64::from(w[0].x - w[1].x).hypot(f64::from(w[0].y - w[1].y)))
        .sum()
}

/// Rectangles with disjoint interiors: one per chosen cell of a coarse lattice, filling
/// up to the whole cell, so neighbours may touch along sides and at corners.
fn disjoint_rects() -> impl Strategy<Value = Vec<Rect>> {
    proptest::collection::btree_set((0_i32..5, 0_i32..5, 1_i32..=10, 1_i32..=10), 0..12).prop_map(
        |cells| {
            let mut used = std::collections::BTreeSet::new();
            cells
                .into_iter()
                .filter(|&(i, j, _, _)| used.insert((i, j)))
                .map(|(i, j, w, h)| Rect::new(i * 10, j * 10, i * 10 + w, j * 10 + h))
                .collect()
        },
    )
}

proptest! {
    #![proptest_config(crate::support::cases(64))]

    /// Poly-line routes are valid and as short as the independent oracle's.
    #[test]
    fn polyline_is_optimal(rects in disjoint_rects(), a in (0..50, 0..50), b in (0..50, 0..50)) {
        let (a, b) = (Point::new(a.0, a.1), Point::new(b.0, b.1));
        let blocked = |p: Point| rects.iter().any(|r| r.interior_contains(p));
        prop_assume!(!blocked(a) && !blocked(b) && a != b);
        let mut router = PolylineRouter::new(Config::default());
        for r in &rects {
            router.add_obstacle(*r).unwrap();
        }

        let c = router.add_connector(a, b).unwrap();
        let route = router.route(c).unwrap().to_vec();
        check::polyline(&route, &rects).unwrap();
        prop_assert_eq!((route[0], *route.last().unwrap()), (a, b));
        prop_assert_eq!(router.cost(c), oracle(&rects, a, b));
    }

    /// After random obstacle additions and removals, rerouting only the reported
    /// connectors keeps every connector optimal, and the visibility graph equals one
    /// built from scratch.
    #[test]
    fn polyline_incremental(
        rects in disjoint_rects(),
        order in proptest::collection::vec(any::<bool>(), 1..12),
        ends in proptest::collection::vec(((0..50, 0..50), (0..50, 0..50)), 1..5),
    ) {
        let mut router = PolylineRouter::new(Config::default());
        let mut present: Vec<(connector_routing::ObstacleId, Rect)> = Vec::new();
        let mut pending: Vec<Rect> = rects.clone();
        let mut connectors = Vec::new();
        for ((ax, ay), (bx, by)) in ends {
            let (a, b) = (Point::new(ax, ay), Point::new(bx, by));
            if rects.iter().any(|r| r.interior_contains(a) || r.interior_contains(b)) {
                continue;
            }

            connectors.push((router.add_connector(a, b).unwrap(), a, b));
        }

        for add in order {
            let change = if add && !pending.is_empty() {
                let r = pending.remove(0);
                let (id, change) = router.add_obstacle(r).unwrap();
                present.push((id, r));
                change
            } else if let Some((id, r)) = present.pop() {
                pending.push(r);
                router.remove_obstacle(id).unwrap()
            } else {
                continue;
            };
            for c in change.invalidated.iter().chain(&change.may_improve) {
                router.reroute(*c);
            }

            let now: Vec<Rect> = present.iter().map(|&(_, r)| r).collect();
            let mut fresh = PolylineRouter::new(Config::default());
            for r in &now {
                fresh.add_obstacle(*r).unwrap();
            }

            for &(c, a, b) in &connectors {
                let f = fresh.add_connector(a, b).unwrap();
                prop_assert_eq!(router.cost(c), fresh.cost(f), "connector {:?} stays optimal", c);
            }

            prop_assert_eq!(router.visibility_edges(), fresh.visibility_edges());
        }
    }

    /// Batched sleeve routes are valid, connect their endpoints, and are never shorter
    /// than the shortest route.
    #[test]
    fn sleeve_routes(rects in disjoint_rects(), pairs in proptest::collection::vec(((0..50, 0..50), (0..50, 0..50)), 1..6)) {
        let blocked = |p: Point| rects.iter().any(|r| r.interior_contains(p));
        let pairs: Vec<(Point, Point)> = pairs
            .into_iter()
            .map(|((a, b), (c, d))| (Point::new(a, b), Point::new(c, d)))
            .filter(|&(a, b)| !blocked(a) && !blocked(b))
            .collect();
        let endpoints: Vec<Point> = pairs.iter().flat_map(|&(a, b)| [a, b]).collect();
        let router = SleeveRouter::new(&rects, &endpoints, 5).unwrap();
        let batched = router.route_all(&pairs);
        // Only a walled-in endpoint is out of reach.
        for (&(a, b), route) in pairs.iter().zip(&batched) {
            let Some(route) = route.as_ref() else {
                prop_assert!(oracle(&rects, a, b).is_none());
                continue;
            };
            check::polyline(route, &rects).unwrap();
            prop_assert_eq!((route[0], *route.last().unwrap()), (a, b));
            let optimum = f64::from(u32::try_from(oracle(&rects, a, b).unwrap()).unwrap()) / 256.0;
            // The oracle rounds every edge up by less than 1/256.
            prop_assert!(euclid(route) >= optimum - 0.05);
        }
    }
}

/// An endpoint on a wall shared by two touching boxes has no free triangle around it
/// (the slit has no area): sleeve routing leaves along the wall to its nearest end, as
/// poly-line routing does.
#[test]
fn sleeve_reaches_a_shared_wall() {
    let rects = [Rect::new(0, 0, 10, 10), Rect::new(10, 0, 20, 10)];
    let (inside, outside) = (Point::new(10, 4), Point::new(30, 5));
    let router = SleeveRouter::new(&rects, &[inside, outside], 5).unwrap();
    let route = router.route(outside, inside).unwrap();
    check::polyline(&route, &rects).unwrap();
    assert_eq!((route[0], *route.last().unwrap()), (outside, inside));
    assert!(
        route.contains(&Point::new(10, 0)),
        "leaves the slit at its lower end: {route:?}"
    );

    let mut polyline = PolylineRouter::new(Config::default());
    for r in rects {
        polyline.add_obstacle(r).unwrap();
    }

    let c = polyline.add_connector(outside, inside).unwrap();
    let route = polyline.route(c).unwrap().to_vec();
    check::polyline(&route, &rects).unwrap();
}

proptest! {
    #![proptest_config(crate::support::cases(64))]

    /// Hidden-region pruning changes no cost, and segment and angle penalties keep
    /// routes valid and never make them cheaper than plain length.
    #[test]
    fn pruning_and_angle_penalty(
        rects in disjoint_rects(),
        a in (0..50, 0..50),
        b in (0..50, 0..50),
        segment in 0_u64..20,
        angle in 1_u64..40,
    ) {
        let (a, b) = (Point::new(a.0, a.1), Point::new(b.0, b.1));
        prop_assume!(a != b && !rects.iter().any(|r| r.interior_contains(a) || r.interior_contains(b)));
        let cost = |config: Config| {
            let mut router = PolylineRouter::new(config);
            for r in &rects {
                router.add_obstacle(*r).unwrap();
            }
            let c = router.add_connector(a, b).unwrap();
            if let Some(route) = router.route(c) {
                check::polyline(route, &rects).unwrap();
            }
            router.cost(c)
        };
        let plain = cost(Config::default());
        prop_assert_eq!(plain, cost(Config { hidden_region_pruning: false, ..Config::default() }));
        let segmented = cost(Config { segment_penalty: segment, ..Config::default() });
        prop_assert!(segmented >= plain);
        let angled = cost(Config { angle_penalty: angle, ..Config::default() });
        prop_assert_eq!(plain.is_some(), angled.is_some());
        prop_assert!(angled >= plain);
    }
}

/// The angle penalty prefers one sharp turn less than two gentle ones in a way that a
/// pure length cost does not: a large `beta` straightens a route that bends around a
/// corner it barely touches.
#[test]
fn angle_classes() {
    let route_cost = |beta: u64| {
        let mut router = PolylineRouter::new(Config {
            angle_penalty: beta,
            ..Config::default()
        });
        router.add_obstacle(Rect::new(10, -1, 20, 1)).unwrap();
        let c = router
            .add_connector(Point::new(0, 0), Point::new(30, 0))
            .unwrap();
        (router.route(c).unwrap().len(), router.cost(c).unwrap())
    };
    let (len0, cost0) = route_cost(0);
    let (_, cost1) = route_cost(10);
    assert_eq!(len0, 4);
    assert!(cost1 > cost0, "bends cost extra: {cost0} {cost1}");
}

/// An obstacle added over a connector's endpoint leaves it without a route, even for a
/// connector from a point to itself (found by the `polyline_route` fuzz target).
#[test]
fn polyline_endpoint_covered_later_has_no_route() {
    let mut router = PolylineRouter::new(Config::default());
    let p = Point::new(5, 5);
    let point = router.add_connector(p, p).unwrap();
    let line = router.add_connector(p, Point::new(20, 5)).unwrap();
    assert_eq!(router.route(point), Some(&[p][..]));

    router.add_obstacle(Rect::new(0, 0, 10, 10)).unwrap();
    for id in [point, line] {
        assert!(!router.reroute(id));
        assert_eq!(router.route(id), None);
    }
}
