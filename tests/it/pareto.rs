//! Pareto fronts (feature `multi-objective`) against the single-objective searches.

use connector_routing::global::{Config, route_all};
use connector_routing::{Connector, Objective, Point, Port, Scene, Search, check, pareto, route};
use proptest::prelude::*;

use crate::support::rect;

proptest! {
    #![proptest_config(crate::support::cases(48))]

    /// Front members are valid and mutually non-dominated; with no other routes, the
    /// optimum of every weighted sum of length and bends is on the front.
    #[test]
    fn front_contains_weighted_optima(
        rects in proptest::collection::vec(rect(12), 0..5),
        a in (0..=12, 0..=12),
        b in (0..=12, 0..=12),
        bend in 0_u64..30,
    ) {
        let rects: Vec<_> = rects.iter().map(|r| connector_routing::Rect::new(r.min.x * 4, r.min.y * 4, r.max.x * 4, r.max.y * 4)).collect();
        let (a, b) = (Point::new(a.0 * 4, a.1 * 4), Point::new(b.0 * 4, b.1 * 4));
        prop_assume!(a != b && !rects.iter().any(|r| r.interior_contains(a) || r.interior_contains(b)));
        let scene = Scene::build(rects.iter().copied(), [Port::free(a), Port::free(b)]).unwrap();
        let ids: Vec<_> = scene.ports().map(|(id, _)| id).collect();
        let front = pareto::front(&scene, ids[0], ids[1], &[]).unwrap();
        for p in &front {
            check::orthogonal(&p.route.points, &rects).unwrap();
            prop_assert_eq!(p.cost[0], p.route.length);
            prop_assert_eq!(p.cost[1], u64::from(p.route.bends));
        }

        for (i, x) in front.iter().enumerate() {
            for y in &front[i + 1..] {
                let dominates = |p: [u64; 3], q: [u64; 3]| p.iter().zip(&q).all(|(a, b)| a <= b) && p != q;
                prop_assert!(!dominates(x.cost, y.cost) && !dominates(y.cost, x.cost));
            }
        }

        let single = route(&scene, ids[0], ids[1], Objective::Weighted { length: 1, bend }, Search::Ovg).unwrap();
        let best = front.iter().map(|p| p.cost[0] + bend * p.cost[1]).min();
        prop_assert_eq!(best, Some(single.cost));
    }
}

/// Crossings with fixed routes are a third objective: a detour without the crossing
/// joins the front next to the straight route that crosses.
#[test]
fn crossings_join_the_front() {
    let mut scene = Scene::new();
    let blocker_a = scene.add_port(Port::free(Point::new(10, -20))).unwrap();
    let blocker_b = scene.add_port(Port::free(Point::new(10, 20))).unwrap();
    let a = scene.add_port(Port::free(Point::new(0, 0))).unwrap();
    let b = scene.add_port(Port::free(Point::new(20, 0))).unwrap();
    let wall = route_all(
        &scene,
        &[Connector {
            source: blocker_a,
            target: blocker_b,
        }],
        &Config::default(),
    )
    .unwrap();
    let front = pareto::front(&scene, a, b, &wall.routes).unwrap();
    assert!(
        front.iter().any(|p| p.cost[2] == 1 && p.cost[1] == 0),
        "straight through, one crossing"
    );
    assert!(
        front.iter().any(|p| p.cost[2] == 0),
        "around the wall's end, no crossing"
    );
}
