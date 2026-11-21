//! Single-connector routing against brute force on the unit grid.

use connector_routing::{
    Objective, Point, Port, Rect, Route, Scene, Search, check, cost_of, route,
};
use proptest::prelude::*;

use crate::support::{grid_optimum, interesting, port, rect};

/// A random objective: weighted with small weights, or lexicographic.
fn objective() -> impl Strategy<Value = Objective> {
    prop_oneof![
        Just(Objective::ShortestThenFewestBends),
        Just(Objective::FewestBendsThenShortest),
        (1_u64..4, 0_u64..25).prop_map(|(length, bend)| Objective::Weighted { length, bend }),
    ]
}

/// Checks the structural properties of a route and returns it.
fn validate(scene: &Scene, r: &Route, source: Port, target: Port, rects: &[Rect]) {
    check::orthogonal(&r.points, rects).expect("the route avoids every obstacle interior");
    assert_eq!(r.points.first(), Some(&source.point));
    assert_eq!(r.points.last(), Some(&target.point));
    if let Some(d) = r.first_direction() {
        assert!(
            source.directions.contains(d),
            "leaves in an allowed direction"
        );
    }

    if let Some(d) = r.last_direction() {
        assert!(
            target.directions.reversed().contains(d),
            "arrives in an allowed direction"
        );
    }

    let length: u64 = r.points.windows(2).map(|w| w[0].manhattan(w[1])).sum();
    assert_eq!(length, r.length);
    let _ = scene;
}

/// The worked example: around a box.
#[test]
fn around_a_box() {
    let rects = [Rect::new(0, 0, 10, 10)];
    let a = Port::new(Point::new(-5, 5), connector_routing::DirSet::RIGHT);
    let b = Port::new(Point::new(15, 5), connector_routing::DirSet::LEFT);
    let scene = Scene::build(rects, [a, b]).unwrap();
    let ids: Vec<_> = scene.ports().map(|(id, _)| id).collect();
    for search in [Search::Ovg, Search::OneBend] {
        let r = route(&scene, ids[0], ids[1], Objective::default(), search).unwrap();
        assert_eq!((r.length, r.bends), (30, 4));
        validate(&scene, &r, a, b, &rects);
    }
}

/// Checks one random case: the OVG A* and, if `one_bend_exact`, the 1-bend search
/// against the brute-force optimum; otherwise both only against invalid or too cheap
/// routes.
fn compare(
    rects: &[Rect],
    source: Port,
    target: Port,
    objective: Objective,
    exact: (bool, bool),
) -> Result<(), TestCaseError> {
    let scene = Scene::build(rects.iter().copied(), [source, target]).unwrap();
    let ids: Vec<_> = scene.ports().map(|(id, _)| id).collect();
    let weights = objective.weights(scene.graph()).unwrap();
    let points = interesting(rects, &[source, target]);
    let expected = grid_optimum(
        rects,
        &points,
        source,
        target,
        (weights.length, weights.bend),
        1,
    );
    prop_assert_eq!(
        expected,
        grid_optimum(
            rects,
            &points,
            source,
            target,
            (weights.length, weights.bend),
            4
        ),
        "padding the grid does not change the optimum"
    );

    for (search, exact) in [(Search::Ovg, exact.0), (Search::OneBend, exact.1)] {
        let found = route(&scene, ids[0], ids[1], objective, search).ok();
        prop_assert_eq!(
            found.is_some(),
            expected.is_some(),
            "{:?} finds a route iff one exists",
            search
        );
        let Some(r) = found else { continue };
        validate(&scene, &r, source, target, rects);
        prop_assert_eq!(cost_of(weights, r.length, r.bends), r.cost);
        if exact {
            prop_assert_eq!(
                Some(r.cost),
                expected,
                "{:?} {:?} {:?} {:?} {:?} {:?}",
                search,
                r.points,
                weights,
                source,
                target,
                rects
            );
        } else {
            prop_assert!(Some(r.cost) >= expected, "{:?}", search);
        }
    }

    Ok(())
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(400))]

    /// Ports that may be left in any direction, anywhere outside obstacles: both
    /// searches find the brute-force optimum, with touching and overlapping obstacles.
    ///
    /// Inputs lie on a lattice of spacing [`SCALE`] and the brute force searches the
    /// unit grid, which is finer: a route may use any integer line. (With spacing one,
    /// port stubs can coincide with obstacle sides and a route squeezing through a gap of
    /// one unit needs lines the graph does not have; see `valid_with_any_port_directions`.)
    #[test]
    fn optimal_with_free_ports(
        rects in proptest::collection::vec(rect(12), 0..6),
        a in (0..=12, 0..=12),
        b in (0..=12, 0..=12),
        objective in objective(),
    ) {
        let rects = scaled(&rects);
        let source = Port::free(Point::new(a.0 * SCALE, a.1 * SCALE));
        let target = Port::free(Point::new(b.0 * SCALE, b.1 * SCALE));
        let blocked = |p: Point| rects.iter().any(|r| r.interior_contains(p));
        prop_assume!(!blocked(source.point) && !blocked(target.point));
        compare(&rects, source, target, objective, (true, true))?;
    }

    /// Ports on obstacle sides pointing outwards (the diagram case the papers assume):
    /// the OVG search finds the brute-force optimum; the 1-bend search is not exact here
    /// (see `one_bend_misses_slit_between_touching_boxes`).
    #[test]
    fn optimal_with_side_ports(
        rects in proptest::collection::vec(rect(12), 1..6),
        picks in proptest::collection::vec((any::<usize>(), 0_usize..4, 1_u32..64), 2),
        objective in objective(),
    ) {
        let rects = scaled(&rects);
        let ports: Vec<Port> = picks.iter().map(|&(i, side, t)| side_port(&rects[i % rects.len()], side, t)).collect();
        let blocked = |p: Point| rects.iter().any(|r| r.interior_contains(p));
        prop_assume!(!blocked(ports[0].point) && !blocked(ports[1].point));
        compare(&rects, ports[0], ports[1], objective, (true, false))?;
    }

    /// Arbitrary direction sets: routes are always valid, found whenever one exists,
    /// and never cheaper than brute force.
    #[test]
    fn valid_with_any_port_directions(
        rects in proptest::collection::vec(rect(16), 0..6),
        source in port(16),
        target in port(16),
        objective in objective(),
    ) {
        let blocked = |p: Point| rects.iter().any(|r| r.interior_contains(p));
        prop_assume!(!blocked(source.point) && !blocked(target.point));
        compare(&rects, source, target, objective, (false, false))?;
    }
}

/// The lattice spacing of the exactness tests.
const SCALE: i32 = 4;

/// The rectangles scaled by [`SCALE`].
fn scaled(rects: &[Rect]) -> Vec<Rect> {
    rects
        .iter()
        .map(|r| {
            Rect::new(
                r.min.x * SCALE,
                r.min.y * SCALE,
                r.max.x * SCALE,
                r.max.y * SCALE,
            )
        })
        .collect()
}

/// A port on side `side` of `rect` (0 right, 1 top, 2 left, 3 bottom), at an interior
/// position chosen by `t`, pointing outwards.
fn side_port(rect: &Rect, side: usize, t: u32) -> Port {
    let side = connector_routing::Side::ALL[side];
    let along = |lo: i32, hi: i32| {
        let span = u32::try_from(hi - lo - 1).unwrap_or(0).max(1);
        (lo + 1 + i32::try_from(t % span).unwrap())
            .min(hi - 1)
            .max(lo)
    };
    let point = match side {
        connector_routing::Side::Right => Point::new(rect.max.x, along(rect.min.y, rect.max.y)),
        connector_routing::Side::Left => Point::new(rect.min.x, along(rect.min.y, rect.max.y)),
        connector_routing::Side::Top => Point::new(along(rect.min.x, rect.max.x), rect.max.y),
        connector_routing::Side::Bottom => Point::new(along(rect.min.x, rect.max.x), rect.min.y),
    };

    Port::new(point, side.outward())
}

/// Regression for a gap in the paper's graph: a port on the outer side of the outermost
/// obstacle, pointing outwards, needs a line beyond every interesting point. The
/// routing frame and the port stubs provide it.
#[test]
fn outward_port_needs_frame_and_stub() {
    let rects = [Rect::new(0, 0, 10, 10)];
    let a = Port::new(Point::new(0, 5), connector_routing::DirSet::LEFT);
    let b = Port::new(Point::new(10, 5), connector_routing::DirSet::RIGHT);
    let scene = Scene::build(rects, [a, b]).unwrap();
    let ids: Vec<_> = scene.ports().map(|(id, _)| id).collect();
    let r = route(&scene, ids[0], ids[1], Objective::default(), Search::Ovg).unwrap();
    assert_eq!((r.length, r.bends), (24, 4));
    validate(&scene, &r, a, b, &rects);
}

/// Regression: the 1-bend search is not exact. Two boxes touch along a slit; the optimal
/// route (length 20, 6 bends) leaves its port, turns back into the slit and squeezes
/// through it, turning where no obstacle corner supports the segment. The 1-bend search
/// finds only a route around the box (length 38, 4 bends). Found by the lattice
/// property test after the bend-estimate and port-stub fixes.
#[test]
fn one_bend_misses_slit_between_touching_boxes() {
    let rects = [Rect::new(8, 24, 16, 40), Rect::new(16, 16, 20, 28)];
    let a = Port::new(Point::new(16, 28), connector_routing::DirSet::RIGHT);
    let b = Port::new(Point::new(8, 26), connector_routing::DirSet::LEFT);
    let scene = Scene::build(rects, [a, b]).unwrap();
    let ids: Vec<_> = scene.ports().map(|(id, _)| id).collect();
    let objective = Objective::ShortestThenFewestBends;
    let ovg = route(&scene, ids[0], ids[1], objective, Search::Ovg).unwrap();
    let one_bend = route(&scene, ids[0], ids[1], objective, Search::OneBend).unwrap();
    assert_eq!((ovg.length, ovg.bends), (20, 6));
    assert_eq!((one_bend.length, one_bend.bends), (38, 4));
    validate(&scene, &one_bend, a, b, &rects);
}
