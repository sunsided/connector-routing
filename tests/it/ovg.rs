//! The orthogonal visibility graph against its definition, and incremental updates
//! against rebuilding from scratch.

use connector_routing::{Point, Port, Rect, Scene};
use proptest::prelude::*;

use crate::support::{interesting, naive_ovg, port, rect};

/// The canonical form of a scene's graph, with edges sorted like the naive oracle's.
fn canonical(scene: &Scene) -> (Vec<Point>, Vec<(Point, Point)>) {
    let (points, mut edges) = scene.graph().canonical();
    edges.sort();

    (points, edges)
}

/// A hand-checked scene: two boxes side by side and a port between them.
#[test]
fn two_boxes() {
    let rects = [Rect::new(0, 0, 4, 4), Rect::new(8, 0, 12, 4)];
    let ports = [Port::free(Point::new(6, 2))];
    let scene = Scene::build(rects, ports).unwrap();
    let (points, edges) = canonical(&scene);

    // The port's horizontal line stops at both boxes; its vertical line is unbounded.
    assert!(points.contains(&Point::new(4, 2)));
    assert!(points.contains(&Point::new(8, 2)));
    assert!(points.contains(&Point::new(6, 0)));
    assert!(points.contains(&Point::new(6, 4)));
    assert!(!points.contains(&Point::new(2, 2)));
    assert!(edges.contains(&(Point::new(4, 2), Point::new(5, 2))));
    assert!(
        points.contains(&Point::new(7, 2)),
        "the port's stubs are interesting"
    );
    assert_eq!(
        (points, edges),
        naive_ovg(&rects, &interesting(&rects, &ports))
    );
}

/// One step of a random edit sequence.
#[derive(Debug, Clone, Copy)]
enum Edit {
    /// Add an obstacle.
    Add(Rect),
    /// Remove the obstacle with this index (modulo the live count).
    Remove(usize),
    /// Move the obstacle with this index to a new rectangle.
    Move(usize, Rect),
    /// Add a port.
    AddPort(Port),
    /// Remove the port with this index.
    RemovePort(usize),
}

/// Random edits in a 30 x 30 area.
fn edit() -> impl Strategy<Value = Edit> {
    prop_oneof![
        3 => rect(30).prop_map(Edit::Add),
        1 => any::<usize>().prop_map(Edit::Remove),
        2 => (any::<usize>(), rect(30)).prop_map(|(i, r)| Edit::Move(i, r)),
        2 => port(30).prop_map(Edit::AddPort),
        1 => any::<usize>().prop_map(Edit::RemovePort),
    ]
}

proptest! {
    #![proptest_config(crate::support::cases(200))]

    /// The batch build equals the definition on random scenes, overlaps included.
    #[test]
    fn build_matches_definition(
        rects in proptest::collection::vec(rect(30), 0..8),
        ports in proptest::collection::vec(port(30), 0..6),
    ) {
        let scene = Scene::build(rects.iter().copied(), ports.iter().copied()).unwrap();
        prop_assert_eq!(canonical(&scene), naive_ovg(&rects, &interesting(&rects, &ports)));
    }

    /// After every random add, move and remove, the incrementally updated graph equals
    /// the graph rebuilt from scratch.
    #[test]
    fn incremental_equals_rebuild(edits in proptest::collection::vec(edit(), 1..25)) {
        let mut scene = Scene::new();
        for edit in edits {
            apply(&mut scene, edit);
            let rects = scene.rects();
            let ports: Vec<Port> = scene.ports().map(|(_, p)| p).collect();
            let fresh = Scene::build(rects, ports).unwrap();
            prop_assert_eq!(canonical(&scene), canonical(&fresh));
        }
    }
}

/// Applies one edit to the scene.
fn apply(scene: &mut Scene, edit: Edit) {
    let obstacles: Vec<_> = scene.obstacles().map(|(id, _)| id).collect();
    let ports: Vec<_> = scene.ports().map(|(id, _)| id).collect();
    match edit {
        Edit::Add(r) => {
            scene.add_obstacle(r).unwrap();
        }
        Edit::Remove(i) if !obstacles.is_empty() => {
            scene
                .remove_obstacle(obstacles[i % obstacles.len()])
                .unwrap();
        }
        Edit::Move(i, r) if !obstacles.is_empty() => {
            scene
                .move_obstacle(obstacles[i % obstacles.len()], r)
                .unwrap();
        }
        Edit::AddPort(p) => {
            scene.add_port(p).unwrap();
        }
        Edit::RemovePort(i) if !ports.is_empty() => {
            scene.remove_port(ports[i % ports.len()]).unwrap();
        }
        _ => {}
    }
}
