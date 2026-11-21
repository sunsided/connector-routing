# connector-routing

Global connector routing: many orthogonal connectors at once with bend, length and crossing costs, negotiated congestion and rip-up and reroute.

Routes connectors around rectangular obstacles, as diagram editors, schematic tools and cable-plan generators need: optimal single routes, many routes negotiating channel capacity, crossing reduction, and final placement that separates shared stretches. All geometry is exact integer arithmetic; the crate is `no_std` with `alloc`.

## Scope and status

Orthogonal routing:

- [x] Exact geometry, routing graph, orthogonal visibility graph (Wybrow, Marriott and Stuckey 2009) with a routing frame and port stubs, incremental updates for added, moved and removed obstacles and ports
- [x] Single connector: A* over `(vertex, heading)` with an exact free-space bend estimate; weighted and exact lexicographic length/bend objectives; checked against brute force on the unit grid
- [x] 1-bend visibility graph search (Marriott, Stuckey and Wybrow 2014), opt-in: tests show it is not exact in general, so the OVG search is the default
- [x] Global routing: PathFinder negotiated congestion over channel capacities, rip-up and reroute, crossing penalties, deterministic net order
- [x] Reports of the connectors a scene change invalidates or may improve
- [x] Path ordering in shared channels, required crossings, crossing reduction
- [x] Nudging: scan-line separation constraints (exported as data) solved exactly as difference constraints with water-filling, no LP; or VPSC projection of desired positions
- [x] Pipeline of arXiv 2309.01671: port assignment, visibility or channel routing graph, routing, crossing reduction, ordering, constrained or full nudging (boxes move and grow)
- [x] Hyperedges (Wybrow et al. 2012): Steiner trees by both heuristics with segment-shifting improvement, nudged like connectors
- [x] Pareto fronts over length, bends and crossings (feature `multi-objective`, via `multiobjective-search`)

Any-angle routing:

- [x] Sleeve routing (arXiv 2605.17498): exact constrained Delaunay triangulation, dual-graph Dijkstra, funnel algorithm, vertex-cover batching, exits along walls between touching obstacles
- [x] Incremental poly-line routing (Wybrow et al. 2005): visibility and invisibility graphs, rerouting reports, hidden-region pruning, segment, angle and crossing penalties

See [`docs/design.md`](docs/design.md) for the models, the deviations from the papers that testing found, and the paper table.

## Example

```rust
use connector_routing::{Connector, DirSet, Point, Port, Rect, Scene, global, nudge};

let mut scene = Scene::new();
let boxes = [Rect::new(0, 0, 20, 10), Rect::new(60, 0, 80, 10)];
for b in boxes {
    scene.add_obstacle(b).unwrap();
}

let a = scene.add_port(Port::new(Point::new(20, 5), DirSet::RIGHT)).unwrap();
let b = scene.add_port(Port::new(Point::new(60, 5), DirSet::LEFT)).unwrap();
let connectors = [Connector { source: a, target: b }];
let routing = global::route_all(&scene, &connectors, &global::Config::default()).unwrap();
let nudged = nudge::nudge(scene.graph(), &boxes, &routing.routes, &nudge::Config::default());
assert_eq!(nudged.routes[0], [Point::new(20, 5), Point::new(60, 5)]);
```

## Performance

On 192 boxes and 300 connectors (`cargo bench --bench routing`): the visibility graph has about 9,500 vertices and builds in 7 ms, global routing takes about 0.9 s, nudging 0.3 s, and the whole pipeline 1.3 s.

## Status

Pre-release. The API is a draft and will change.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
