//! Benchmarks with hundreds of connectors: scene construction, global routing, nudging,
//! the full pipeline, single-connector searches, sleeve and poly-line routing.

use std::hint::black_box;

use connector_routing::global::{self, route_all};
use connector_routing::polyline::{self, PolylineRouter};
use connector_routing::sleeve::SleeveRouter;
use connector_routing::{
    Connector, Objective, Point, Port, Rect, Scene, Search, nudge, pipeline, route,
};
use criterion::Criterion;

/// A grid of `cols x rows` boxes, 40 x 24 each, 80 apart.
fn boxes(cols: i32, rows: i32) -> Vec<Rect> {
    (0..cols)
        .flat_map(|i| (0..rows).map(move |j| Rect::new(i * 80, j * 80, i * 80 + 40, j * 80 + 24)))
        .collect()
}

/// `count` pseudo-random edges between distinct boxes, deterministic.
fn edges(n: usize, count: usize) -> Vec<(usize, usize)> {
    let mut seed: u64 = 0x5eed;
    let mut next = || {
        seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        usize::try_from(seed >> 33).unwrap() % n
    };
    (0..count)
        .map(|_| {
            loop {
                let (a, b) = (next(), next());
                if a != b {
                    break (a, b);
                }
            }
        })
        .collect()
}

/// The scene and connectors of a pipeline-style instance.
fn instance(
    cols: i32,
    rows: i32,
    count: usize,
) -> (Scene, Vec<Connector<connector_routing::PortId>>, Vec<Rect>) {
    let rects = boxes(cols, rows);
    let es = edges(rects.len(), count);
    let ports = pipeline::assign_ports(&rects, &es);
    let all: Vec<Port> = ports.iter().flat_map(|&(a, b)| [a, b]).collect();
    let scene = Scene::build(rects.iter().copied(), all).unwrap();
    let ids: Vec<_> = scene.ports().map(|(id, _)| id).collect();
    let connectors = ids
        .chunks(2)
        .map(|c| Connector {
            source: c[0],
            target: c[1],
        })
        .collect();

    (scene, connectors, rects)
}

/// Runs every benchmark on one instance of 192 boxes and 300 connectors.
fn benches(c: &mut Criterion) {
    let (scene, connectors, rects) = instance(16, 12, 300);
    eprintln!(
        "instance: {} boxes, {} connectors, OVG {} vertices",
        rects.len(),
        connectors.len(),
        scene.graph().len()
    );

    let mut group = c.benchmark_group("orthogonal");
    group.sample_size(10);
    group.bench_function("scene_build_192_boxes_300_connectors", |b| {
        let es = edges(rects.len(), 300);
        let ports = pipeline::assign_ports(&rects, &es);
        let all: Vec<Port> = ports.iter().flat_map(|&(a, b)| [a, b]).collect();
        b.iter(|| Scene::build(rects.iter().copied(), all.iter().copied()).unwrap());
    });
    group.bench_function("single_route_ovg", |b| {
        let c0 = connectors[0];
        b.iter(|| {
            route(
                &scene,
                c0.source,
                c0.target,
                Objective::default(),
                Search::Ovg,
            )
            .unwrap()
        });
    });
    group.bench_function("single_route_one_bend", |b| {
        let c0 = connectors[0];
        b.iter(|| {
            route(
                &scene,
                c0.source,
                c0.target,
                Objective::default(),
                Search::OneBend,
            )
            .unwrap()
        });
    });
    group.bench_function("route_all_300", |b| {
        b.iter(|| route_all(&scene, &connectors, &global::Config::default()).unwrap());
    });
    let routing = route_all(&scene, &connectors, &global::Config::default()).unwrap();
    eprintln!(
        "route_all: {} iterations, overuse {}",
        routing.iterations, routing.overuse
    );
    group.bench_function("nudge_300", |b| {
        b.iter(|| {
            nudge::nudge(
                scene.graph(),
                &rects,
                &routing.routes,
                &nudge::Config::default(),
            )
        });
    });
    group.bench_function("pipeline_300", |b| {
        let es = edges(rects.len(), 300);
        b.iter(|| pipeline::draw(&rects, &es, &pipeline::Config::default()).unwrap());
    });
    group.finish();

    let mut group = c.benchmark_group("any_angle");
    group.sample_size(10);
    let centres: Vec<(Point, Point)> = edges(rects.len(), 300)
        .into_iter()
        .map(|(a, b)| {
            (
                Point::new(rects[a].min.x - 1, rects[a].min.y - 1),
                Point::new(rects[b].max.x + 1, rects[b].max.y + 1),
            )
        })
        .collect();
    let endpoints: Vec<Point> = centres.iter().flat_map(|&(a, b)| [a, b]).collect();
    group.bench_function("sleeve_build_and_route_300", |b| {
        b.iter(|| {
            let router = SleeveRouter::new(&rects, &endpoints, 10).unwrap();
            black_box(router.route_all(&centres))
        });
    });
    group.bench_function("polyline_build_and_route_100", |b| {
        b.iter(|| {
            let mut router = PolylineRouter::new(polyline::Config::default());
            for r in &rects {
                router.add_obstacle(*r).unwrap();
            }

            for &(a, b) in centres.iter().take(100) {
                black_box(router.add_connector(a, b).unwrap());
            }
        });
    });
    group.finish();
}

/// Runs the benchmarks with Criterion's command-line configuration.
fn main() {
    let mut criterion = Criterion::default().configure_from_args();
    benches(&mut criterion);
    criterion.final_summary();
}
