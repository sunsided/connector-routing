//! Draws a small diagram with [`pipeline::draw`] and prints it as SVG.
//!
//! `docs/routing.svg`, the picture in the README, is its output:
//!
//! ```sh
//! cargo run --example svg > docs/routing.svg
//! ```

use core::fmt::Write as _;

use connector_routing::{Point, Rect, pipeline};

/// Margin around the drawing, in scene units.
const MARGIN: i32 = 20;

/// Stroke colours of the routes, one per edge (Tableau 10).
const COLOURS: [&str; 10] = [
    "#4e79a7", "#f28e2b", "#e15759", "#76b7b2", "#59a14f", "#edc948", "#b07aa1", "#ff9da7",
    "#9c755f", "#bab0ac",
];

fn main() -> Result<(), connector_routing::Error> {
    let boxes = [
        Rect::new(0, 100, 80, 150),
        Rect::new(160, 0, 240, 50),
        Rect::new(160, 100, 240, 150),
        Rect::new(160, 200, 240, 250),
        Rect::new(320, 40, 400, 90),
        Rect::new(320, 160, 400, 210),
        Rect::new(480, 100, 560, 150),
    ];
    let edges = [
        (0, 1),
        (0, 2),
        (0, 3),
        (1, 4),
        (2, 4),
        (2, 5),
        (3, 5),
        (1, 6),
        (4, 6),
        (5, 6),
    ];
    let drawing = pipeline::draw(&boxes, &edges, &pipeline::Config::default())?;
    print!("{}", svg(&drawing.boxes, &drawing.routes));
    Ok(())
}

/// Renders `boxes` and `routes` as a standalone SVG document.
fn svg(boxes: &[Rect], routes: &[Vec<Point>]) -> String {
    let points = boxes
        .iter()
        .flat_map(|b| [b.min, b.max])
        .chain(routes.iter().flatten().copied());
    let (mut x0, mut y0, mut x1, mut y1) = (i32::MAX, i32::MAX, i32::MIN, i32::MIN);
    for p in points {
        (x0, y0, x1, y1) = (x0.min(p.x), y0.min(p.y), x1.max(p.x), y1.max(p.y));
    }
    let (x0, y0) = (x0 - MARGIN, y0 - MARGIN);
    let (w, h) = (x1 - x0 + MARGIN, y1 - y0 + MARGIN);

    let mut out = String::new();
    // Writing to a String cannot fail.
    let _ = writeln!(
        out,
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="{x0} {y0} {w} {h}" width="{w}" height="{h}" font-family="sans-serif" font-size="14">"#
    );
    let _ = writeln!(
        out,
        r##"<rect x="{x0}" y="{y0}" width="{w}" height="{h}" fill="#ffffff"/>"##
    );
    for (i, route) in routes.iter().enumerate() {
        let colour = COLOURS[i % COLOURS.len()];
        let path: Vec<String> = route.iter().map(|p| format!("{},{}", p.x, p.y)).collect();
        let _ = writeln!(
            out,
            r#"<polyline points="{}" fill="none" stroke="{colour}" stroke-width="2" stroke-linejoin="round"/>"#,
            path.join(" ")
        );
        for p in [route.first(), route.last()].into_iter().flatten() {
            let _ = writeln!(
                out,
                r#"<circle cx="{}" cy="{}" r="3" fill="{colour}"/>"#,
                p.x, p.y
            );
        }
    }
    for (i, b) in boxes.iter().enumerate() {
        let (bw, bh) = (b.max.x - b.min.x, b.max.y - b.min.y);
        let _ = writeln!(
            out,
            r##"<rect x="{}" y="{}" width="{bw}" height="{bh}" rx="4" fill="#f2f2f2" stroke="#444444" stroke-width="1.5"/>"##,
            b.min.x, b.min.y
        );
        let _ = writeln!(
            out,
            r##"<text x="{}" y="{}" text-anchor="middle" dominant-baseline="central" fill="#222222">{}</text>"##,
            b.min.x + bw / 2,
            b.min.y + bh / 2,
            char::from(b'A' + u8::try_from(i).unwrap_or(0))
        );
    }
    out.push_str("</svg>\n");
    out
}
