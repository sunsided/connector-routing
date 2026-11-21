//! Orthogonal hyperedge routing (Wybrow, Marriott and Stuckey 2012): one net connecting
//! more than two ports as a rectilinear Steiner tree of minimal length and bends.
//!
//! Both heuristics of the paper run on the *separated* orthogonal visibility graph:
//! every vertex is split into a horizontal and a vertical copy, joined by an edge that
//! costs one bend, so length and bends are treated uniformly.
//!
//! - [`Heuristic::Sequential`] (Heuristic 1, after Long et al.): the shortest path
//!   terminal forest grows a cost tree around every terminal at once; edges between
//!   trees are bridges; Kruskal over the bridges (cost: bridge plus both tree paths)
//!   picks the minimum terminal spanning tree.
//! - [`Heuristic::Interleaved`] (Heuristic 2, Fig. 6): the forest and the spanning tree
//!   grow together; whenever the cheapest bridge costs at most twice the forest's
//!   frontier, the two sub-routes it joins merge, and the forest regrows from the merged
//!   sub-route at cost zero, so shared segments are discounted.
//!
//! Afterwards non-terminal leaves are pruned and the paper's local improvement runs:
//! unbalanced segments shift towards the side with more diverging branches (see
//! `improve`), which never makes the tree worse.

use alloc::collections::{BTreeMap, BTreeSet, BinaryHeap};
use alloc::vec::Vec;
use core::cmp::Reverse;

use crate::error::Error;
use crate::geom::{Axis, DirSet, Direction};
use crate::graph::{RoutingGraph, VertexId};
use crate::nets::Net;
use crate::scene::{PortId, Scene};
use crate::search::{Objective, Weights};

mod improve;

/// Which construction to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Heuristic {
    /// Heuristic 1: shortest path terminal forest, then a minimum spanning tree over
    /// the bridges.
    Sequential,
    /// Heuristic 2: forest and spanning tree interleaved, discounting shared segments.
    #[default]
    Interleaved,
}

/// A routed hyperedge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Tree {
    /// The graph edges of the tree, each once as `(a, b)` with `b` right of or above
    /// `a`, sorted.
    pub edges: Vec<(VertexId, VertexId)>,
    /// The total length.
    pub length: u64,
    /// The number of bends (vertices where the tree turns from one axis to the other).
    pub bends: u32,
    /// The tree as a [`Net`] for [`nudge::nudge_nets`](crate::nudge::nudge_nets).
    pub net: Net,
}

/// A node of the separated graph: vertex slot times two plus the axis (0 horizontal).
type Node = u32;

/// The node of vertex `v` on `axis`.
fn node(v: VertexId, axis: Axis) -> Node {
    v.0 * 2 + u32::from(axis == Axis::Y)
}

/// The vertex of node `n`.
fn vertex(n: Node) -> VertexId {
    VertexId(n / 2)
}

/// The separated graph of one hyperedge.
struct Separated<'a> {
    /// The routing graph.
    graph: &'a RoutingGraph,
    /// The weights.
    weights: Weights,
    /// The allowed directions at each terminal vertex.
    terminals: BTreeMap<VertexId, DirSet>,
}

impl Separated<'_> {
    /// Calls `emit(next, cost)` for every edge of node `n`.
    fn edges(&self, n: Node, mut emit: impl FnMut(Node, u64)) {
        let v = vertex(n);
        let axis = if n % 2 == 0 { Axis::X } else { Axis::Y };
        let allowed = self.terminals.get(&v).copied().unwrap_or(DirSet::ALL);
        let dirs = match axis {
            Axis::X => [Direction::Left, Direction::Right],
            Axis::Y => [Direction::Down, Direction::Up],
        };
        for d in dirs {
            if !allowed.contains(d) {
                continue;
            }

            if let Some(w) = self.graph.neighbor(v, d) {
                let allowed_back = self.terminals.get(&w).copied().unwrap_or(DirSet::ALL);
                if allowed_back.contains(d.reverse()) {
                    let length = self.graph.point(v).manhattan(self.graph.point(w));
                    emit(node(w, axis), self.weights.length.saturating_mul(length));
                }
            }
        }

        emit(n ^ 1, self.weights.bend);
    }

    /// The seed nodes of a terminal: its copies on the axes of its allowed directions.
    fn seeds(&self, v: VertexId) -> Vec<Node> {
        let allowed = self.terminals[&v];
        let mut out = Vec::new();
        if allowed.iter().any(|d| d.axis() == Axis::X) {
            out.push(node(v, Axis::X));
        }

        if allowed.iter().any(|d| d.axis() == Axis::Y) {
            out.push(node(v, Axis::Y));
        }

        out
    }
}

/// Routes a hyperedge between `ports` of `scene`.
///
/// # Errors
///
/// [`Error::TooFewPorts`] for fewer than two distinct ports, [`Error::UnknownPort`] or
/// [`Error::PortBlocked`] for a bad port, [`Error::CostOverflow`] for the objective,
/// and [`Error::NoRoute`] if the ports are not all connected.
///
/// # Examples
///
/// ```
/// use connector_routing::{DirSet, Objective, Point, Port, Rect, Scene, hyperedge};
///
/// let mut scene = Scene::new();
/// scene.add_obstacle(Rect::new(0, 0, 10, 10))?;
/// let ports = [(10, 5), (30, 20), (30, -10)]
///     .map(|(x, y)| scene.add_port(Port::free(Point::new(x, y))).unwrap());
/// let tree = hyperedge::route(&scene, &ports, Objective::default(), hyperedge::Heuristic::Interleaved)?;
/// // A trunk to x = 30 and one vertical branch through both other ports.
/// assert_eq!(tree.length, 20 + 30);
/// # Ok::<(), connector_routing::Error>(())
/// ```
pub fn route(
    scene: &Scene,
    ports: &[PortId],
    objective: Objective,
    heuristic: Heuristic,
) -> Result<Tree, Error> {
    route_with(scene, ports, objective, heuristic, true)
}

/// [`route`], with the local improvement (segment shifting) switched on or off.
///
/// # Errors
///
/// As [`route`].
pub fn route_with(
    scene: &Scene,
    ports: &[PortId],
    objective: Objective,
    heuristic: Heuristic,
    improve: bool,
) -> Result<Tree, Error> {
    let graph = scene.graph();
    let weights = objective.weights(graph)?;
    let mut terminals: BTreeMap<VertexId, DirSet> = BTreeMap::new();
    for &id in ports {
        let port = scene.port(id).ok_or(Error::UnknownPort)?;
        let v = scene
            .port_vertex(id)
            .ok_or(Error::PortBlocked(port.point))?;
        *terminals.entry(v).or_default() |= port.directions;
    }

    if terminals.len() < 2 {
        return Err(Error::TooFewPorts);
    }

    let separated = Separated {
        graph,
        weights,
        terminals,
    };
    let edges = match heuristic {
        Heuristic::Sequential => sequential(&separated),
        Heuristic::Interleaved => interleaved(&separated),
    }
    .ok_or(Error::NoRoute { connector: 0 })?;

    Ok(finish(
        graph,
        &separated,
        &edges,
        improve.then_some(weights),
    ))
}

/// Improves any tree over the graph of `scene` (for example one drawn by a user, as
/// in the paper's semi-automatic routing) by unbalanced segment shifts. `terminals` are
/// the ports the tree must keep. The result is never more expensive under `objective`.
///
/// # Errors
///
/// [`Error::UnknownPort`] or [`Error::PortBlocked`] for a bad terminal,
/// [`Error::CostOverflow`] for the objective.
pub fn improve(
    scene: &Scene,
    tree: &Tree,
    terminals: &[PortId],
    objective: Objective,
) -> Result<Tree, Error> {
    let graph = scene.graph();
    let weights = objective.weights(graph)?;
    let mut ends: BTreeMap<VertexId, DirSet> = BTreeMap::new();
    for &id in terminals {
        let port = scene.port(id).ok_or(Error::UnknownPort)?;
        let v = scene
            .port_vertex(id)
            .ok_or(Error::PortBlocked(port.point))?;
        *ends.entry(v).or_default() |= port.directions;
    }

    let mut edges: improve::Edges = tree.edges.iter().copied().collect();
    let keep: Vec<VertexId> = ends.keys().copied().collect();
    improve::improve(graph, &mut edges, &keep, weights, 1000);

    Ok(from_edges(graph, edges, keep))
}

/// The [`Tree`] of a graph edge set with the given terminals.
fn from_edges(
    graph: &RoutingGraph,
    edges: BTreeSet<(VertexId, VertexId)>,
    terminals: Vec<VertexId>,
) -> Tree {
    let mut net = Net::default();
    for &(a, b) in &edges {
        net.add_path(graph, &[a, b]);
    }

    net.terminals = terminals;
    let length = edges
        .iter()
        .map(|&(a, b)| graph.point(a).manhattan(graph.point(b)))
        .sum();
    let bends = net
        .arms
        .values()
        .filter(|arms| {
            let horizontal = arms.contains(Direction::Left) || arms.contains(Direction::Right);
            let vertical = arms.contains(Direction::Up) || arms.contains(Direction::Down);
            horizontal && vertical
        })
        .count();

    Tree {
        edges: edges.into_iter().collect(),
        length,
        bends: u32::try_from(bends).unwrap_or(u32::MAX),
        net,
    }
}

/// Builds the tree from the separated edges it uses, pruning non-terminal leaves.
fn finish(
    graph: &RoutingGraph,
    separated: &Separated<'_>,
    edges: &BTreeSet<(Node, Node)>,
    improve: Option<Weights>,
) -> Tree {
    let mut graph_edges: BTreeSet<(VertexId, VertexId)> = BTreeSet::new();
    for &(a, b) in edges {
        let (va, vb) = (vertex(a), vertex(b));
        if va != vb {
            let (lo, hi) = if graph.point(va) < graph.point(vb) {
                (va, vb)
            } else {
                (vb, va)
            };
            graph_edges.insert((lo, hi));
        }
    }

    prune_leaves(&mut graph_edges, &separated.terminals);
    if let Some(weights) = improve {
        let terminals: Vec<VertexId> = separated.terminals.keys().copied().collect();
        improve::improve(graph, &mut graph_edges, &terminals, weights, 1000);
    }

    from_edges(
        graph,
        graph_edges,
        separated.terminals.keys().copied().collect(),
    )
}

/// Removes leaves that are not terminals until none is left.
fn prune_leaves(
    edges: &mut BTreeSet<(VertexId, VertexId)>,
    terminals: &BTreeMap<VertexId, DirSet>,
) {
    loop {
        let mut degree: BTreeMap<VertexId, usize> = BTreeMap::new();
        for &(a, b) in edges.iter() {
            *degree.entry(a).or_default() += 1;
            *degree.entry(b).or_default() += 1;
        }

        let before = edges.len();
        edges.retain(|&(a, b)| {
            let leaf = |v: VertexId| degree[&v] == 1 && !terminals.contains_key(&v);
            !(leaf(a) || leaf(b))
        });
        if edges.len() == before {
            return;
        }
    }
}

/// One sub-route of the interleaved heuristic: its separated edges, nodes and terminals.
#[derive(Debug, Clone, Default)]
struct SubRoute {
    /// The separated edges.
    edges: BTreeSet<(Node, Node)>,
    /// The nodes on it.
    nodes: BTreeSet<Node>,
    /// The terminal vertices it connects.
    terminals: BTreeSet<VertexId>,
}

/// The forest state of a growing search: per node the cost, parent and sub-route.
struct Forest {
    /// The cost of each reached node, `u64::MAX` if unreached.
    cost: Vec<u64>,
    /// The parent of each reached node in its tree, `u32::MAX` for seeds.
    parent: Vec<Node>,
    /// The sub-route each reached node grows from.
    owner: Vec<usize>,
}

impl Forest {
    /// A forest over `nodes` nodes with nothing reached.
    fn new(nodes: usize) -> Self {
        Self {
            cost: alloc::vec![u64::MAX; nodes],
            parent: alloc::vec![u32::MAX; nodes],
            owner: alloc::vec![usize::MAX; nodes],
        }
    }

    /// Whether node `n` is reached.
    fn reached(&self, n: Node) -> bool {
        self.cost[n as usize] != u64::MAX
    }

    /// The separated edges from `n` back to its seed.
    fn path_edges(
        &self,
        mut n: Node,
        out: &mut BTreeSet<(Node, Node)>,
        nodes: &mut BTreeSet<Node>,
    ) {
        nodes.insert(n);
        while self.parent[n as usize] != u32::MAX {
            let p = self.parent[n as usize];
            out.insert((n.min(p), n.max(p)));
            nodes.insert(p);
            n = p;
        }
    }
}

/// Heuristic 2: the interleaved shortest path terminal forest and spanning tree.
fn interleaved(separated: &Separated<'_>) -> Option<BTreeSet<(Node, Node)>> {
    let total = separated.terminals.len();
    let mut forest = Forest::new(2 * separated.graph.capacity());
    let mut routes: Vec<SubRoute> = Vec::new();
    let mut frontier: BinaryHeap<Reverse<(u64, Node, Node, usize)>> = BinaryHeap::new();
    let mut bridges: BinaryHeap<Reverse<(u64, Node, Node)>> = BinaryHeap::new();
    for &t in separated.terminals.keys() {
        let id = routes.len();
        let seeds = separated.seeds(t);
        routes.push(SubRoute {
            edges: BTreeSet::new(),
            nodes: seeds.iter().copied().collect(),
            terminals: core::iter::once(t).collect(),
        });
        for n in seeds {
            frontier.push(Reverse((0, n, u32::MAX, id)));
        }
    }

    let mut alive: Vec<bool> = alloc::vec![true; routes.len()];
    loop {
        let grow = match (frontier.peek(), bridges.peek()) {
            (Some(Reverse(f)), Some(Reverse(b))) => f.0.saturating_mul(2) < b.0,
            (Some(_), None) => true,
            (None, Some(_)) => false,
            (None, None) => return None,
        };

        if grow {
            let Reverse((cost, n, parent, owner)) = frontier.pop().expect("peeked");
            if forest.reached(n) || !alive[owner] {
                continue;
            }

            forest.cost[n as usize] = cost;
            forest.parent[n as usize] = parent;
            forest.owner[n as usize] = owner;
            separated.edges(n, |next, w| {
                if forest.reached(next) {
                    if forest.owner[next as usize] != owner {
                        let c = cost
                            .saturating_add(forest.cost[next as usize])
                            .saturating_add(w);
                        bridges.push(Reverse((c, n, next)));
                    }
                } else {
                    frontier.push(Reverse((cost.saturating_add(w), next, n, owner)));
                }
            });
            continue;
        }

        let Reverse((c, a, b)) = bridges.pop().expect("peeked");
        let (oa, ob) = (forest.owner[a as usize], forest.owner[b as usize]);
        // Skip bridges made stale by a merge or a regrowth.
        let fresh = forest.reached(a)
            && forest.reached(b)
            && oa != ob
            && alive[oa]
            && alive[ob]
            && c == forest.cost[a as usize]
                .saturating_add(forest.cost[b as usize])
                .saturating_add(bridge_weight(separated, a, b));
        if !fresh {
            continue;
        }

        let merged = merge(&forest, &routes[oa], &routes[ob], a, b);
        if merged.terminals.len() == total {
            return Some(merged.edges);
        }

        alive[oa] = false;
        alive[ob] = false;
        let id = routes.len();
        // Regrow from the merged sub-route: forget everything its parts reached.
        for n in 0..forest.cost.len() {
            let owner = forest.owner[n];
            if owner == oa || owner == ob {
                forest.cost[n] = u64::MAX;
                forest.owner[n] = usize::MAX;
                forest.parent[n] = u32::MAX;
            }
        }

        for &n in &merged.nodes {
            frontier.push(Reverse((0, n, u32::MAX, id)));
        }

        routes.push(merged);
        alive.push(true);
    }
}

/// The weight of the separated edge between `a` and `b`.
fn bridge_weight(separated: &Separated<'_>, a: Node, b: Node) -> u64 {
    let mut weight = u64::MAX;
    separated.edges(a, |next, w| {
        if next == b {
            weight = weight.min(w);
        }
    });

    weight
}

/// The sub-route joining `ra` (which reached `a`) and `rb` (which reached `b`) through
/// the bridge `a b`.
fn merge(forest: &Forest, ra: &SubRoute, rb: &SubRoute, a: Node, b: Node) -> SubRoute {
    let mut merged = SubRoute {
        edges: ra.edges.union(&rb.edges).copied().collect(),
        nodes: ra.nodes.union(&rb.nodes).copied().collect(),
        terminals: ra.terminals.union(&rb.terminals).copied().collect(),
    };
    forest.path_edges(a, &mut merged.edges, &mut merged.nodes);
    forest.path_edges(b, &mut merged.edges, &mut merged.nodes);
    merged.edges.insert((a.min(b), a.max(b)));

    merged
}

/// Heuristic 1: the shortest path terminal forest, then Kruskal over its bridges.
fn sequential(separated: &Separated<'_>) -> Option<BTreeSet<(Node, Node)>> {
    let mut forest = Forest::new(2 * separated.graph.capacity());
    let terminals: Vec<VertexId> = separated.terminals.keys().copied().collect();
    let mut frontier: BinaryHeap<Reverse<(u64, Node, Node, usize)>> = BinaryHeap::new();
    for (id, &t) in terminals.iter().enumerate() {
        for n in separated.seeds(t) {
            frontier.push(Reverse((0, n, u32::MAX, id)));
        }
    }

    while let Some(Reverse((cost, n, parent, owner))) = frontier.pop() {
        if forest.reached(n) {
            continue;
        }

        forest.cost[n as usize] = cost;
        forest.parent[n as usize] = parent;
        forest.owner[n as usize] = owner;
        separated.edges(n, |next, w| {
            if !forest.reached(next) {
                frontier.push(Reverse((cost.saturating_add(w), next, n, owner)));
            }
        });
    }

    // Bridges: separated edges between different trees, cost plus both tree paths.
    let mut bridges: Vec<(u64, Node, Node)> = Vec::new();
    for n in 0..forest.cost.len() {
        let n = u32::try_from(n).expect("few nodes");
        if !forest.reached(n) {
            continue;
        }

        separated.edges(n, |next, w| {
            if forest.reached(next)
                && forest.owner[next as usize] != forest.owner[n as usize]
                && n < next
            {
                let c = forest.cost[n as usize]
                    .saturating_add(forest.cost[next as usize])
                    .saturating_add(w);
                bridges.push((c, n, next));
            }
        });
    }

    bridges.sort_unstable();
    let mut parent: Vec<usize> = (0..terminals.len()).collect();
    let mut edges = BTreeSet::new();
    let mut nodes = BTreeSet::new();
    let mut joined = 1;
    for (_, a, b) in bridges {
        let (ra, rb) = (
            find(&mut parent, forest.owner[a as usize]),
            find(&mut parent, forest.owner[b as usize]),
        );
        if ra == rb {
            continue;
        }

        parent[ra] = rb;
        forest.path_edges(a, &mut edges, &mut nodes);
        forest.path_edges(b, &mut edges, &mut nodes);
        edges.insert((a.min(b), a.max(b)));
        joined += 1;
    }

    (joined == terminals.len()).then_some(edges)
}

/// Union-find root of `i`, with path halving.
fn find(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }

    i
}
