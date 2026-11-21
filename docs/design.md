# connector-routing design

Global connector routing: route many connectors at once around rectangular obstacles
(the nodes of a diagram, the parts of a schematic, the cabinets of a cable plan),
minimising length, bends, crossings and overlaps. The crate covers the whole chain from
the routing graph to final, separated geometry, in three route styles:

1. **Orthogonal** (the main style): the orthogonal visibility graph (OVG) of Wybrow,
   Marriott and Stuckey (2009), single-connector A* over `(vertex, direction)`, the
   1-bend visibility graph of Marriott, Stuckey and Wybrow (2014), global routing with
   PathFinder negotiated congestion, path ordering, crossing reduction and nudging.
2. **Sleeve** (smooth, any-angle): a constrained Delaunay triangulation of the free
   space, Dijkstra over its dual graph and the funnel algorithm (arXiv 2605.17498).
3. **Poly-line** (any-angle, shortest): the visibility graph over obstacle corners with
   incremental updates (Wybrow, Marriott and Stuckey 2005).

Priorities, in this order: correctness, performance, developer experience.

## Geometry model

- **Exact integer geometry.** Points are `i32` pairs. Every predicate (orientation,
  in-circle, segment against rectangle, visibility) is evaluated exactly in `i64` or
  `i128`. No floating point is used anywhere, so there are no robustness bugs and every
  result is reproducible bit for bit.
- **Coordinate range.** Input coordinates must lie in `[-2^29, 2^29]`
  ([`MAX_COORD`]). This keeps the in-circle determinant of the CDT inside `i128` (its
  terms are products of four coordinate differences of at most `2^30` each, times a
  small constant) and leaves headroom for nudging margins. The API rejects larger
  coordinates with an error instead of overflowing.
- **Rectangles** are closed axis-aligned boxes with `min.x < max.x` and
  `min.y < max.y`. Routes may touch an obstacle's boundary but must never enter its
  **open interior**. Users who want clearance inflate their rectangles (a `buffer`
  helper is provided); this is libavoid's "shape buffer distance".
- **Overlapping obstacles** are allowed (containers, labels on nodes). An interesting
  point that lies in the open interior of any obstacle is ignored; a port in such a
  position is unreachable and routing it fails with an error.
- **Axes.** `y` grows "up" in the documentation (`Direction::Up` is `+y`). Nothing in
  the algorithms depends on handedness except the names of directions and sides, so
  y-down screen coordinates work unchanged; `Up` then points down the screen.
- **Ports.** A port is a point plus the set of directions in which a connector may leave
  it (`DirSet`). A connector that *ends* at a port must arrive moving opposite to one of
  those directions, so a port on the top side of a box allows `Up` and connectors arrive
  moving `Down`. Ports are interesting points of the OVG.
- **Lengths and costs** are `u64`. A route's cost is
  `length_weight * length + bend_penalty * bends` (plus congestion and crossing terms in
  global routing). Products are checked; an overflow is an error.
- **Euclidean styles.** Sleeve and poly-line routes have irrational lengths. Their
  predicates stay exact; their path *costs* are fixed-point Euclidean lengths computed
  with integer square roots (`u64::isqrt` of squared lengths scaled by `2^16`). The
  geometry of the result is exact (all route vertices are input points), only the
  comparison of nearly equal path lengths may pick the longer one by a rounding margin.

## Routing graph

`RoutingGraph` is a partial grid: vertices are points, and each vertex has at most four
neighbours, one per direction, joined by obstacle-free axis-parallel edges. The OVG is
the routing graph of every orthogonal router here; the searches, congestion, ordering
and nudging only rely on the partial-grid structure, so another builder (such as the
channel graph of arXiv 2309.01671, not built) could plug in. Vertex ids are slots that
stay stable while the graph is updated; freed slots are reused.

### Orthogonal visibility graph (Wybrow et al. 2009, section 3)

Interesting points `I` are the corners of every obstacle and every port. From each
interesting point not inside an obstacle, the *interesting horizontal segment* is the
maximal obstacle-free horizontal segment through it (extended in both directions until
it would enter an obstacle's open interior, or without bound); vertical segments are
defined the same way. The OVG vertices are all intersections of an interesting
horizontal with an interesting vertical segment, and edges join consecutive vertices
along each segment.

This is the paper's definition: the paper takes segments between pairs of mutually
visible interesting points and nodes in `X_I x Y_I` visible from an interesting point on
both axes, which is the same set of nodes and the same nearest-neighbour edges (checked
against the original, and tested against a direct implementation of the definition).

Two additions, both found by the brute-force comparison below, make the graph exact
when port directions constrain the route (the paper's "shrink each segment onto the
graph" argument fails when the first or last segment would shrink to nothing):

- **Routing frame.** The corners of the bounding box of all interesting points, grown
  by one unit, are interesting too. A port on the outer side of the outermost obstacle,
  pointing outwards, needs a line beyond every other interesting point.
- **Port stubs.** For each allowed direction `d` of a port at `p`, the points `p + d`
  and `p + d ± e` (`e` perpendicular to `d`) are interesting. A route forced to turn
  right after leaving a port, or to hook back into it, needs lines one unit ahead of and
  beside the port.

Two maximal segments on the same line are either equal or disjoint (they are separated
by an obstacle of positive width), so a segment is keyed by its line and start and
remembers the interesting points that generate it.

Construction computes each segment by a scan over the obstacles (`O(|I| n)`, a
documented shortcut against the `O(n log n)` sweep of the paper), then intersects the
segments in `O(H log V + k)` with a sorted index of vertical segments.

### Incremental OVG

`Scene` keeps obstacles (stable `ObstacleId`s), ports (`PortId`s), the segments with
their generators, and the routing graph, and updates them in place:

- Adding, removing or moving obstacle `R` affects exactly the segments whose line
  passes strictly through `R`'s `y` (or `x`) range and whose closed extent meets `R`'s
  closed extent (they are blocked by `R`, or stopped by it), plus the interesting points
  in `R`'s closure. All other segments keep their extent, since blocking only depends on
  obstacles that meet the segment. The affected segments are removed together with
  their vertices, and the segments of the affected generators are recomputed and
  inserted.
- Adding or removing a port adds or removes one generator.
- Vertex insertion and removal relink the four-neighbour lists in `O(log)`.

The test oracle compares the point and edge sets (not ids) of the incremental graph
with a graph rebuilt from scratch after random add, move and remove sequences.

The EditLens paper (arXiv 1612.05064) instead keeps an OVG whose edges are L-shaped
connections between pairs of connector points, which is quadratic in the number of
points; its incremental rules (delete the edges of a removed node, block the edges a new
node covers) are what our segment-level update does on the smaller Wybrow graph. The
paper's random-offset "local nudging" is not implemented; the exact nudging below
replaces it.

`incremental::invalidated_by` and `incremental::may_improve` report, from the committed
routes, which connectors a change invalidates (a route through a new obstacle) or may
improve (a removed obstacle that a cheaper path could cross, with the lower-bound
pruning of Wybrow et al. 2005, adapted to the Manhattan metric). A layout engine can
then reroute only those (Dwyer et al. 2006). The scene itself does not track routes.

## Single connector

### A* over the OVG (Wybrow et al. 2009, section 4)

The state is `(vertex, heading)`, where the heading is the direction of the last move,
or "not moved yet" at the source. Moves never reverse. A move costs
`length_weight * length`, plus `bend_penalty` when it changes the heading. The source
expands only along its port's directions; the goal is the target vertex reached with a
heading opposite to one of the target port's directions.

The search runs on `heuristic_search::a_star`. The heuristic is `length_weight * manhattan + bend_penalty * b`, where
`b` is the exact minimum number of bends from `(p, heading)` to the target with a valid
arrival heading in the obstacle-free plane. This is a shortest-path distance in a
relaxation, hence consistent. It differs from the paper's `s_d` table (checked against
the original) in these cases:

| case (heading `D`, arrival `A`) | paper `s_d` | ours |
|---|---|---|
| `D` perpendicular to `A`, target ahead in `D` but not on the side `A` points to | 1 | 3 |
| `D = reverse(A)`, target on the line ahead | 2 | 4 |
| `D = reverse(A)`, at the target | 2 | 4 |
| `D = A`, at the target | 4 | 0 |
| `D` perpendicular to `A`, target exactly beside the current vertex on the side `A` points to | 3 | 1 |

A unit test enumerates the free-space value by breadth-first search on a grid and
compares it with the closed form. As we read the paper's table, its case 3 gives 3 for a
target exactly on the perpendicular line through the current vertex (`D0` not in
`dirns`), where one bend in place suffices; our value is 1 there.

Deviations from the paper: the paper prunes left/right neighbours whose visibility line
"neither passes right by an obstacle nor contains the target"; we keep every neighbour
(exactness first, and the 1-bend graph below removes the redundancy properly). The
paper breaks ties by a time stamp, preferring the newest entry and the order straight,
right, left; `heuristic_search` breaks ties by larger `g`, then first in, first out, and
we generate straight, then right, then left. Both are deterministic.

The cost model `f(length, bends)` of the paper is any monotone function; we support the
weighted sum and, as exact lexicographic objectives, "shortest, then fewest bends" and
"fewest bends, then shortest" (weights derived from bounds on the other term).

### 1-bend visibility graph (Marriott et al. 2014)

Nodes are `(interesting point, direction)` pairs: obstacle corners with a direction that
runs along one of the corner's sides, and ports. An edge joins two nodes when an
orthogonal path with one bend (or none) connects them, leaving the first node in its
direction and hitting the second node's corner from outside. By the paper's Theorems 1
and 2 every optimal route has a topologically equivalent obstacle-hugging route of the
same length and bends, whose intermediate segments each pass a supporting corner, so
searching over 1-bend edges between supports is exact for the length-and-bends cost.

We generate the 1-bend successors lazily by walking the OVG rays from a node, so the
quadratic edge set is never stored. Two details the paper leaves implicit are needed even
for simple cases: the second leg of an edge may have zero length (the support vertex can
be the bend point itself, as in a U-turn around a box), and a node reached that way must
not bend again in place. Nodes next to the ports cover turns that port directions force
without a supporting obstacle.

**Result of the test (not the default).** With free port directions the 1-bend search
matched brute force on every lattice instance tested. With ports on obstacle sides it
does not: the regression `one_bend_misses_slit_between_touching_boxes` has an optimum of
length 20 with 6 bends that leaves the port, turns back into a slit between touching
boxes and squeezes through it, turning where no corner supports the segment; the 1-bend
search returns length 38 with 4 bends. Its routes are always valid and never cheaper than
the optimum, and it falls back to the OVG search when it finds nothing. It is also not
faster in this lazy form (about 70 times slower than the OVG A* in the benchmark: each
expansion walks whole rays and their perpendiculars). So `Search::Ovg` is the default and
`Search::OneBend` is opt-in. Theorem 1 covers pure length-and-bends only in any case;
PathFinder searches the OVG.

### Brute-force oracle

Tests search the full unit grid of the bounding box (plus padding, to confirm that
optimal routes stay within one unit of it) with the same `(point, heading)` semantics,
and compare the optimal cost. Exactness is asserted on inputs on a lattice of spacing 4
(the unit grid is finer, so routes may use lines the graph does not have): ports with
free directions anywhere, and ports on obstacle sides pointing outwards, with touching
and overlapping obstacles. With arbitrary direction sets on the unit lattice, a port stub
can coincide with an obstacle side and a route squeezing through a one-unit gap needs
lines between the graph's; there the test asserts validity and that the route is never
cheaper than brute force.

## Global routing

### Resources and capacities

Connectors may share OVG edges: nudging separates them later. What limits sharing is
the width of the channel. The resource model:

- The `x` axis is split into *elementary slabs* between consecutive distinct `x`
  coordinates of vertical OVG lines (and symmetrically for `y`).
- In each slab, the obstacle-free `y` intervals are its *gaps*. A horizontal OVG edge
  crosses one gap in each slab it spans; that `(slab, gap)` pair is a resource, shared
  by every horizontal edge in the same gap, whatever line it is on.
- The capacity of a gap of height `h` is `floor(h / separation) + 1` tracks (segments
  may touch obstacle boundaries), unbounded for gaps that reach infinity.

Per-cross-section capacity is necessary but not sufficient for separation `δ`:
chained segments in a staircase can need more room than any single cross-section shows.
So nudging always reports the separation it achieved, and no test assumes that zero
overuse implies full separation.

### PathFinder (McMurchie and Ebeling 1995; not checked against the original)

Each iteration routes the nets in a deterministic order (by decreasing half-perimeter
of the bounding box, ties by index) with A* where a resource `r` costs
`base_r * (1 + h_r) * (1 + p * overuse_r(+1))`, `base_r` being the slab width times the
length weight. After each iteration, `h_r += history_increment * overuse_r` and the
present factor `p` grows geometrically. The first iteration routes every net; later
iterations rip up and reroute only the nets that use an overused resource. The loop stops
when no resource is overused or after `max_iterations`, and reports the overuse.

Integer arithmetic throughout, saturating, so the run is deterministic. Factors are at
least one, so the free-space heuristic stays admissible.

A crossing penalty is charged when a route passes straight through a vertex that
another net passes straight through in the perpendicular direction (a certain
crossing). Crossings at shared corners depend on the later ordering and are counted by
the ordering stage instead.

## Post-processing

### Path ordering (Wybrow et al. 2009, section 5.1; arXiv 2309.01671)

For every OVG edge used by several nets we need their order across the edge. Each
shared edge first gets a *pseudo direction* (Wybrow et al. 2009): starting from an
unassigned shared edge, directions propagate along every net through its connected
shared stretch, head to tail, bends included; where a net would force a conflict (the
paper's split point) the first assignment stays. Two nets on an edge are compared by
walking both from the edge towards its pseudo entry while they share edges; at the first
divergence the one turning more to the left of the walking direction lies on the left
side. A walk that reaches a net's end is inconclusive and the walk towards the exit
decides; only if both are inconclusive does an end rank between a left turn and going
straight; identical walks fall back to net index. Every walk from an edge is a sequence of
ranked choices, so the comparison is a total order on each edge, and all edges of one
shared stretch walk to the same end, so they agree: a necessary crossing happens once, at
the exit (the paper's Theorem 3).

The first version used the fixed look directions of arXiv 2309.01671 (horizontal edges
look left, vertical edges down). On a shared stretch with a bend, the two legs then look
to different ends and the order flips at the corner: three crossings instead of one. The
regression test `shared_corner_crosses_once` keeps that case.

Line orders are topological orders of all pairwise orders plus hard geometric arcs:
terminal points (below), and *fixed sides* (a movable segment beside a port-pinned
segment on a line along an obstacle side must go to the free side). A pair whose two
ends both decide and disagree crosses once in either order; that order is *soft* and is
dropped first if the line's arcs contain a cycle, then the kiss arcs, and only then is a
cycle broken deterministically. Tests assert that drawn crossings never fall below the
required ones, and equal them exactly in obstacle-free scenes; with obstacles, a segment
pinned by a port against an obstacle side can force an order that costs crossings the
combinatorial count does not see. The planar-bundle test (Theorem 4) and the swap test
(Theorem 3) check the guarantees with obstacles.

Compared with the paper, the order comes from the walk comparator, not from the
`L ++ S ++ R` insertion along the pseudo direction; both put crossings at the exits.

`crossings(a, b)` counts the crossings two OVG paths must have: an isolated shared vertex
where they interleave, plus one per maximal shared stretch whose two ends demand
different sides (Dwyer et al. 2006 count poly-line crossings the same way).

### Crossing reduction (arXiv 2309.01671)

When two paths must cross more than once, the section of one between their first and
last shared vertex is replaced by the corresponding section of the other, if that keeps
the path simple and strictly lowers that net's total crossing count (so the pass
terminates). Capacities are not checked; the pipeline measures the overuse again
afterwards (`global::overuse`).

### Nudging (Wybrow et al. 2009, section 5.2; arXiv 2309.01671)

Paths become maximal segments. The horizontal pass moves vertical segments, the
vertical pass horizontal ones; a segment ending at a port is fixed. Constraint
generation follows Dwyer's scan line (used by both papers): objects (segments and
obstacle sides) are ordered by position and, on the same line, by the path order;
sweeping along the other axis, each newly opened object is constrained against its
neighbours in the open list. Segments overlap on closed intervals, obstacles on their
open interior. Constraint generation is a public step that returns the constraints as
data (Dwyer et al. 2006), so a layout engine can move nodes without changing the routing
topology.

Three kinds of constraints beyond the papers' turned out necessary in testing:

- **Shape constraints.** A segment stays on its side of the far end of each
  perpendicular segment it bends into (gap zero), so no segment reverses. Without them a
  component bounded only by the region walls drifts far out and flips its neighbours.
- **Terminal points.** A port whose route arrives perpendicular to the pass is a fixed
  point object on its line; another net's segment through it is ordered opposite to the
  route's arm there, so it cannot cross the route's last segment.
- **Kisses.** Two routes that turn away from each other at a shared vertex (`┌` meets
  `┘`) cross after nudging exactly when the first pass puts one's vertical on the
  other's horizontal side and the second pass puts the other's horizontal on the first's
  vertical side; the second pass orders such pairs from the first pass's result.

The solver is exact and needs no LP: every constraint is a difference constraint
`x_v - x_u >= w`. Per independent component of moving segments we

1. find the largest integer gap `t` (capped by `max_separation`) such that every
   segment-segment and segment-obstacle arc can have gap `t`, by binary search with a
   longest-path feasibility test (2309.01671 maximises the sum of such gap variables with
   an LP; one shared gap per component is the same objective for a single variable);
2. if `t < separation`, retry with zero gap to obstacles (they may be touched);
3. place every segment at the midpoint of its leftmost and rightmost feasible positions,
   which is feasible (the feasible set is convex) and integral after rounding down.
   Components anchored on one side only (routes around the outside of the scene) are
   packed against that side at gap `t` instead, with `t` capped at the separation.

The constraint graph is acyclic (every constraint points forward in the scan order), so
the extreme solutions are single forward and backward passes instead of Bellman-Ford,
and the binary search costs `O(E log W)` per component. `Nudged::overlaps` counts
segments still on top of each other (two port segments on one line, or a channel with no
room); `Nudged::separation` is the smallest gap achieved.

The result centres single segments in their alley and spreads bundles evenly.

**Water-filling.** One common gap per component let a single tight spot squeeze a whole
bundle. Each component now takes its largest gap; if that is below the cap, only the
bottleneck (the members with the least slack) is placed, and the rest get a new, larger
gap in the next round, with the placed segments as constants. On the 300-connector bench
this cut the overlapping segment pairs from 8,985 to 7,238 at first; the later
port-point and terminal-tier ordering changes needed by full nudging moved it to 9,615.
The rest come from genuinely overfull lines (see open questions).

**Projection (`Placement::Project`).** The alternative of Wybrow et al.: desired positions
(the middle of the alley for the middle segment of a Z or S, the current line otherwise:
the obstacle side a U bends around) are projected onto the constraints at the separation
by least squares, after the VPSC method of Dwyer, Marriott and Stuckey (blocks of tight
constraints at the mean of their desired positions; merge along the most violated
constraint; split at a negative Lagrange multiplier; repeat). Arithmetic is exact over
rationals; the result is rounded per block and repaired into the extreme feasible
positions in scan order, which keeps every constraint. Not checked against the VPSC paper
(not available here; implemented from its description in Wybrow 2009 and the thesis).
Compared on the same scenes: both pass the same property tests (obstacles, ports,
topology, separation, crossings) and keep bundles at exactly the separation under
projection instead of spreading them. On the 300-connector bench most components are
held at gap 0 by overfull lines, water-filling places them the same way in both modes,
and the totals are close (about 108,000 units of movement each; projection 0.64 s
against 0.26 s); before the ordering changes they differed more (83,412 against
296,278). Every pass preserves the order of overlapping segments, and every moved
segment sweeps an obstacle-free rectangle, so routes keep their topology (Marriott et
al. 2014's notion of topology-preserving edge moves). The property test checks exactly
that (vertical segments sweep with their original extent in the first pass, horizontal
ones with their final extent in the second), plus obstacle avoidance, fixed ports,
separation between movable segments, and that the drawn crossings equal the required
ones.

## Pipeline (arXiv 2309.01671)

`pipeline::draw` takes boxes and edges (multi-edges and self-loops allowed) and runs:

1. **Port assignment.** The port sides follow the straight segment between the box
   centres; when it leaves a side in its outer quarter, one port moves to the adjacent
   side so the edge becomes an L bending away from the barycentre of the box centres.
   Ports on a side are ordered by the circular order of their edges and spread evenly.
   Self-loops get neighbouring ports on the least populated side.
2. **Routing graph.** The OVG, or the paper's channel graph (`GraphKind::Channels`,
   `pipeline::channel_graph`): for every box the narrowest empty channel to its right
   (above) towards a box or the frame, its centre line as representative, channels that
   meet a channel whose extent across contains theirs dropped, plus a representative
   from every port outwards; vertices at crossings and ports. Overlapping
   representatives on one line are merged so the graph stays a partial grid. If an edge
   cannot be routed on it, the drawing falls back to the OVG (on random grid layouts the
   channel graph routed 97 of 100 drawings).
3. **Routing** with bend-minimal shortest paths tracking the entry direction (A*
   instead of the paper's Dijkstra; same optimum).
4. **Path ordering and crossing reduction** as above.
5. **Nudging** (the paper's constrained mode: boxes and ports fixed).

**Full nudging** (`pipeline::Config::full_nudging`, `nudge::full`): box sides become
variables too. Boxes keep at least their size and stay the separation apart (new
constraint kind between obstacle sides), routes keep the separation to boxes, and every
port is tied to its box: a port on a side across the pass axis moves with that side, a
port on a side along it slides between the box's sides, so a box grows when its ports
need room, as in the paper. Ties make the constraint graph cyclic, so the leftmost
solution comes from Bellman-Ford longest paths over tied groups (a positive cycle means
infeasible, and the drawing falls back to constrained nudging). The leftmost solution
minimises every position and so the drawing's extent and every box's growth at once;
the paper weights these in an LP instead. Testing found two interactions: segment
extents must follow moved ports, and a route running along a box side through another
route's port must cross that port's segment rather than enter the box (fixed-side arcs
now include port points and win over terminal arcs). With those, full nudging applied
to all of 100 random grid drawings; tests check that boxes never shrink, stay apart,
keep their ports, and that routes avoid the moved boxes.

## Hyperedges (Wybrow et al. 2012)

A net with more than two ports is routed as a rectilinear Steiner tree over the
"separated" OVG (each vertex split into a horizontal and a vertical copy joined by a
bend-penalty edge), with Heuristic 1 (shortest path terminal forest, Kruskal over bridge
edges, MTST; Long et al.) and Heuristic 2 (interleaved SPTF and MCST, committing a bridge
when its cost is at most twice the SPTF frontier, regrowing from the merged sub-route at
cost zero). Stale bridges after a merge are recognised by recomputing their cost. Port
directions restrict the separated edges at terminals, and non-terminal leaves are pruned.
The local improvement of section 3 shifts a straight run of the tree that holds no
terminal towards the side with more diverging branches, up to the end of the shortest of
them, where it vanishes with its bend (this also merges parallel routes, the paper's
first transformation). A shift is kept only if the result is still a tree over graph
edges and strictly cheaper, so improvement never makes a tree worse; `hyperedge::improve`
also takes trees from elsewhere (the paper's semi-automatic routing). On random scenes
the heuristics' trees rarely leave a terminal-free unbalanced run, so the gain shows
mostly on hand-drawn trees (tested on a comb). Hyperedge trees are edge sets (`Net`), so ordering and nudging handle
them like connectors; shape constraints cover branches attached in the middle of a run.
Tests check connectivity before and after nudging, obstacle avoidance, and the bound by
the minimum spanning tree of the pairwise shortest routes.

## Sleeve routing (arXiv 2605.17498)

The free space is triangulated with an exact constrained Delaunay triangulation
(incremental insertion with Lawson flips, then constraint recovery by flipping (Sloan),
all predicates in `i128`; not checked against an original, the paper cites
Domiter and Žalik). Obstacle sides are constraints, split at collinear input points.
Dijkstra runs on the dual graph (centroid distances, as in the paper, with integer square
roots), skipping every triangle inside an obstacle, and the funnel algorithm (Lee and
Preparata; not checked against an original) pulls the path taut through the sleeve. Per
the paper, routes are batched by a greedy vertex cover of the demand graph (one Dijkstra
tree per cover vertex) and a straight segment is tried first. The paper's vertex collapse
fixes detours of routes that start at node centres inside padded obstacles; our endpoints
lie on obstacle boundaries, where it does not apply. What does apply is its idea of
letting the route leave along the obstacle: an endpoint on a wall shared by two touching
obstacles touches no free triangle (the slit has no area), so the router first walks
along such walls (constrained edges between two blocked triangles) to the nearest
vertices that touch free space, seeds the search there and prepends the walk; the target
side likewise. Obstacle interiors must not
overlap (touching is fine); a failed constraint recovery is an error, not a silent gap.
Shortcut: constraint recovery scans all triangles per flip (`O(T)`), fine for hundreds of
obstacles.

## Poly-line routing (Wybrow et al. 2005)

The visibility graph over obstacle corners and ports, A* with the Euclidean lower bound,
and the invisibility graph for incremental updates: every rejected node pair remembers
one obstacle that blocks it, so removing an obstacle retests only its pairs. Object
removal reports only connectors whose lower bound through the freed region beats their
current cost (or that had no route). The bound is the larger of the straight distance
and the distances to the region, weaker than the paper's exact closed form but safe.

The paper's "hidden region" pruning is implemented: at a corner of exactly one obstacle a
route may only go straight on or turn towards the obstacle (around it); any other turn
could be cut short. It is exact for pure shortest paths only: with a segment penalty the
property test found a route that turns the other way to save a bend, so pruning is
applied only when the segment and angle penalties are zero. Search states are
`(node, previous node)`, so bends and angles can be charged. Lengths are fixed point
(`2^8` per unit), rounded up per edge against a heuristic rounded down, which keeps A*
admissible and consistent.

Costs follow the thesis' formula
`length + alpha * bends + beta * a * ln(a + 1) / 10 + gamma * crossings`, with `a` the
turning angle scaled to `0..=10`. The angle term is evaluated without floating point: the
cosine of the turn (integer square root of the scaled squared norms) selects one of 101
angle classes, and a precomputed fixed-point table gives `a ln(a + 1) / 10`. Straight
continuations through a node are not bends (the thesis' `s = 0` for `a = 0`).

## Multi-objective (feature `multi-objective`)

For one connector, with the other routes fixed, the Pareto front over
`[length, bends, crossings]` via NAMOA*dr from `multiobjective-search`, over the same `(vertex, heading)` state graph.

## Papers

Status: **checked** (rules compared with the original text in `papers-routing/`),
**alternative** (a deliberate different method, named), **not checked** (the original was
not available; implemented from secondary descriptions).

| Paper | Used for | Status |
|---|---|---|
| Wybrow, Marriott, Stuckey, *Orthogonal Connector Routing*, GD 2009 | OVG definition and construction | checked; scan instead of sweep (shortcut) |
| same | OVG definition, exactness under port directions | checked; frame and port stubs added (regression tests) |
| same | cost `f(length, bends)`, `(vertex, direction)` A* | checked; exact bend estimate (paper's case 3 overestimates as read), no neighbour pruning, other tie-break |
| same | ordering of shared edges: pseudo directions, Theorems 3 and 4 | checked; pseudo directions as in the paper, order by walk comparator instead of `L ++ S ++ R`; theorems tested |
| same | nudging: separation constraints by scan line | checked |
| same | nudging: VPSC projection to desired positions | checked against the paper's description; both placements built, spreading default |
| Marriott, Stuckey, Wybrow, *Seeing Around Corners*, Diagrams 2014 | obstacle-hugging routes, 1-bend visibility graph | checked; lazy edges; not exact in tests, so opt-in, not default |
| Wybrow, Marriott, Stuckey, *Orthogonal Hyperedge Routing*, Diagrams 2012 | hyperedge Heuristics 1 and 2 (Fig. 6), local improvement | checked; segment shifting subsumes junction moving |
| Wybrow, Marriott, Stuckey, *Incremental Connector Routing*, GD 2005 | poly-line visibility graph, invisibility graph, removal pruning, hidden regions | checked; hidden-region pruning only without bend costs |
| Dwyer, Marriott, Wybrow, *Integrating Edge Routing into Force-Directed Layout*, GD 2006 | exported topology constraints, crossing counting on shared paths | checked (routing parts only) |
| Wybrow, PhD thesis, 2008 | libavoid background, poly-line cost formula with angle penalty | checked (cost formula; angle term by table) |
| Hegemann, Wolff, *A Simple Pipeline for Orthogonal Graph Drawing*, arXiv 2309.01671 | port assignment, channel graph, crossing reduction, constrained and full nudging, fixed look directions | checked; LP replaced by exact max-gap (constrained) and Bellman-Ford packing (full); fixed look directions replaced (regression test) |
| Gladisch et al., *Orthogonal Edge Routing for the EditLens*, arXiv 1612.05064 | incremental OVG update rules | checked; applied to the Wybrow OVG |
| Nachmanson, Chen, *Browsing Large Graphs with Tile Pyramids and Sleeve Routing in the Browser*, arXiv 2605.17498 | sleeve routing, vertex cover batching, straight-line first | checked; vertex collapse replaced by wall exits for endpoints in slits |
| McMurchie, Ebeling, *PathFinder*, FPGA 1995 | negotiated congestion | not checked |
| Pupyrev et al., metro-line crossing minimisation | path order comparator | not checked (via 2309.01671) |
| Dwyer, Marriott, Stuckey, VPSC / *Fast node overlap removal*, GD 2005 | scan-line constraint generation, VPSC satisfy and refine | not checked (via Wybrow 2009 and the thesis) |
| Lee and Preparata; Chazelle (funnel algorithm) | funnel | not checked |
| Lawson; Sloan (Delaunay flips, constraint recovery) | CDT | not checked |
| Long, Zhou, Memik, EBOARST (SPTF/MTST) | hyperedge Heuristic 1 | not checked (via Wybrow 2012) |

## Open questions (with the defaults taken)

- **1-bend search as default.** Not exact in tests (a slit between touching boxes), and
  slower than the OVG search in the lazy form here. Default: OVG search; 1-bend opt-in.
- **Port stubs and frame.** Additions to the paper's graph, needed for exactness with
  port directions. Default: on; they add up to nine interesting points per port.
- **Nudging placement.** Spreading (max gap plus midpoint, 2309.01671) is the default;
  VPSC projection (Wybrow 2009) is `Placement::Project`. Default: separation 4, spread at
  most 8; `max_separation = u32::MAX` spreads over whole channels.
- **Dense scenes leave overlaps.** On the 300-connector bench, lines that PathFinder
  considers within capacity (a channel crossing a free band has unbounded capacity) carry
  long chains of mutually ordered segments that the channel cannot hold at separation 1;
  about 9,600 overlapping pairs remain. Blocking other connectors' ports in the router (as
  libavoid does) was tried and made it worse (10,509), so it is not in. A capacity model
  per line rather than per cross-section would address it.
- **Forced orders.** With obstacles, port-pinned segments can force orders that cost
  crossings beyond the combinatorial count. Default: hard geometric arcs win over soft
  orders; remaining cycles are broken deterministically.
- **Channel capacity model.** `(slab, gap)` resources with `h / separation + 1` tracks.
  Necessary, not sufficient, for full separation. Default as described.
- **Default costs.** Global routing uses length 1, bend 10, crossing 20, 40 iterations;
  single routes default to shortest, then fewest bends. These are scale dependent.
- **Fixed-point Euclidean costs** (`2^8` per unit for poly-lines, integer square roots
  for sleeves). Exact geometry, approximate comparison of nearly equal lengths.
- **Overlapping obstacles** are allowed for orthogonal routing and poly-lines, rejected
  for sleeve routing (the CDT needs non-crossing constraints).
- **Not built:** sleeve vertex collapse as such (replaced by wall exits) and the sleeve
  edge hints between zoom levels.
- **Full nudging objective.** Leftmost (compact) solution instead of the paper's
  weighted LP. Default: off; constrained nudging is the default.
