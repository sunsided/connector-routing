//! An exact solver for the separation constraints of one pass, without linear
//! programming.
//!
//! Every constraint is a difference constraint `x_r - x_l >= gap`, and every constraint
//! points forward in the scan order, so the constraint graph is acyclic: the leftmost
//! feasible positions follow in one forward pass (longest paths), the rightmost in one
//! backward pass. Per independent component of movable segments, the solver finds the
//! largest common gap `t` by binary search on that feasibility test, then places each
//! segment between its extreme positions.

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use super::constraints::{Constraint, Item, Kind};

/// The inputs of one pass.
pub(crate) struct Problem<'a> {
    /// The constraints, all pointing forward in `order`.
    pub(crate) constraints: &'a [Constraint],
    /// The position of every non-movable item (obstacle sides, fixed segments, walls).
    pub(crate) constant: &'a dyn Fn(Item) -> i64,
    /// Whether segment `i` may move.
    pub(crate) movable: &'a [bool],
    /// The scan rank of every segment; constraints go from lower to higher rank.
    pub(crate) order: &'a [usize],
    /// The current position of every segment, used to place free-floating components.
    pub(crate) current: &'a [i32],
    /// The desired separation between segments.
    pub(crate) separation: i64,
    /// The largest gap to spread segments to in components bounded on both sides.
    pub(crate) max_separation: i64,
    /// Whether to project desired positions (VPSC) instead of spreading.
    pub(crate) project: bool,
    /// Whether each segment is the middle of a Z or S (its neighbours leave to opposite
    /// sides), whose desired position is the middle of its alley.
    pub(crate) zig: &'a [bool],
}

/// The result of one pass.
pub(crate) struct Solution {
    /// The new position of every segment (unchanged for fixed ones).
    pub(crate) position: Vec<i64>,
    /// The smallest gap between segments achieved in any component with two or more
    /// constrained segments; `None` if there is no such component.
    pub(crate) separation: Option<i64>,
}

/// A constraint arc of one component: the other end and the kind.
type Arc = (Item, Kind);

/// One independent component of movable segments.
struct Component {
    /// Its segments in scan order.
    members: Vec<usize>,
    /// Incoming arcs per member.
    incoming: BTreeMap<usize, Vec<Arc>>,
    /// Outgoing arcs per member.
    outgoing: BTreeMap<usize, Vec<Arc>>,
    /// Whether it has a segment-to-segment constraint.
    has_pairs: bool,
}

/// Union-find root of `i`, with path halving.
fn root(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }

    i
}

impl Problem<'_> {
    /// Whether item `item` is a movable segment.
    fn is_var(&self, item: Item) -> Option<usize> {
        match item {
            Item::Segment(i) if self.movable[i] => Some(i),
            _ => None,
        }
    }

    /// Splits the movable segments into independent components.
    fn components(&self) -> Vec<Component> {
        let n = self.movable.len();
        let mut parent: Vec<usize> = (0..n).collect();
        for c in self.constraints {
            if let (Some(a), Some(b)) = (self.is_var(c.left), self.is_var(c.right)) {
                let (ra, rb) = (root(&mut parent, a), root(&mut parent, b));
                parent[ra] = rb;
            }
        }

        let mut by_root: BTreeMap<usize, Component> = BTreeMap::new();
        let mut members: Vec<usize> = (0..n).filter(|&i| self.movable[i]).collect();
        members.sort_by_key(|&i| self.order[i]);
        for i in members {
            let r = root(&mut parent, i);
            by_root
                .entry(r)
                .or_insert_with(|| Component {
                    members: Vec::new(),
                    incoming: BTreeMap::new(),
                    outgoing: BTreeMap::new(),
                    has_pairs: false,
                })
                .members
                .push(i);
        }

        for c in self.constraints {
            if let Some(b) = self.is_var(c.right) {
                let comp = by_root.get_mut(&root(&mut parent, b)).expect("component");
                comp.incoming.entry(b).or_default().push((c.left, c.kind));
                comp.has_pairs |= c.kind == Kind::Segments;
            }

            if let Some(a) = self.is_var(c.left) {
                let comp = by_root.get_mut(&root(&mut parent, a)).expect("component");
                comp.outgoing.entry(a).or_default().push((c.right, c.kind));
                comp.has_pairs |= c.kind == Kind::Segments;
            }
        }

        by_root.into_values().collect()
    }

    /// The gap of a constraint of `kind` for common gap `t`; obstacle gaps are `t` or,
    /// when `relaxed`, zero.
    fn gap(kind: Kind, t: i64, relaxed: bool) -> i64 {
        match kind {
            Kind::Segments => t,
            Kind::Obstacle if !relaxed => t,
            Kind::Obstacle | Kind::Wall | Kind::Shape => 0,
        }
    }

    /// The leftmost feasible positions of `comp`'s members for gap `t`, or `None` if
    /// infeasible.
    fn lowest(&self, comp: &Component, t: i64, relaxed: bool) -> Option<BTreeMap<usize, i64>> {
        let mut lo: BTreeMap<usize, i64> = BTreeMap::new();
        for &v in &comp.members {
            let mut value = i64::MIN;
            for &(from, kind) in comp.incoming.get(&v).map_or(&[][..], Vec::as_slice) {
                let base = match self.is_var(from) {
                    Some(u) => lo[&u],
                    None => (self.constant)(from),
                };
                value = value.max(base + Self::gap(kind, t, relaxed));
            }

            lo.insert(v, value);
        }

        for &v in &comp.members {
            for &(to, kind) in comp.outgoing.get(&v).map_or(&[][..], Vec::as_slice) {
                if self.is_var(to).is_none()
                    && lo[&v] + Self::gap(kind, t, relaxed) > (self.constant)(to)
                {
                    return None;
                }
            }
        }

        Some(lo)
    }

    /// The rightmost feasible positions of `comp`'s members for a feasible gap `t`.
    fn highest(&self, comp: &Component, t: i64, relaxed: bool) -> BTreeMap<usize, i64> {
        let mut hi: BTreeMap<usize, i64> = BTreeMap::new();
        for &v in comp.members.iter().rev() {
            let mut value = i64::MAX;
            for &(to, kind) in comp.outgoing.get(&v).map_or(&[][..], Vec::as_slice) {
                let base = match self.is_var(to) {
                    Some(w) => hi[&w],
                    None => (self.constant)(to),
                };
                value = value.min(base - Self::gap(kind, t, relaxed));
            }

            hi.insert(v, value);
        }

        hi
    }

    /// The largest `t` in `[0, cap]` that is feasible, if any.
    fn largest_gap(&self, comp: &Component, cap: i64, relaxed: bool) -> Option<i64> {
        self.lowest(comp, 0, relaxed)?;
        let (mut good, mut bad) = (0, cap + 1);
        while bad - good > 1 {
            let mid = good + (bad - good) / 2;
            if self.lowest(comp, mid, relaxed).is_some() {
                good = mid;
            } else {
                bad = mid;
            }
        }

        Some(good)
    }

    /// Solves the pass.
    pub(crate) fn solve(&self) -> Solution {
        let mut position: Vec<i64> = self.current.iter().map(|&c| i64::from(c)).collect();
        let mut separation: Option<i64> = None;
        let mut placed: BTreeMap<usize, i64> = BTreeMap::new();
        // Water-filling: each round gives every component its largest common gap; a
        // component held below its cap by a bottleneck places only the bottleneck's
        // segments and leaves the rest for the next round, where they get gaps of their
        // own. So one tight spot does not squeeze a whole bundle.
        loop {
            let movable: Vec<bool> = (0..self.movable.len())
                .map(|i| self.movable[i] && !placed.contains_key(&i))
                .collect();
            if !movable.iter().any(|&m| m) {
                break;
            }

            let constant = |item: Item| match item {
                Item::Segment(i) if placed.contains_key(&i) => placed[&i],
                _ => (self.constant)(item),
            };
            let round = Problem {
                movable: &movable,
                constant: &constant,
                ..*self
            };
            let finished = round.solve_round(&mut position, &mut separation);
            for v in finished {
                placed.insert(v, position[v]);
            }
        }

        Solution {
            position,
            separation,
        }
    }

    /// One water-filling round over all components; returns the segments it placed.
    fn solve_round(&self, position: &mut [i64], separation: &mut Option<i64>) -> Vec<usize> {
        let mut finished = Vec::new();
        for comp in self.components() {
            let (t, relaxed, cap) = self.choose_gap(&comp);
            if comp.has_pairs {
                *separation = Some(separation.map_or(t, |s: i64| s.min(t)));
            }

            let lo = self
                .lowest(&comp, t, relaxed)
                .expect("the chosen gap is feasible");
            let hi = self.highest(&comp, t, relaxed);
            if t < cap && comp.has_pairs {
                // Place the bottleneck (the members with the least slack) and leave the
                // others for the next round.
                let least = comp
                    .members
                    .iter()
                    .map(|v| hi[v] - lo[v])
                    .min()
                    .unwrap_or(0);
                for &v in &comp.members {
                    if hi[&v] - lo[&v] == least {
                        position[v] = (lo[&v] + hi[&v]).div_euclid(2);
                        finished.push(v);
                    }
                }

                continue;
            }

            finished.extend(comp.members.iter().copied());
            self.place(&comp, (t, relaxed), (&lo, &hi), position);
        }

        finished
    }

    /// Places a whole component at gap `t` between its extreme solutions `lo` and `hi`.
    fn place(
        &self,
        comp: &Component,
        gap: (i64, bool),
        (lo, hi): (&BTreeMap<usize, i64>, &BTreeMap<usize, i64>),
        position: &mut [i64],
    ) {
        let anchored_low = self.anchored(comp, true);
        let anchored_high = self.anchored(comp, false);
        if self.project {
            self.project_component(comp, gap, (lo, hi), anchored_low && anchored_high, position);
            return;
        }

        for (&v, &l) in lo {
            // Bounded on both sides: centred. Open on one side: packed against the
            // anchored side (routes outside the scene hug it). Free: placed below.
            position[v] = match (anchored_low, anchored_high) {
                (true, true) => (l + hi[&v]).div_euclid(2),
                (false, true) => hi[&v],
                _ => l,
            };
        }

        if !anchored_low && !anchored_high {
            self.float(comp, lo, hi, position);
        }
    }

    /// Places a component by least-squares projection of its desired positions (Wybrow
    /// et al. 2009, section 5.2): the middle of the alley for the middle segment of a Z
    /// (when the component is bounded on both sides), the current position otherwise.
    /// The rounded projection is repaired into `[lo, hi]` in scan order, which keeps
    /// every constraint.
    fn project_component(
        &self,
        comp: &Component,
        (t, relaxed): (i64, bool),
        (lo, hi): (&BTreeMap<usize, i64>, &BTreeMap<usize, i64>),
        bounded: bool,
        position: &mut [i64],
    ) {
        let mut index: BTreeMap<Item, usize> = BTreeMap::new();
        let mut desired = Vec::new();
        let mut fixed = Vec::new();
        for &v in &comp.members {
            index.insert(Item::Segment(v), desired.len());
            let middle = (lo[&v] + hi[&v]).div_euclid(2);
            desired.push(Some(if bounded && self.zig[v] {
                middle
            } else {
                i64::from(self.current[v])
            }));
            fixed.push(0);
        }

        let mut node = |item: Item, desired: &mut Vec<Option<i64>>, fixed: &mut Vec<i64>| {
            *index.entry(item).or_insert_with(|| {
                desired.push(None);
                fixed.push((self.constant)(item));
                desired.len() - 1
            })
        };
        let mut arcs = Vec::new();
        for &v in &comp.members {
            for &(from, kind) in comp.incoming.get(&v).map_or(&[][..], Vec::as_slice) {
                let left = node(from, &mut desired, &mut fixed);
                let right = node(Item::Segment(v), &mut desired, &mut fixed);
                arcs.push(super::vpsc::Arc {
                    left,
                    right,
                    gap: Self::gap(kind, t, relaxed),
                });
            }

            for &(to, kind) in comp.outgoing.get(&v).map_or(&[][..], Vec::as_slice) {
                if self.is_var(to).is_none() {
                    let left = node(Item::Segment(v), &mut desired, &mut fixed);
                    let right = node(to, &mut desired, &mut fixed);
                    arcs.push(super::vpsc::Arc {
                        left,
                        right,
                        gap: Self::gap(kind, t, relaxed),
                    });
                }
            }
        }

        let projected = super::vpsc::project(&desired, &fixed, &arcs);
        for &v in &comp.members {
            let mut x = projected[index[&Item::Segment(v)]].max(lo[&v]);
            for &(from, kind) in comp.incoming.get(&v).map_or(&[][..], Vec::as_slice) {
                let base = match self.is_var(from) {
                    Some(u) => position[u],
                    None => (self.constant)(from),
                };
                x = x.max(base + Self::gap(kind, t, relaxed));
            }

            position[v] = x.min(hi[&v]);
        }
    }

    /// Picks the common gap of a component, with the cap it was searched up to: the largest gap with obstacles kept at that
    /// gap too if it reaches the separation, else the largest gap with obstacles allowed
    /// to touch.
    fn choose_gap(&self, comp: &Component) -> (i64, bool, i64) {
        let bounded = self.anchored(comp, true) && self.anchored(comp, false);
        let cap = if bounded && !self.project {
            self.max_separation
        } else {
            self.separation
        };
        let strict = self.largest_gap(comp, cap, false).unwrap_or(0);
        if strict >= self.separation.min(cap) {
            return (strict, false, cap);
        }

        let relaxed = self.largest_gap(comp, cap, true).unwrap_or(0);
        if relaxed > strict {
            (relaxed, true, cap)
        } else {
            (strict, false, cap)
        }
    }

    /// Whether some member is constrained by an obstacle or a fixed segment on the low
    /// (`low = true`) or high side, so the component is anchored there.
    fn anchored(&self, comp: &Component, low: bool) -> bool {
        let arcs = if low { &comp.incoming } else { &comp.outgoing };
        arcs.values()
            .flatten()
            .any(|&(item, kind)| kind != Kind::Wall && self.is_var(item).is_none())
    }

    /// Places a component that only walls bound: the leftmost solution shifted as close
    /// to the current position of its first member as the slack allows.
    fn float(
        &self,
        comp: &Component,
        lo: &BTreeMap<usize, i64>,
        hi: &BTreeMap<usize, i64>,
        position: &mut [i64],
    ) {
        let first = comp.members[0];
        let slack = comp
            .members
            .iter()
            .map(|v| hi[v] - lo[v])
            .min()
            .unwrap_or(0);
        let shift = (i64::from(self.current[first]) - lo[&first]).clamp(0, slack.max(0));
        for &v in &comp.members {
            position[v] = lo[&v] + shift;
        }
    }
}
