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
        for comp in self.components() {
            let (t, relaxed) = self.choose_gap(&comp);
            if comp.has_pairs {
                separation = Some(separation.map_or(t, |s: i64| s.min(t)));
            }

            let lo = self
                .lowest(&comp, t, relaxed)
                .expect("the chosen gap is feasible");
            let hi = self.highest(&comp, t, relaxed);
            let anchored_low = self.anchored(&comp, true);
            let anchored_high = self.anchored(&comp, false);
            for (&v, &l) in &lo {
                // Bounded on both sides: centred. Open on one side: packed against the
                // anchored side (routes outside the scene hug it). Free: placed below.
                position[v] = match (anchored_low, anchored_high) {
                    (true, true) => (l + hi[&v]).div_euclid(2),
                    (false, true) => hi[&v],
                    _ => l,
                };
            }

            if !anchored_low && !anchored_high {
                self.float(&comp, &lo, &hi, &mut position);
            }
        }

        Solution {
            position,
            separation,
        }
    }

    /// Picks the common gap of a component: the largest gap with obstacles kept at that
    /// gap too if it reaches the separation, else the largest gap with obstacles allowed
    /// to touch.
    fn choose_gap(&self, comp: &Component) -> (i64, bool) {
        let bounded = self.anchored(comp, true) && self.anchored(comp, false);
        let cap = if bounded {
            self.max_separation
        } else {
            self.separation
        };
        let strict = self.largest_gap(comp, cap, false).unwrap_or(0);
        if strict >= self.separation.min(cap) {
            return (strict, false);
        }

        let relaxed = self.largest_gap(comp, cap, true).unwrap_or(0);
        if relaxed > strict {
            (relaxed, true)
        } else {
            (strict, false)
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
