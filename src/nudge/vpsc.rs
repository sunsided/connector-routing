//! Projection onto separation constraints, after the VPSC method of Dwyer, Marriott and
//! Stuckey ("Fast node overlap removal", GD 2005) as Wybrow et al. (2009) use it for
//! nudging: the positions closest (least squares) to the desired ones that satisfy
//! every constraint.
//!
//! Variables are grouped into blocks whose active constraints hold with equality; a
//! block sits at the mean of its members' desired positions less their offsets, or at
//! the position its fixed member dictates. Satisfying merges the blocks of the most
//! violated constraint into a block until nothing is violated; refining splits a block
//! at an active constraint with a negative Lagrange multiplier (the two halves would
//! rather move apart) and satisfies again, until every multiplier is non-negative.
//! Arithmetic is exact (rational block positions over `i128`); the result is rounded to
//! integers per block. Not checked against the original paper (unavailable here).

use alloc::vec::Vec;
use core::cmp::Ordering;

/// One constraint `x[right] - x[left] >= gap` between node indices.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Arc {
    /// The node with the smaller position.
    pub(crate) left: usize,
    /// The node with the larger position.
    pub(crate) right: usize,
    /// The minimum gap.
    pub(crate) gap: i64,
}

/// A block of variables held together by active constraints.
#[derive(Debug, Clone, Default)]
struct Block {
    /// Its nodes.
    nodes: Vec<usize>,
    /// The active constraints (indices into the arc list), a spanning tree of `nodes`.
    active: Vec<usize>,
}

/// The projection problem.
struct Projection<'a> {
    /// Desired position per node; `None` for fixed nodes, which take `fixed`.
    desired: &'a [Option<i64>],
    /// The position of fixed nodes.
    fixed: &'a [i64],
    /// The constraints.
    arcs: &'a [Arc],
    /// The block of every node.
    block_of: Vec<usize>,
    /// The offset of every node within its block.
    offset: Vec<i64>,
    /// The blocks; emptied blocks stay as tombstones.
    blocks: Vec<Block>,
}

/// A rational `num / den` with `den > 0`.
type Ratio = (i128, i128);

/// Compares two rationals.
fn cmp(a: Ratio, b: Ratio) -> Ordering {
    (a.0 * b.1).cmp(&(b.0 * a.1))
}

impl Projection<'_> {
    /// The position of block `b`: fixed by a fixed member, else the mean of desired
    /// positions less offsets.
    fn position(&self, b: usize) -> Ratio {
        let block = &self.blocks[b];
        if let Some(&n) = block.nodes.iter().find(|&&n| self.desired[n].is_none()) {
            return (i128::from(self.fixed[n] - self.offset[n]), 1);
        }

        let sum: i128 = block
            .nodes
            .iter()
            .map(|&n| i128::from(self.desired[n].expect("free node") - self.offset[n]))
            .sum();

        (
            sum,
            i128::try_from(block.nodes.len()).expect("few nodes").max(1),
        )
    }

    /// The position of node `n` as a rational.
    fn node_position(&self, n: usize) -> Ratio {
        let (num, den) = self.position(self.block_of[n]);

        (num + i128::from(self.offset[n]) * den, den)
    }

    /// The violation of arc `a` (positive if violated).
    fn violation(&self, a: usize) -> Ratio {
        let arc = self.arcs[a];
        let (ln, ld) = self.node_position(arc.left);
        let (rn, rd) = self.node_position(arc.right);
        // left + gap - right, over ld * rd.
        (ln * rd + i128::from(arc.gap) * ld * rd - rn * ld, ld * rd)
    }

    /// Merges the blocks of arc `a` (which must be different) so that `a` is tight.
    fn merge(&mut self, a: usize) {
        let arc = self.arcs[a];
        let (into, from) = (self.block_of[arc.right], self.block_of[arc.left]);
        // Shift `from`'s offsets so that offset[right] - offset[left] = gap.
        let shift = self.offset[arc.right] - arc.gap - self.offset[arc.left];
        let moved = core::mem::take(&mut self.blocks[from]);
        for &n in &moved.nodes {
            self.offset[n] += shift;
            self.block_of[n] = into;
        }

        let block = &mut self.blocks[into];
        block.nodes.extend(moved.nodes);
        block.active.extend(moved.active);
        block.active.push(a);
    }

    /// Merges along the most violated constraint until none is violated (at most
    /// `limit` merges).
    fn satisfy(&mut self, limit: usize) {
        for _ in 0..limit {
            let worst = (0..self.arcs.len())
                .filter(|&a| self.block_of[self.arcs[a].left] != self.block_of[self.arcs[a].right])
                .map(|a| (a, self.violation(a)))
                .filter(|&(_, v)| v.0 > 0)
                .max_by(|x, y| cmp(x.1, y.1).then(y.0.cmp(&x.0)));
            let Some((a, _)) = worst else {
                return;
            };
            self.merge(a);
        }
    }

    /// The nodes on the right side of active arc `a` in its block's constraint tree.
    fn right_side(&self, a: usize) -> Vec<usize> {
        let block = &self.blocks[self.block_of[self.arcs[a].right]];
        let mut side = alloc::vec![self.arcs[a].right];
        let mut stack = side.clone();
        while let Some(n) = stack.pop() {
            for &c in &block.active {
                if c == a {
                    continue;
                }

                let arc = self.arcs[c];
                for (x, y) in [(arc.left, arc.right), (arc.right, arc.left)] {
                    if x == n && !side.contains(&y) {
                        side.push(y);
                        stack.push(y);
                    }
                }
            }
        }

        side
    }

    /// The Lagrange multiplier of active arc `a`, up to a positive factor: the sum of
    /// `x - desired` over the right side, or minus that over the left side when the
    /// right side has a fixed node. `None` if both sides have one.
    fn multiplier(&self, a: usize) -> Option<Ratio> {
        let block = self.block_of[self.arcs[a].right];
        let right = self.right_side(a);
        let left: Vec<usize> = self.blocks[block]
            .nodes
            .iter()
            .copied()
            .filter(|n| !right.contains(n))
            .collect();
        let fixed = |side: &[usize]| side.iter().any(|&n| self.desired[n].is_none());
        let residual = |side: &[usize]| {
            let (pn, pd) = self.position(block);
            let num: i128 = side
                .iter()
                .map(|&n| {
                    pn + (i128::from(self.offset[n])
                        - i128::from(self.desired[n].expect("free node")))
                        * pd
                })
                .sum();
            (num, pd)
        };
        match (fixed(&right), fixed(&left)) {
            (false, _) => Some(residual(&right)),
            (true, false) => {
                let (num, den) = residual(&left);
                Some((-num, den))
            }
            (true, true) => None,
        }
    }

    /// Splits the block at the active arc with the most negative multiplier; returns
    /// whether it split one.
    fn split_once(&mut self) -> bool {
        let mut worst: Option<(usize, Ratio)> = None;
        for block in &self.blocks {
            for &a in &block.active {
                if let Some(m) = self.multiplier(a) {
                    if m.0 < 0 && worst.is_none_or(|(_, w)| cmp(m, w).is_lt()) {
                        worst = Some((a, m));
                    }
                }
            }
        }

        let Some((a, _)) = worst else {
            return false;
        };
        let old = self.block_of[self.arcs[a].right];
        let right = self.right_side(a);
        let fresh = self.blocks.len();
        let mut new_block = Block::default();
        let mut kept = Block::default();
        for &n in &self.blocks[old].nodes {
            if right.contains(&n) {
                new_block.nodes.push(n);
            } else {
                kept.nodes.push(n);
            }
        }

        for &c in &self.blocks[old].active {
            if c == a {
                continue;
            }

            if right.contains(&self.arcs[c].right) {
                new_block.active.push(c);
            } else {
                kept.active.push(c);
            }
        }

        for &n in &new_block.nodes {
            self.block_of[n] = fresh;
        }

        self.blocks[old] = kept;
        self.blocks.push(new_block);

        true
    }
}

/// The least-squares projection of `desired` positions onto the `arcs`, rounded to
/// integers. Nodes with `desired = None` are fixed at `fixed`.
pub(crate) fn project(desired: &[Option<i64>], fixed: &[i64], arcs: &[Arc]) -> Vec<i64> {
    let n = desired.len();
    let mut p = Projection {
        desired,
        fixed,
        arcs,
        block_of: (0..n).collect(),
        offset: alloc::vec![0; n],
        blocks: (0..n)
            .map(|i| Block {
                nodes: alloc::vec![i],
                active: Vec::new(),
            })
            .collect(),
    };
    let limit = 4 * (n + arcs.len()) + 16;
    p.satisfy(limit);
    for _ in 0..limit {
        if !p.split_once() {
            break;
        }

        p.satisfy(limit);
    }

    (0..n)
        .map(|i| {
            let (num, den) = p.node_position(i);
            // Round to nearest, halves up; integer offsets keep their differences.
            i64::try_from((2 * num + den).div_euclid(2 * den)).unwrap_or(0)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    //! Hand-checked projections.

    use super::*;

    /// Two segments wanting the same spot spread around it; a fixed wall pushes them.
    #[test]
    fn spreads_and_respects_walls() {
        let arcs = [Arc {
            left: 0,
            right: 1,
            gap: 4,
        }];
        assert_eq!(project(&[Some(10), Some(10)], &[0, 0], &arcs), [8, 12]);

        // A wall at 9 (fixed node 2) left of both.
        let arcs = [
            Arc {
                left: 2,
                right: 0,
                gap: 0,
            },
            Arc {
                left: 0,
                right: 1,
                gap: 4,
            },
        ];
        assert_eq!(
            project(&[Some(10), Some(10), None], &[0, 0, 9], &arcs),
            [9, 13, 9]
        );

        // A constraint that is not needed stays inactive after refinement.
        let arcs = [
            Arc {
                left: 0,
                right: 1,
                gap: 2,
            },
            Arc {
                left: 1,
                right: 2,
                gap: 2,
            },
        ];
        assert_eq!(
            project(&[Some(0), Some(10), Some(11)], &[0; 3], &arcs),
            [0, 10, 12]
        );
    }
}
