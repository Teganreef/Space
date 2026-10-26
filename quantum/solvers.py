"""Classical baselines for the replanning problem."""

from __future__ import annotations

import itertools

import numpy as np

from problem import PAYLOAD_W, Instance, Qubo


def exact(inst: Instance) -> tuple[int, tuple[int, ...], int]:
    """Brute force over target subsets. Returns (best value, a best selection, number of optimal selections)."""
    best, best_x, count = -1, (), 0
    for x in itertools.product((0, 1), repeat=inst.n):
        if not inst.feasible(x):
            continue
        v = inst.value(x)
        if v > best:
            best, best_x, count = v, x, 1
        elif v == best:
            count += 1
    return best, best_x, count


def greedy(inst: Instance) -> tuple[int, ...]:
    """Same policy as the onboard scheduler (src/scheduler.cpp): value per joule first."""
    order = sorted(range(inst.n), key=lambda i: -inst.targets[i].value / (inst.targets[i].duration * PAYLOAD_W))
    conflicts = set(inst.conflicts)
    chosen = [0] * inst.n
    spent = 0
    for i in order:
        t = inst.targets[i]
        if spent + t.energy_units > inst.budget:
            continue
        if any(chosen[j] and (min(i, j), max(i, j)) in conflicts for j in range(inst.n)):
            continue
        chosen[i] = 1
        spent += t.energy_units
    return tuple(chosen)


def simulated_annealing(q: Qubo, rng: np.random.Generator, sweeps: int = 2000, restarts: int = 8) -> np.ndarray:
    """Single-bit-flip Metropolis on the QUBO with a geometric temperature schedule."""
    Q = q.Q + q.Q.T - np.diag(np.diag(q.Q))  # symmetric, diagonal kept once
    m = Q.shape[0]
    scale = float(np.max(np.abs(q.Q)))
    best_x, best_e = None, np.inf
    for _ in range(restarts):
        x = rng.integers(0, 2, m).astype(float)
        e = q.energy(x)
        for t in np.geomspace(scale, scale * 1e-3, sweeps):
            for i in rng.permutation(m):
                # Energy change of flipping bit i.
                d = (1 - 2 * x[i]) * (Q[i, i] + (Q[i] @ x - Q[i, i] * x[i]))
                if d <= 0 or rng.random() < np.exp(-d / t):
                    x[i] = 1 - x[i]
                    e += d
            if e < best_e:
                best_e, best_x = e, x.copy()
    return best_x
