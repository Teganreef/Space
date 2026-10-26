"""Payload replanning as a knapsack with conflicts, and its QUBO form.

After a recovery the ground has to rebuild the payload plan: choose which
targets to observe so total science value is maximised, no two chosen windows
overlap in time, and the energy spent fits what the battery can afford.

The target catalog is read from the flight code header so the scheduler on
the spacecraft and this benchmark cannot drift apart.
"""

from __future__ import annotations

import itertools
import math
import pathlib
import re
from dataclasses import dataclass, field

import numpy as np

HEADER = pathlib.Path(__file__).resolve().parent.parent / "include" / "sfs" / "scheduler.hpp"
PAYLOAD_W = 8.0
ENERGY_UNIT_J = 800.0  # budget is expressed in integer units of this size
ORBIT_TICKS = 5400


@dataclass(frozen=True)
class Target:
    offset: int
    duration: int
    value: int

    @property
    def energy_units(self) -> int:
        return round(self.duration * PAYLOAD_W / ENERGY_UNIT_J)


def load_catalog(header: pathlib.Path = HEADER) -> list[Target]:
    text = header.read_text()
    block = text[text.index("kOnboardTargets"):]
    block = block[: block.index("}};")]
    return [Target(int(a), int(b), int(c)) for a, b, c in re.findall(r"\{(\d+),\s*(\d+),\s*(\d+)\}", block)]


@dataclass
class Instance:
    targets: list[Target]
    budget: int  # energy units
    conflicts: list[tuple[int, int]] = field(default_factory=list)

    @property
    def n(self) -> int:
        return len(self.targets)

    @property
    def slack_bits(self) -> int:
        return max(1, math.ceil(math.log2(self.budget + 1)))

    @property
    def n_qubits(self) -> int:
        return self.n + self.slack_bits

    def feasible(self, chosen: tuple[int, ...] | list[int]) -> bool:
        x = list(chosen)
        if sum(self.targets[i].energy_units * x[i] for i in range(self.n)) > self.budget:
            return False
        return all(not (x[i] and x[j]) for i, j in self.conflicts)

    def value(self, chosen) -> int:
        return sum(self.targets[i].value * chosen[i] for i in range(self.n))


def overlaps(a: Target, b: Target) -> bool:
    return a.offset < b.offset + b.duration and b.offset < a.offset + a.duration


def make_instance(rng: np.random.Generator, orbits: int = 1) -> Instance:
    """Catalog targets over `orbits` orbits with perturbed values and a random budget."""
    catalog = load_catalog()
    targets = []
    for k in range(orbits):
        for t in catalog:
            v = int(np.clip(t.value + rng.integers(-2, 3), 1, 9))
            targets.append(Target(t.offset + k * ORBIT_TICKS, t.duration, v))
    total = sum(t.energy_units for t in targets)
    budget = int(rng.integers(total // 4, total // 2 + 1))
    conflicts = [(i, j) for i, j in itertools.combinations(range(len(targets)), 2)
                 if overlaps(targets[i], targets[j])]
    return Instance(targets, budget, conflicts)


# ---------------------------------------------------------------- QUBO ----

@dataclass
class Qubo:
    """Minimise  x^T Q x + offset  over binary x (Q upper triangular)."""

    Q: np.ndarray
    offset: float

    def energy(self, x: np.ndarray) -> float:
        return float(x @ self.Q @ x + self.offset)


def to_qubo(inst: Instance, penalty: float | None = None) -> Qubo:
    """
    Variables: one per target, then `slack_bits` slack variables that turn
    the budget inequality into an equality.

        minimise  - sum v_i x_i
                  + P * sum_{(i,j) conflicting} x_i x_j
                  + P * (sum e_i x_i + sum 2^k s_k - B)^2

    P is set just above the largest single value, which is enough for any
    infeasible assignment to cost more than the best feasible one, while
    keeping the energy landscape as flat as possible for QAOA.
    """
    n, k = inst.n, inst.slack_bits
    m = n + k
    p = penalty if penalty is not None else max(t.value for t in inst.targets) + 1.0
    Q = np.zeros((m, m))

    for i, t in enumerate(inst.targets):
        Q[i, i] -= t.value
    for i, j in inst.conflicts:
        Q[i, j] += p

    w = np.array([t.energy_units for t in inst.targets] + [2 ** b for b in range(k)], dtype=float)
    B = float(inst.budget)
    for a in range(m):
        Q[a, a] += p * (w[a] * w[a] - 2.0 * B * w[a])
        for b in range(a + 1, m):
            Q[a, b] += p * 2.0 * w[a] * w[b]
    return Qubo(Q, p * B * B)


def decode(inst: Instance, bits) -> tuple[int, ...]:
    """Target selection from a full bitstring (slack bits dropped)."""
    return tuple(int(b) for b in bits[: inst.n])
