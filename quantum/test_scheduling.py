import itertools

import numpy as np
import pytest

from problem import load_catalog, make_instance, to_qubo
from solvers import exact, greedy, simulated_annealing


def test_catalog_matches_flight_code():
    cat = load_catalog()
    assert len(cat) == 6
    assert (cat[1].offset, cat[1].duration, cat[1].value) == (700, 500, 6)


def brute_force_qubo(q):
    m = q.Q.shape[0]
    return min(q.energy(np.array(x, dtype=float)) for x in itertools.product((0, 1), repeat=m))


@pytest.mark.parametrize("seed", range(8))
def test_qubo_minimum_is_the_true_optimum(seed):
    inst = make_instance(np.random.default_rng(seed))
    best, _, _ = exact(inst)
    assert brute_force_qubo(to_qubo(inst)) == pytest.approx(-best)


@pytest.mark.parametrize("seed", range(8))
def test_qubo_ground_states_are_feasible(seed):
    inst = make_instance(np.random.default_rng(seed))
    q = to_qubo(inst)
    m = q.Q.shape[0]
    e_min = brute_force_qubo(q)
    for x in itertools.product((0, 1), repeat=m):
        if q.energy(np.array(x, dtype=float)) == pytest.approx(e_min):
            assert inst.feasible(x[: inst.n])


@pytest.mark.parametrize("seed", range(20))
def test_greedy_is_always_feasible(seed):
    inst = make_instance(np.random.default_rng(seed), orbits=2)
    assert inst.feasible(greedy(inst))


def test_annealing_energy_bookkeeping_finds_optimum():
    inst = make_instance(np.random.default_rng(3))
    q = to_qubo(inst)
    x = simulated_annealing(q, np.random.default_rng(0), sweeps=300, restarts=4)
    best, _, _ = exact(inst)
    assert inst.value(tuple(int(b) for b in x[: inst.n])) == best


def test_qaoa_runs_end_to_end():
    from qaoa import run_qaoa

    inst = make_instance(np.random.default_rng(1))
    best, _, _ = exact(inst)
    r = run_qaoa(inst, to_qubo(inst), best, reps=1, shots=512, restarts=1)
    assert r.best_selection is not None
    assert inst.feasible(r.best_selection)
    assert 0.0 <= r.p_optimal <= r.p_feasible <= 1.0
