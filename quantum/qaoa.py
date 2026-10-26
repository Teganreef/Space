"""QAOA for the replanning QUBO, run on Qiskit's statevector simulator.

Real hardware is optional (see run_on_ibm). Nothing here claims a speedup:
at these sizes exact search takes microseconds. The point is the pipeline -
formulate, map to qubits, optimise, sample, decode, check feasibility - and an
honest measure of how much probability QAOA puts on good answers.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from qiskit.circuit.library import qaoa_ansatz
from qiskit.primitives import StatevectorEstimator, StatevectorSampler
from qiskit.quantum_info import SparsePauliOp
from scipy.optimize import minimize

from problem import Instance, Qubo, decode


def to_ising(q: Qubo) -> tuple[SparsePauliOp, float]:
    """Substitute x = (1 - z) / 2. Returns (operator, constant). Qubit i is variable i."""
    m = q.Q.shape[0]
    terms: list[tuple[str, list[int], float]] = []
    const = q.offset
    for i in range(m):
        h = q.Q[i, i]
        const += h / 2
        if h:
            terms.append(("Z", [i], -h / 2))
        for j in range(i + 1, m):
            J = q.Q[i, j]
            if not J:
                continue
            const += J / 4
            terms.append(("Z", [i], -J / 4))
            terms.append(("Z", [j], -J / 4))
            terms.append(("ZZ", [i, j], J / 4))
    op = SparsePauliOp.from_sparse_list(terms, num_qubits=m).simplify()
    return op, float(const)


@dataclass
class QaoaResult:
    reps: int
    best_selection: tuple[int, ...] | None
    best_value: int
    p_optimal: float   # probability a single shot decodes to an optimal selection
    p_feasible: float  # probability a single shot decodes to a feasible selection
    evaluations: int
    params: np.ndarray


def run_qaoa(inst: Instance, q: Qubo, optimum: int, reps: int = 2, shots: int = 4096,
             restarts: int = 3, seed: int = 0) -> QaoaResult:
    op, _ = to_ising(q)
    # Normalising the cost does not move its minimum but keeps the angles in
    # a range where the optimiser behaves.
    op = op / float(np.max(np.abs(op.coeffs)))
    ansatz = qaoa_ansatz(op, reps=reps)
    estimator = StatevectorEstimator()
    rng = np.random.default_rng(seed)
    evals = 0

    def cost(params):
        nonlocal evals
        evals += 1
        return float(estimator.run([(ansatz, op, params)]).result()[0].data.evs)

    best = None
    for _ in range(restarts):
        x0 = rng.uniform(0, np.pi, ansatz.num_parameters)
        r = minimize(cost, x0, method="COBYLA", options={"maxiter": 300})
        if best is None or r.fun < best.fun:
            best = r

    measured = ansatz.assign_parameters(best.x)
    measured.measure_all()
    counts = StatevectorSampler(seed=seed).run([measured], shots=shots).result()[0].data.meas.get_counts()

    best_sel, best_val, n_opt, n_feas = None, -1, 0, 0
    for key, c in counts.items():
        bits = [int(b) for b in reversed(key)]  # qubit 0 is the rightmost character
        sel = decode(inst, bits)
        if not inst.feasible(sel):
            continue
        n_feas += c
        v = inst.value(sel)
        if v == optimum:
            n_opt += c
        if v > best_val:
            best_sel, best_val = sel, v
    return QaoaResult(reps, best_sel, best_val, n_opt / shots, n_feas / shots, evals, best.x)


def run_on_ibm(inst: Instance, q: Qubo, params: np.ndarray, reps: int, backend_name: str | None = None,
               shots: int = 4096):  # pragma: no cover - needs an IBM Quantum account
    """Sample optimised QAOA parameters on IBM hardware.

    Requires `pip install qiskit-ibm-runtime` and a saved account
    (QiskitRuntimeService.save_account(...)). Not exercised by the tests.
    """
    from qiskit.transpiler.preset_passmanagers import generate_preset_pass_manager
    from qiskit_ibm_runtime import QiskitRuntimeService, SamplerV2

    service = QiskitRuntimeService()
    backend = service.backend(backend_name) if backend_name else service.least_busy(operational=True, simulator=False)
    op, _ = to_ising(q)
    op = op / float(np.max(np.abs(op.coeffs)))
    circuit = qaoa_ansatz(op, reps=reps).assign_parameters(params)
    circuit.measure_all()
    isa = generate_preset_pass_manager(optimization_level=3, backend=backend).run(circuit)
    return SamplerV2(mode=backend).run([isa], shots=shots).result()[0].data.meas.get_counts()
