# Replanning benchmark

## 1 orbit: 6 targets, 9-10 qubits, 12 instances

| Solver | Mean value / optimum | Instances solved optimally |
|---|---:|---:|
| Exact search | 1.000 | 12/12 |
| Greedy (onboard policy) | 0.983 | 10/12 |
| Simulated annealing | 1.000 | 12/12 |

QAOA on 12 of these instances (statevector simulation, 4096 shots):

| Method | P(single shot optimal) | P(single shot feasible) | Best of 4096 = optimum | Mean time (s) |
|---|---:|---:|---:|---:|
| Uniform random bits | 0.021 | 0.230 | n/a | n/a |
| QAOA p=1 | 0.049 | 0.706 | 12/12 | 1.5 |
| QAOA p=2 | 0.056 | 0.820 | 12/12 | 6.0 |
| QAOA p=3 | 0.056 | 0.812 | 12/12 | 9.0 |

## 2 orbits: 12 targets, 16-17 qubits, 12 instances

| Solver | Mean value / optimum | Instances solved optimally |
|---|---:|---:|
| Exact search | 1.000 | 12/12 |
| Greedy (onboard policy) | 0.987 | 9/12 |
| Simulated annealing | 0.961 | 3/12 |

QAOA on 3 of these instances (statevector simulation, 4096 shots):

| Method | P(single shot optimal) | P(single shot feasible) | Best of 4096 = optimum | Mean time (s) |
|---|---:|---:|---:|---:|
| Uniform random bits | 0.001 | 0.127 | n/a | n/a |
| QAOA p=1 | 0.001 | 0.722 | 3/3 | 37.6 |
