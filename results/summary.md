Campaign: 1000 missions per scenario per controller, same seeds for both.

| Scenario | Controller | Runs with a safety violation | Violation episodes | Faults detected | Mean detection latency (s) | False safe-mode entries / run | Science vs. plan |
|---|---|---:|---:|---:|---:|---:|---:|
| nominal | naive | 0.0% | 0 | n/a | n/a | 10.36 | 99.9% |
| nominal | hardened | 0.0% | 0 | n/a | n/a | 0.00 | 100.0% |
| sensor | naive | 1.3% | 14 | 0.0% | n/a | 5.91 | 97.8% |
| sensor | hardened | 0.0% | 0 | 89.9% | 257.0 | 0.00 | 99.0% |
| power | naive | 0.0% | 0 | 71.6% | 4724.8 | 3.80 | 98.9% |
| power | hardened | 0.0% | 0 | 90.4% | 14.9 | 0.00 | 90.2% |
| comms | naive | 58.4% | 2134 | n/a | n/a | 4.29 | 65.9% |
| comms | hardened | 0.0% | 0 | n/a | n/a | 0.00 | 112.0% |
| seu | naive | 46.2% | 749 | 0.0% | n/a | 3.32 | 93.8% |
| seu | hardened | 0.0% | 0 | 100.0% | 0.0 | 0.00 | 99.8% |
| combined | naive | 28.2% | 523 | 2.9% | 5418.9 | 1.89 | 62.4% |
| combined | hardened | 0.0% | 0 | 100.0% | 12.4 | 0.00 | 107.7% |
| seu_storm | naive | 100.0% | 3749 | 0.0% | n/a | 0.00 | 176.0% |
| seu_storm | hardened | 0.0% | 0 | 100.0% | 0.0 | 0.00 | 5.3% |

### Violations by type (all scenarios)

| Controller | deep_discharge | payload_low_power | invalid_mode | payload_in_safe | duplicate_exec | stale_exec |
|---|---:|---:|---:|---:|---:|---:|
| naive | 1251 | 1515 | 2080 | 130 | 1019 | 1174 |
| hardened | 0 | 0 | 0 | 0 | 0 | 0 |

### Detection by fault type (hardened controller, all scenarios)

| Fault | Observable events | Detected | Mean latency (s) |
|---|---:|---:|---:|
| batt_bias | 1011 | 100.0% | 25.4 |
| batt_stuck | 989 | 100.0% | 10.0 |
| cell_loss | 599 | 74.5% | 9.0 |
| gyro_bias_jump | 614 | 96.7% | 557.0 |
| seu | 2785292 | 100.0% | 0.0 |
| sun_bias | 645 | 91.6% | 328.8 |
| sun_noise | 488 | 91.0% | 50.3 |
| sun_stuck | 598 | 95.8% | 10.1 |
