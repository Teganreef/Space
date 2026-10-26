# Engineering log

Problems found by running the fault campaign against the hardened controller,
what caused them and what changed. Numbers are from campaign runs over the
same seeds before and after each change.

## 1. Start-up transient read as a gyro fault

**Symptom.** In the *nominal* scenario, with no faults injected, the hardened
controller spent on average 49% of the mission in degraded mode (100 runs).
The affected runs went degraded right after boot (tick 10) and stayed there.

**Cause.** The attitude filter started from an estimate of 0 rad while the
spacecraft could be anywhere within +/-0.3 rad. The first innovations were
large, the bias estimator integrated them as gyro bias, the bias estimate
crossed the fault threshold and the gyro was flagged as failed for the rest of
the mission.

**Fix.** Initialise the angle estimate directly from the first measurement on
which both sun sensors agree, and only learn bias while the innovation is
small (the filter is tracking, not converging). Regression test:
`Harness.StartupTransientIsNotMistakenForGyroFault`.

**After.** No degraded time in nominal runs.

## 2. Discarding the command table was safe but too expensive

**Symptom.** Zero safety violations under single event upsets, but in the SEU
scenario (about 68 upsets per mission) the hardened controller kept only
**5.1%** of planned science time (200 runs).

**Cause.** The command table was CRC protected and dropped on any mismatch.
It made up 96 of the 112 bytes the upsets could land in, so most upsets wiped
the plan, and nothing came back until the next ground pass.

**Fix.** Keep two copies of the table, each with its own CRC. The CRC
identifies the damaged copy and the other one repairs it. The plan is only
dropped when both copies are damaged in the same cycle.

**After.** Same 200 seeds: **99.9%** science, still zero violations.

**Where it stops working.** In the `seu_storm` stress scenario (one upset
every 10 s, far above anything realistic in LEO) two upsets land in the same
control cycle often enough that both copies get hit, and science falls to
about 5%. The controller stays safe (no violations in 1000 storm missions)
because it drops the plan rather than executing a corrupted one. Scrubbing
faster than the control cycle, or a third copy with voting, would be the next
step.

## 3. A second fault hidden behind the first

**Symptom.** In the power scenario, 46% of real battery cell losses went
undetected (54% detection over 200 runs).

**Cause.** If the voltage sensor had already been declared suspect (reading
high), the cell-loss check was skipped, and the coulomb counter kept
believing charge that no longer existed.

**Fix.** While the voltage sensor is suspect, estimate its offset against the
coulomb counter and keep using the corrected reading. A constant sensor
offset cannot produce a step change, so a real loss of charge still shows up.

**After.** 78% detection (same seeds). The remaining misses are cell losses
while the voltage sensor is *frozen*: with one sensor stuck and the other
estimate integrating current, nothing onboard can see a sudden loss of stored
charge. That is a sensing gap, not a logic gap, and it is listed under
limitations.

## 4. Gyro bias jump after losing a sun sensor

**Symptom.** Gyro bias jumps were sometimes missed when one sun sensor had
already failed.

**Cause.** Bias was only learned while both sun sensors agreed.

**Fix.** Learn bias from a single remaining sensor too. A constant offset in
that sensor is absorbed by the angle estimate; a rate error keeps producing
innovation and is absorbed by the bias estimate.

**After.** Sensor-scenario detection 85.1% to 87.2% (200 runs).

## 5. Scoring faults nobody could see

Not a controller change, a measurement fix. Sun sensor faults that start and
end during eclipse never alter a reading, yet they were counted as missed
detections. They are now reported separately as unobservable and excluded
from detection rates. This moved the sensor-scenario detection rate from
81.8% to 85.1% without touching the controller, and is recorded here so the
numbers are not mistaken for an improvement in the flight code.

## 6. Single bad samples leaking into the charge estimate

The hardened controller already ignored single-sample glitches when deciding
anything (persistence counters), but the state-of-charge estimate itself
passed them through, which showed up as spikes in the replay plots. The
estimate now holds the coulomb-counter value for a sample that disagrees with
it, until the disagreement persists. No change in campaign metrics.
