# Clock-power recovery experiment handoff

The implementation in this branch includes cumulative timing budgets, bounded intermediate exploration, native checkpoint validation, and restoration of the best final-feasible tree. The new default-budget version has passed implementation tests but has **not yet been benchmarked on Croc or Basilisk**. Do not attribute the measurements below to that version.

The command is `recover_clock_power -strategy bottom_up_batch`. The final timing allowance defaults to half the intrinsic delay of an inferred minimum-area compatible buffer. Override it with `-max_timing_regression`; `-max_intermediate_timing_regression` controls the larger exploration allowance. Each scene and clock group is anchored to command-start metrics. The metrics have independent allowances, and checkpoints do not replenish them. Slew and capacitance guards remain hard constraints.

## Last measured Croc implementation

These measurements use strict V5 (source SHA-256 `a90e9e6e5c2ddecb58aa0326418a39b59f8ee7fd8d09a81c8f51da98ba45dbba`), before the new cumulative-budget behavior. V5 permits up to 2 ns insertion and 1 ns clock-arrival spread, preserves each scene's setup/hold floor at the smaller of zero and its initial worst slack, and permits no new or worse slew/capacitance violations. Each run uses four threads and a 120 s soft recovery budget. Native operations can exceed the soft budget.

| Input tree | Clock power (mW) | Clock-driver area (um²) | Max insertion (ns) | Max spread (ns) | Setup slack (ns) | Hold slack (ns) | Recovery (s) |
|---|---:|---:|---:|---:|---:|---:|---:|
| All-buffer CTS tree | 3.9586 → 1.5083 | 1775.3 | 1.718585 | 0.637160 | +0.002349 | -0.021473 | 121.58 |
| Croc default CTS tree | 2.3704 → 1.3826 | 1258.8 | 1.696665 | 0.619558 | +0.003834 | -0.002584 | 121.37 |
| Manually selected CTS tree | 2.2449 → 1.5910 | 1836.0 | 1.750500 | 0.703471 | +0.012363 | +0.001058 | 106.54 |

All three V5 runs passed independent connectivity, placement, clock-envelope, scene slack-floor, and electrical report checks. V4 results, which have the same measured power and timing as V5, also matched fresh-process reloads with original input SDC. V5 itself has not had a separate fresh-process reload. The retained serial power savings are 88.0%, 86.5%, and 75.4%, respectively; the manual-tree result falls below the initial 80% research target.

Power is OpenSTA's vectorless typical-scene Clock group estimate using placement parasitics. It includes internal, switching and leakage power; it is not a routed or workload-based signoff estimate. Some activity propagation runs report the existing iteration-limit warning. Max spread is latest minus earliest arrival within a clock/scene/edge group, distinct from setup skew. Clock-driver area sums core output drivers of CLOCK-type nets.

## Last measured Basilisk implementation

Strict V5 completed with four threads in 299.631 s total, including 249.864 s recovery, and about 12.4 GiB peak RSS. It performed 180 tentative checks, accepted 152 moves into a tentative batch, rejected 28 local electrical trials, and used two native whole-design power evaluations. Exact cached clock scans took 61.688 s. The batch failed the final electrical certificate because an existing data-pin slew violation worsened by about 1.4 fs. It was rolled back: **zero buffers changed and no power improvement**. Final clock power remained 134.8973 mW, setup slack −2.811036348 ns and hold slack −12.614426613 ns.

The new version adds a small watched set of existing slew-violating pins to catch that failure during tentative moves. Its effect on quality and runtime is not yet measured. No electrical tolerance was relaxed.

## Resume work

1. Run the prepared Basilisk batch05 harness with a frozen binary of this commit. Use the original DEF plus original base and post-CTS SDC recipe. Do not substitute the historical ODB (schema mismatch) or cached ODB/exported-SDC input (a previously observed hold/clock roundtrip discrepancy).
2. Compare its reports against unchanged batch04 baseline reports using the printed final allowance. Check each scene/group separately; the new policy is command-start WNS minus allowance, not the previous zero-slack floor unless an explicit margin supplies it.
3. Run Croc with the automatic allowance, explicit zero, and at least one larger override. Recompute power, clock-driver area, insertion, spread, setup/hold and runtime. New timing-policy results cannot be assigned the old retained-savings percentage without matching the old bounds.
4. Reload saved successful results in fresh processes with original constraints before claiming final QoR.

Local, uncommitted artifacts remain in the original workspace under `evaluation/`: the generic Croc harness in `clock_recovery/`, public summaries in `recovery_batch_results/`, frozen binaries and source snapshots in `private/clock_recovery_batch_01/`, and Basilisk runs/prepared batch05 in `private/scaling_basilisk_01/`. Raw design databases, libraries, netlists and binaries are deliberately not part of this branch.

The independent Astra review found no blocking implementation issue in the final core at SHA-256 `dfd7b8274871b1aab1e98b1213e9e2b1393799902973ba2628cb31aac7fd022d`. The base OpenROAD revision is `c1b9cf06435aaae29aeef71c626920e05119474d`; Croc input revision is `71abd392bec14930292007cfb3a2cda2901bceeb`. No OpenSTA submodule files were changed.

## Verification at handoff

The final OpenROAD build passed with `--jobs=6`. All 48 C++ recovery tests, the registered Tcl command test, README checks, and the complete repository lint set passed. The pinned `//:tidy_cc` check made no changes. Independent Astra review found no remaining code or documentation blockers. The final local binary SHA-256 is `0c0807f6439431742c4f4ae33c96e36dd34750839dc4fcea189c309073f11b2a`. No new EDA runs were launched after the request to stop for the night.
