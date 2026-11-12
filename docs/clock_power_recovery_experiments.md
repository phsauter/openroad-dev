# Clock-power recovery experiment handoff

The implementation in this branch includes cumulative timing budgets, bounded intermediate exploration, native checkpoint validation, and restoration of the best final-feasible tree. The new default-budget version has passed implementation tests. One bounded Basilisk smoke test is documented below; its recovery completed but external reporting timed out. The new version has **not yet been benchmarked on Croc**. Do not attribute the strict V5 measurements below to the new version.

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

The new version adds a small watched set of existing slew-violating pins to catch that failure during tentative moves. No electrical tolerance was relaxed. Its one bounded smoke test is documented below; broader QoR and scaling evaluation remains pending.

## Resume work

1. Inspect the batch05 smoke-test result below before extending Basilisk evaluation. Omit the optional `report_clock_skew` from time-limited runs. Use the original DEF plus original base and post-CTS SDC recipe. Do not substitute the historical ODB (schema mismatch) or cached ODB/exported-SDC input (a previously observed hold/clock roundtrip discrepancy).
2. Compare its reports against unchanged batch04 baseline reports using the printed final allowance. Check each scene/group separately; the new policy is command-start WNS minus allowance, not the previous zero-slack floor unless an explicit margin supplies it.
3. Run Croc with the automatic allowance, explicit zero, and at least one larger override. Recompute power, clock-driver area, insertion, spread, setup/hold and runtime. New timing-policy results cannot be assigned the old retained-savings percentage without matching the old bounds.
4. Reload saved successful results in fresh processes with original constraints before claiming final QoR.

Local, uncommitted artifacts remain in the original workspace under `evaluation/`: the generic Croc harness in `clock_recovery/`, public summaries in `recovery_batch_results/`, frozen binaries and source snapshots in `private/clock_recovery_batch_01/`, and Basilisk runs including batch05 in `private/scaling_basilisk_01/`. Raw design databases, libraries, netlists and binaries are deliberately not part of this branch.

The independent Astra review found no blocking implementation issue in the final core at SHA-256 `dfd7b8274871b1aab1e98b1213e9e2b1393799902973ba2628cb31aac7fd022d`. The base OpenROAD revision is `c1b9cf06435aaae29aeef71c626920e05119474d`; Croc input revision is `71abd392bec14930292007cfb3a2cda2901bceeb`. No OpenSTA submodule files were changed.

## Verification at handoff

The final OpenROAD build passed with `--jobs=6`. All 48 C++ recovery tests, the registered Tcl command test, README checks, and the complete repository lint set passed. The pinned `//:tidy_cc` check made no changes. Independent Astra review found no remaining code or documentation blockers. The final local binary SHA-256 is `0c0807f6439431742c4f4ae33c96e36dd34750839dc4fcea189c309073f11b2a`. After the request to stop, the user explicitly authorized one additional Basilisk test with a five-minute total ceiling. No retries or further EDA runs were launched.

## Default-budget Basilisk smoke test

The user-authorized smoke test used the final built binary recorded above, four threads, a 230 s soft recovery budget and a 285 s hard process-group timeout. The original DEF/SDC inputs matched the previous baseline. The inferred unit-buffer intrinsic delay was 26.3423068 ps, giving final and intermediate allowances of 13.1711534 ps and 26.3423068 ps, respectively. The reference was evaluated at zero input slew and zero external load.

**Recovery completed, but the overall experiment is incomplete.** The hard timeout killed the process during the optional `report_clock_skew` call in final reporting. Global power and worst setup/hold reports exist; per-scene reports, after-area data, and recovered ODB/DEF/SDC were not written. No fresh-process validation is possible from this run. No process remained after the cutoff.

| Metric | Result |
|---|---:|
| Native recovery elapsed | 209.868 s |
| Initial / final clock power | 134.897307 / 134.897307 mW |
| Net changed buffers | 0 |
| Initial / final setup slack | −2.811036348 / −2.811036348 ns |
| Initial / final hold slack | −12.614426613 / −12.614426613 ns |
| Tentative trials | 79 |
| Temporary exploratory changes | 51 |
| Local electrical rejections | 28 |
| Watched existing slew-violating pins | 203 |
| Native whole-design power calls | 3 |
| Whole-design power evaluation time | 98.756 s |
| Cached clock-sink checks / time | 51 / 17.427 s |
| Local/watched electrical checks / time | 79 / 0.043 s |

The single native exploratory checkpoint increased clock power to 135.967195 mW, about 1.070 mW above baseline. It did not replace the saved best state. Recovery restored all 51 exploratory changes and returned baseline power and timing; there was **no power improvement**. This exercises exploration and best-state restoration but does not establish recovery quality on Basilisk.

Before recovery, the design had 5,421 unique core clock drivers with 244,924.0416 um² clock-driver area and 13,842,102.9600 um² standard-cell area. After-area data and peak RSS are unavailable because reporting was interrupted; do not infer them as measured results.

Next priorities are improving proposal power estimates (smaller area did not imply lower native clock power in this checkpoint) and reducing whole-design power-evaluation cost. Three native power calls consumed nearly half the recovery runtime. The optional skew report should be excluded from the short recovery smoke test rather than extending its deadline.
