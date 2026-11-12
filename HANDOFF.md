# Clock-power recovery handoff

## Start here

The implementation builds, passes 48 C++ tests and the registered Tcl test, and has passed independent Astra review. **Efficient power recovery on Basilisk is not achieved yet.** Its latest short run explored a higher-power tree and correctly returned the baseline. The new cumulative-budget version has not been evaluated on Croc.

The branch is `feature/clock-power-recovery` on `git@github.com:phsauter/openroad-dev.git`.

- `1fc472ebc4`: implementation, tests, command documentation and initial experiment handoff.
- `c98e44ed3d`: bounded Basilisk smoke-test results.
- [Detailed experiment results](docs/clock_power_recovery_experiments.md).
- [Command documentation](src/rsz/README.md).

The branch is checked out in the **greedy worktree**, not the outer workspace checkout:

```text
Workspace: /home/phsauter/repos/openroad-dev
Code:      /home/phsauter/repos/openroad-dev/evaluation/clock_recovery/greedy
```

The outer checkout remains on `master` at `c1b9cf06435aaae29aeef71c626920e05119474d`. Other experimental worktrees (`best`, `dynamic`, `subtree`) remain under `evaluation/clock_recovery/`. Do not overwrite their work or confuse their binaries with the selected implementation.

## Objective and user decisions

CTS often chose the largest buffers from the supplied list. The project started by comparing buffer lists on upstream Croc, then implemented fixed-topology buffer downsizing. The current focus is preserving useful power recovery while reducing runtime enough for much larger designs.

The user explicitly approved:

- Greedy/batched recovery and comparisons against serial recovery, Croc's default CTS list, and a manually selected list.
- Technology-scaled timing regression: default final allowance is half the intrinsic delay of a unit-sized reference buffer, with a numeric override.
- **Each timing metric independently** gets a cumulative allowance across all changes, measured from command-start timing. Do not sum the four metrics or replenish the allowance after each move or checkpoint.
- Intermediate exploration beyond final timing limits, because subsequent changes can recover timing. Save a final-feasible best tree and restore it if exploration does not improve it.
- Investigating a clock-only STA view or parallel trials if needed. Neither is implemented in this branch.

The final overnight instruction was to stop expanding scope, branch/commit/push, then run exactly one additional Basilisk test lasting no more than five minutes. That test is finished; no retries or further experiments were launched. No EDA jobs remain running.

## Implemented behavior

Use `recover_clock_power -strategy bottom_up_batch` for bounded recovery. Serial `bottom_up` and `top_down` remain available for reference.

Important batch options:

| Option | Meaning |
|---|---|
| `-max_timing_regression time` | Final cumulative allowance; omitted means inferred half-buffer intrinsic delay |
| `-max_intermediate_timing_regression time` | Exploration allowance; defaults to twice the final allowance and must be at least the final allowance |
| `-max_checkpoints integer` | Full-validation budget, including baseline and reserved final validation; default 6 |
| `-max_seconds seconds` | Soft recovery deadline; native calls cannot be interrupted; default 300 |
| `-max_passes integer` | Bounded exploration waves; default 5 |
| `-max_trials integer` | Optional bound on attempted individual replacements |
| `-max_latency time`, `-max_skew time` | Additional absolute final clock bounds |
| `-setup_margin time`, `-hold_margin time` | Additional final slack floors only when explicitly supplied in batch mode |

Time-valued options use STA's current user time units; `-max_seconds` is wall-clock seconds. Inspect `report_units` before choosing numeric overrides.

For each clock/scene/source-edge/sink-edge group, final insertion and spread cannot exceed their initial values plus the final allowance, also respecting any supplied absolute limits. Per-scene setup and hold WNS cannot fall below their initial values minus the allowance. Explicit margins add the legacy floor `min(initial_WNS, margin)`. Intermediate absolute bounds are relaxed by the difference between intermediate and final allowances.

Explicit zero means literal zero regression from starting timing. This differs from strict V5, which could spend positive slack down to zero. Do not compare policies as if their constraints were identical.

Automatic characterization searches all compatible ordinary library buffers, independently of `-buffers` restrictions and `dont_use` insertion policy. It chooses a deterministic minimum-area reference and evaluates zero-input-slew, zero-external-load delay across configured scenes, min/max models and transitions. All relevant values must be finite and positive; otherwise automatic relaxation falls back to zero with a warning. The chosen reference and fixed allowances are logged once.

### Acceptance and restoration

1. Proposals use Liberty delay/slew estimates with fixed wire-delay assumptions and baseline native instance-power ranking. Smaller area is a heuristic, not proof of lower power.
2. Each tentative resize refreshes native timing and checks all-scene WNS, local/watched electrical limits, and exact cached clock-sink envelopes.
3. Local electrical checks include the resized buffer, upstream input-net drivers, and pins with pre-existing slew violations. Full checkpoints still check the whole design's slew/capacitance margins.
4. A full native checkpoint may advance exploration within intermediate timing limits even if power increases. Only lower-power checkpoints satisfying final bounds replace the saved best state.
5. Before crossing from a final-feasible nonempty prefix into off-final exploration, the candidate is undone and the prefix is fully certified. An empty prefix can cross immediately, avoiding starvation of cooperative moves.
6. Before return, the exploratory history suffix is reversed, best masters are checked, and a reserved full native final certificate is required. Exception recovery retains enough history to restore invocation-start state without consuming a caller's enclosing ECO.

Slew and capacitance tolerances were not loosened. Topology, connectivity, origins, protected objects, compatible models and physical footprints remain guarded. Generated-clock protection includes the generator's master-clock ancestry; normalized local insertion still uses the unexpanded generated-clock root.

## Source and verification

Primary implementation: [RecoverClockPower.cc](src/rsz/src/RecoverClockPower.cc). Useful entry points include `clock_limits`, `slack_floor`, `collect_electrical`, cached clock checks, watched slew pins, and the saved `best_boundary` in accepted history.

Bindings and tests:

- `src/rsz/include/rsz/Resizer.hh`
- `src/rsz/src/Resizer.i` and `Resizer.tcl`
- `src/rsz/test/cpp/TestRecoverClockPower.cc` and its Verilog fixtures
- `src/rsz/test/recover_clock_power.tcl`
- Both module/test Bazel `BUILD` files

Final core SHA-256: `dfd7b8274871b1aab1e98b1213e9e2b1393799902973ba2628cb31aac7fd022d`.

Final binary SHA-256: `0c0807f6439431742c4f4ae33c96e36dd34750839dc4fcea189c309073f11b2a`.

Build, all 48 C++ cases, registered Tcl, README checks, complete lint, pinned formatting, and independent Astra code/documentation review passed. Tests include cumulative budgets, independent metrics, generated ancestry, boundary saving, actual off-final-to-final recovery, best restoration, caller ECO preservation, and injected restoration failure.

```bash
cd /home/phsauter/repos/openroad-dev/evaluation/clock_recovery/greedy
bazelisk build --jobs=6 //:openroad
bazelisk test --jobs=6 --test_output=errors //src/rsz/test:TestRecoverClockPower //src/rsz/test:recover_clock_power-tcl_test //src/rsz/test:rsz_readme_msgs_check //:lint_test
```

No OpenSTA submodule files were modified. Ask the user before editing `src/sta/`. Keep future fixes in OpenROAD where possible. Use pinned Bazel formatters and signed-off commits.

## Latest results and their limits

### Croc

The last measured version is **strict V5**, not the current budget version. With a roughly two-minute recovery budget, its all-buffer/default/manual trees reached 1.5083/1.3826/1.5910 mW from 3.9586/2.3704/2.2449 mW. This retained 88.0%/86.5%/75.4% of converged serial savings under the old relaxed limits. The manual case misses the initial 80% research target.

Those runs passed independent structural, physical, timing and electrical report checks. V4 had identical measured power/timing and passed fresh-process reload checks; V5 was not separately reloaded. See the detailed results document for area, insertion, spread, setup and hold values.

The converged serial reference took approximately 2,654/3,912/1,614 seconds of recovery for the all/default/manual trees. It reached 1.1747/1.2287/1.3773 mW under the old 2 ns insertion, 1 ns spread and scene slack-floor policy. These are references, not results of the current budget version.

### Basilisk: one bounded current-version smoke test

- Inputs: 955,849 instances, 1,046,697 nets including special nets, two scenes, eight clocks.
- Four threads; 230 s soft recovery budget; 285 s hard process-group timeout, leaving status-writing time inside the five-minute ceiling.
- Automatic final/intermediate allowances: **13.1711534/26.3423068 ps**.
- Recovery finished in **209.868 s** with **zero net changes and no power improvement**.
- A 51-change exploratory checkpoint raised native clock power from 134.897307 to 135.967195 mW. It was not saved as best; all 51 changes were restored.
- Final native/global reports returned baseline clock power, setup −2.811036348 ns and hold −12.614426613 ns.
- The entire experiment is **incomplete**: the hard cutoff occurred during optional `report_clock_skew`. Per-scene final reports, recovered databases/netlists, after-area data and peak RSS were not produced. Do not claim fresh-process validation or successful QoR recovery.
- Three whole-design power evaluations took 98.756 s, nearly half the recovery runtime. Cached clock checks took 17.427 s. Local/watched electrical checks took 0.043 s over 79 trials, watching 203 existing slew-violating pins.

This establishes that exploration, final certification and baseline restoration execute on the large design. It does not establish useful Basilisk power recovery or convergence.

## Local artifacts and input traps

Paths below are relative to the **outer workspace**, not this worktree. Most are local/uncommitted and are not available from a fresh clone of the pushed branch.

| Path | Contents |
|---|---|
| `evaluation/clock_recovery/run_benchmark.py`, `benchmark.tcl` | Croc runner, native reports and independent invariants; supports new timing-budget flags |
| `evaluation/recovery_batch_results/` | V1–V5 summaries and V4 original-SDC reload checks |
| `evaluation/recovery_deep_results/` | Converged serial comparisons, structural study and electrical checks |
| `evaluation/recovery_runtime_results/` | Earlier serial profiling and scaling study |
| `evaluation/private/clock_recovery_batch_01/` | Frozen source/binary snapshots; **`bin/batch_v6budget_final` is the final tested binary** |
| `evaluation/private/scaling_basilisk_01/def_import01/` | Original DEF, original base/post-CTS SDCs, technology setup and preflight |
| `evaluation/private/scaling_basilisk_01/batch04/` | Unchanged strict-V5 Basilisk baseline reports |
| `evaluation/private/scaling_basilisk_01/batch05/` | Current smoke-test runner, Tcl, status JSON, log and partial reports |
| `evaluation/private/recovery/greedy_*_batch01_v5strict/` | Last measured Croc runs |
| `evaluation/private/all_ascending/`, `baseline/`, `single8/` | Frozen Croc CTS ZIP inputs |

Do not use `bin/batch_v6budget` as the final binary: it is an earlier provisional copy. Check hashes before launching a benchmark.

Basilisk input rules:

- Use original DEF plus original base and post-CTS constraints from `def_import01`.
- The historical ODB cannot be loaded by this build: mask/shift schema mismatch.
- A cached ODB/exported-SDC recipe previously changed hold/clock timing. It is not interchangeable with the verified original recipe.
- Keep both clock and signal placement RC on `Metal4`, apply the original `dont_use` list, and estimate placement parasitics as the prepared Tcl does.
- Existing missing-wireload and temporary false-path warnings are known. Earlier constraint inspection verified the post-CTS script removes the temporary Hyperbus exceptions; do not casually change constraints to silence warnings.
- **Remove optional `report_clock_skew` before another short run.** Preserve the old batch05 artifacts and create a new run directory; do not overwrite provenance.

Croc input revision: `71abd392bec14930292007cfb3a2cda2901bceeb`. Extract the exact `03_croc.cts.zip` input; avoid flow checkpoint globs that can accidentally select report files.

## Recommended next work

1. Improve proposal power estimates. The measured higher-power checkpoint shows that area and timing estimates alone are insufficient. Investigate Liberty-based local power estimates using frozen baseline activity, with full native power checks retained for acceptance. Explicitly account for activity/slew clipping; do not label a surrogate exact.
2. Reduce whole-design power-evaluation cost before increasing trial counts. Public instance-power queries still populate whole-design caches. Native timing is already incremental, but invalidated data cones and global power work remain expensive.
3. Evaluate the current version on Croc with automatic allowance, zero, and a larger override. Report power, clock-driver area, insertion, spread, setup/hold, electrical changes and runtime. Match constraint policies before comparing retained serial savings.
4. Run another declared bounded Basilisk test only after adjusting the reporting harness and choosing a concrete hypothesis. Use a hard process-group timeout; `-max_seconds` alone is soft.
5. Save successful results and reload them in a fresh process with original constraints before claiming final QoR.

A clock-only secondary STA view is feasible through `dbSta::makeBlockSta(dbBlock*)`, with examples in `Resizer::cellWireDelay`. It does not automatically clone scenes, clocks, SDC or parasitics. Preserve real sinks, every load, gates/muxes/inverters, generated-clock source circuitry and objective membership. Full-chip setup/hold and electrical certification remains necessary. This is a larger follow-up, not an implemented feature.

Do not mutate one STA/ECO state concurrently. Independent processes are the safer parallel-trial fallback. Prepared read-only calculations might be parallelizable after a thread-safety audit. Neither approach should precede fixing proposal quality without evidence it addresses the bottleneck.

Use `heavy` only for jobs expected to exceed 30 GB or occupy many threads for hours. These four-thread Basilisk runs previously used about 12.4 GiB, so they were run normally. Do not use unbounded compilation or job parallelism.
