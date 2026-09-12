# Recording session log

Append-only. One entry per dataset recording session.

## Format

```
## {dataset_repo_id} — YYYY-MM-DD
backend: sim | real
episodes: N
notes: anything unusual (lighting, calibration, aborted episodes)
```

## EdgeVLA/so101_pickcube_sim_v1_20ep — 2026-09-12
backend: sim
episodes: 20
notes: First Phase 2 recording (issue #8), via `scripts/record_dataset.sh --episodes 20 --repo-id EdgeVLA/so101_pickcube_sim_v1_20ep --push`. All 20 attempts succeeded on the first try (0 discarded episodes) — plausible at the controller's measured 94/100 rate. 56320 frames total (2816/episode), `validate_recorded_dataset()` passed. Recorded with an unseeded backend (`MuJoCoBackend()`, OS entropy), unlike the fixed seed=42 used for all grasp-reliability measurements in `docs/decisions.md`.
