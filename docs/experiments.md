# Experiments / checkpoint registry

Append-only. One entry per training run, whether it worked or not.

## Format

```
## {run_name}@{step} — YYYY-MM-DD
dataset: {repo_id}
lerobot version/commit: ...
result: one-line summary (offline error, eval success rate, or "failed: why")
```
