# Benchmark notes

## Command

```bash
luajit ./wenk benchmark bench=40000
```

## Sample output (2026-03-02)

```text
Benchmark iterations: 40000
max_distance x2: 0.092640s (431779 ops/s) sum=0.000
max_distance_delta: 0.042977s (930730 ops/s) sum=0.000
speedup: 2.16x
```

## Notes

- The benchmark mode compares the previous pinch-distance update strategy (`max_distance` called twice per update) against the new single-pass delta strategy (`max_distance_delta`).
- Results are CPU and environment dependent; rerun on target hardware for final numbers.
