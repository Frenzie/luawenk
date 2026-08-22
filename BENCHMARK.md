# Benchmark notes

## Command

```bash
luajit ./wenk benchmark bench=40000
```

## Sample output (2026-08-21)

```text
Benchmark iterations: 40000
legacy scan x2: 0.036946s (1082661 ops/s) sum=0.000
monitor delta: 0.005715s (6999125 ops/s) sum=0.000
speedup: 6.46x (sums match)
```

## Notes

- The benchmark compares the legacy strategy (two full table scans per update,
  before/after the move) against the incremental max-distance monitor now used
  by `process_update`: an FFI `wenk_point_t` arena plus a cached top-2 pair that
  is updated in place and only rebuilt (O(n^2)) when the move touches the
  tracked pair or slot membership changes.
- Both strategies compute the identical delta sequence; the printed sums are a
  built-in correctness check (`sums match`).
- Results are CPU and environment dependent; rerun on target hardware for final
  numbers.

## Other measurements (bench_ffi.lua, 2026-08-21)

- Incremental monitor vs full table scan at n=5: ~6.0M vs ~1.1M ops/s.
- Staged-guard early exit (stationary finger, dx==dy==0): ~63M checks/s —
  process_update skips the monitor entirely for frames without movement.
- `ev_time_seconds`: tonumber-based conversion remains fastest of the tried
  variants; left as is.
