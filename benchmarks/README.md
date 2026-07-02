# Benchmark data

`results/results.csv` is the committed dataset behind
[docs/benchmarks.md](../docs/benchmarks.md) and the plots in `docs/media/`.
One row per (backend, world size, repeat) run; the `valid` column records whether the
returned plan survived replay through the plan validator.

Regenerate on your machine:

```bash
python3 -m washbot_planning.cli benchmark \
    --rooms 2,4,8,12,16,24,32 --repeats 3 --timeout 60 \
    --out benchmarks/results/results.csv

python3 -m washbot_planning.cli plot-benchmark \
    --csv benchmarks/results/results.csv --out-dir docs/media
```

Backends are auto-detected: on a machine with POPF installed
(`sudo apt install ros-<distro>-popf`), add `--backends internal-gbfs,internal-astar,popf`
to include it. Worlds are generated with a fixed seed, so runs across machines solve
identical problems; absolute times are machine-specific (the committed data's
environment is documented in docs/benchmarks.md).
