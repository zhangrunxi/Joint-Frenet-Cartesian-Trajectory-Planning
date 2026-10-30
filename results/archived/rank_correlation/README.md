# Rank correlation between solve time and completion rate

| Task | Configurations | Spearman | Kendall |
| --- | ---: | ---: | ---: |
| General (`general.csv`) | 19 | -0.3062 | -0.2622 |
| Overtaking (`overtake.csv`) | 19 | -0.2180 | -0.1330 |

The correlation is negative: the ordering by solve time does not predict the
ordering by completion rate, and if anything the slower configurations
complete slightly more cases.

A rank correlation is only meaningful together with the choices that produced
it, so they are written into each CSV and are the following:

- curved-road scenarios only;
- solve time = median of `mean_solve_wall_ms` over the **successful** cases;
- ties broken with the average rank (the `scipy` default).

An earlier figure of -0.34 averaged the solve time over all cases, failures
included. It is superseded by the values above.

## Re-running it

The per-case records are archived with the benchmark runs, so the numbers can
be recomputed from the archive alone:

```bash
.venv/bin/python scripts/measure_rank_correlation.py \
    --input results/archived/random_benchmark --output /tmp/general.csv
.venv/bin/python scripts/measure_rank_correlation.py \
    --input results/archived/random_benchmark_overtake --output /tmp/overtake.csv
```
