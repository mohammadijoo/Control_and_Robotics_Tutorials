\
# Chapter20_Lesson5.py
# AMR Capstone final demo evaluation + research-style summary (Python)

import math
import random

# Each run: [success, time_s, path_m, ref_m, rms_xy_m, clearance_m, collisions, latency_ms, energy_wh, map_iou]
baseline = [
    [1,118.2,28.1,22.0,0.19,0.24,1,31.2,24.8,0.72],
    [1,104.3,24.9,20.0,0.17,0.28,0,28.7,20.1,0.79],
    [0,138.8,30.4,23.0,0.31,0.12,2,39.1,28.5,0.58],
    [1,96.5,22.7,19.0,0.13,0.35,0,24.6,18.4,0.82],
    [1,110.1,26.8,21.0,0.22,0.21,1,33.8,23.7,0.74],
]
proposed = [
    [1,91.4,24.0,22.0,0.11,0.31,0,22.3,21.2,0.84],
    [1,86.0,21.8,20.0,0.10,0.34,0,20.4,18.8,0.87],
    [1,104.2,25.6,23.0,0.16,0.22,1,24.9,22.1,0.76],
    [1,80.9,20.5,19.0,0.09,0.39,0,18.9,17.3,0.90],
    [1,89.1,23.5,21.0,0.14,0.27,0,21.8,19.6,0.83],
]

def mean(xs):
    return sum(xs) / len(xs)

def std(xs):
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))

def path_eff(run):
    return run[3] / run[2]

def energy_per_m(run):
    return run[8] / run[2]

def aggregate(runs):
    return {
        "success_rate": mean([r[0] for r in runs]),
        "collision_free_rate": mean([1.0 if r[6] == 0 else 0.0 for r in runs]),
        "time_s": mean([r[1] for r in runs]),
        "path_eff": mean([path_eff(r) for r in runs]),
        "rms_xy_m": mean([r[4] for r in runs]),
        "clearance_m": mean([r[5] for r in runs]),
        "latency_ms": mean([r[7] for r in runs]),
        "energy_wh_per_m": mean([energy_per_m(r) for r in runs]),
        "map_iou": mean([r[9] for r in runs]),
    }

def paired_ci(prop_vals, base_vals):
    d = [p - b for p, b in zip(prop_vals, base_vals)]
    m = mean(d)
    s = std(d)
    z = 1.959963984540054
    h = z * s / math.sqrt(len(d))
    return (m, m - h, m + h)

def bootstrap_ci(vals, B=2000, seed=0):
    rng = random.Random(seed)
    boots = []
    n = len(vals)
    for _ in range(B):
        sample = [vals[rng.randrange(n)] for _ in range(n)]
        boots.append(mean(sample))
    boots.sort()
    return boots[int(0.025 * B)], boots[int(0.975 * B) - 1]

def zscore_features(run):
    # higher-is-better transformed features
    return [
        run[0],                    # success
        path_eff(run),             # path efficiency
        -run[1],                   # time
        -run[4],                   # rms_xy
        run[5],                    # clearance
        -run[6],                   # collisions
        -run[7],                   # latency
        -energy_per_m(run),        # energy per m
        run[9],                    # map IoU
    ]

def composite_scores(base_runs, prop_runs):
    all_runs = base_runs + prop_runs
    F = [zscore_features(r) for r in all_runs]
    cols = list(zip(*F))
    mu = [mean(list(c)) for c in cols]
    sd = [max(std(list(c)), 1e-12) for c in cols]
    w = [0.25, 0.10, 0.12, 0.05, 0.10, 0.10, 0.08, 0.05, 0.15]

    def score(run):
        f = zscore_features(run)
        return sum(wi * ((fi - mi) / si) for wi, fi, mi, si in zip(w, f, mu, sd))
    return [score(r) for r in base_runs], [score(r) for r in prop_runs]

A = aggregate(baseline)
B = aggregate(proposed)
time_ci = paired_ci([r[1] for r in proposed], [r[1] for r in baseline])
iou_ci = paired_ci([r[9] for r in proposed], [r[9] for r in baseline])
succ_boot = bootstrap_ci([r[0] for r in proposed], seed=1)
b_scores, p_scores = composite_scores(baseline, proposed)
score_ci = paired_ci(p_scores, b_scores)

print("Baseline:", A)
print("Proposed:", B)
print("Paired delta mission time (s):", time_ci)
print("Paired delta map IoU:", iou_ci)
print("Bootstrap CI for proposed success rate:", succ_boot)
print("Paired delta composite score:", score_ci)

print("\nMarkdown row summary:")
for k in ["success_rate", "time_s", "path_eff", "rms_xy_m", "clearance_m", "latency_ms", "energy_wh_per_m", "map_iou"]:
    print(f"{k}: baseline={A[k]:.4f}, proposed={B[k]:.4f}, delta={B[k]-A[k]:+.4f}")
