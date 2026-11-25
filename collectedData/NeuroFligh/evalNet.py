# evalNet - how good is the residual network really, what carries it, and does more data help?
#   1) the same 4-neuron net over SEEDS different 60/20/20 splits: test R2 with its spread
#   2) input removal: without vz / vxy / tilt / z, same splits: how much R2 each input is worth
#   3) data size: only 25 / 50 / 75 / 100 % of the training segments: the learning curve
#   4) the error per height band (all splits together)
# Writes evalNet.png. Runs every training in parallel (each one is train.fit, the per-sample SGD).
#   python evalNet.py
import os
from multiprocessing import Pool

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import train

HERE = os.path.dirname(os.path.abspath(__file__))
SEEDS = [0, 1, 2, 3, 4]
NEURONS, LR, EPOCHS, NOISE = 4, 0.001, 20, 0.05
FRACTIONS = [0.25, 0.5, 0.75, 1.0]
DROPS = [None, "vz", "vxy", "tilt", "z"]
BANDS = [0.0, 0.3, 0.4, 0.5, 0.7, 0.9, 1.2]


def job(args):                          # one training -> (kind, key, seed, test MSE, baseline, test z, errors)
    kind, key, seed = args
    tr, val, test = train.split(train.loadData(), seed)
    if kind == "frac" and key < 1.0:    # a part of the training flights (whole segments)
        segs = np.random.default_rng(100 + seed).permutation(tr["segment"].unique())
        tr = tr[tr["segment"].isin(segs[:max(1, int(key * len(segs)))])]
    inputs = [k for k in train.INPUTS if k != key] if kind == "drop" else train.INPUTS
    mean, std = tr[inputs].mean().to_numpy(), tr[inputs].std().to_numpy()
    X, Xt = (tr[inputs].to_numpy() - mean) / std, (test[inputs].to_numpy() - mean) / std
    Y, Yt = tr["residualZ"].to_numpy(), test["residualZ"].to_numpy()
    Xn, Yn = train.addNoise(X, Y, NOISE, seed=seed)
    net, theta = train.fit((len(inputs), NEURONS, NEURONS), Xn, Yn, LR, EPOCHS, seed)
    err = Yt - train.predict(net, theta, Xt)
    base = float(np.mean((Yt - Y.mean()) ** 2))
    return kind, key, seed, float(np.mean(err ** 2)), base, len(tr), test["z"].to_numpy(), err, Yt - Y.mean()


def r2(rows):
    v = np.array([1 - m / b for (_, _, _, m, b, *_) in rows])
    return v.mean(), v.std()


def main():
    jobs = [("drop", d, s) for d in DROPS for s in SEEDS] + \
           [("frac", f, s) for f in FRACTIONS if f < 1.0 for s in SEEDS]
    with Pool(min(len(jobs), os.cpu_count() - 2)) as pool:
        res = pool.map(job, jobs)
    full = [r for r in res if r[0] == "drop" and r[1] is None]

    m, s = r2(full)
    print(f"all inputs, {len(SEEDS)} splits: test R2 {m:.3f} +- {s:.3f}  ({', '.join(f'{1 - r[3] / r[4]:.2f}' for r in full)})")
    print(f"  test rms error: network {np.sqrt(np.mean([r[3] for r in full])):.3f} m/s^2, "
          f"constant (training mean) {np.sqrt(np.mean([r[4] for r in full])):.3f} m/s^2")
    print("without one input (R2, drop vs all):")
    for d in DROPS[1:]:
        rows = [r for r in res if r[0] == "drop" and r[1] == d]
        md, sd = r2(rows)
        print(f"  no {d:5s} R2 {md:.3f} +- {sd:.3f}   ({md - m:+.3f})")
    print("learning curve (share of the training flights -> rows, R2):")
    curve = []
    for f in FRACTIONS:
        rows = full if f == 1.0 else [r for r in res if r[0] == "frac" and r[1] == f]
        mf, sf = r2(rows)
        n = np.mean([r[5] for r in rows])
        curve.append((n / 50 / 60, mf, sf))
        print(f"  {f * 100:3.0f} %  {n:7.0f} rows ({n / 50 / 60:4.1f} min)  R2 {mf:.3f} +- {sf:.3f}")

    z = np.concatenate([r[6] for r in full])
    e = np.concatenate([r[7] for r in full])
    e0 = np.concatenate([r[8] for r in full])
    print("per height band (all test rows of all splits): seconds, net MSE, baseline MSE, R2, mean error")
    band = []
    for lo, hi in zip(BANDS[:-1], BANDS[1:]):
        k = (z >= lo) & (z < hi)
        if k.sum() < 50:
            print(f"  {lo:.1f}-{hi:.1f} m  too few rows ({k.sum()})")
            continue
        mse, b = np.mean(e[k] ** 2), np.mean(e0[k] ** 2)
        band.append((lo, hi, mse, b, e[k].mean()))
        print(f"  {lo:.1f}-{hi:.1f} m  {k.sum() / 50 / len(SEEDS):5.0f} s  {mse:.3f}  {b:.3f}  R2 {1 - mse / b:+.2f}  bias {e[k].mean():+.2f}")

    fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
    a = ax[0]
    a.errorbar([c[0] for c in curve], [c[1] for c in curve], [c[2] for c in curve], fmt="o-", capsize=4)
    a.set(xlabel="training data (min)", ylabel="test R2", title="Learning curve (5 splits)")
    a = ax[1]
    names = ["all"] + [f"no {d}" for d in DROPS[1:]]
    vals = [r2([r for r in res if r[0] == "drop" and r[1] == d]) for d in DROPS]
    a.bar(names, [v[0] for v in vals], yerr=[v[1] for v in vals], capsize=4)
    a.set(ylabel="test R2", title="Input removal")
    a = ax[2]
    x = np.arange(len(band))
    a.bar(x - 0.2, [b[3] for b in band], 0.4, label="baseline (mean)")
    a.bar(x + 0.2, [b[2] for b in band], 0.4, label="network")
    a.set_xticks(x, [f"{b[0]:.1f}-{b[1]:.1f}" for b in band])
    a.set(xlabel="height (m)", ylabel="test MSE", title="Error per height")
    a.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "evalNet.png"), dpi=100)


if __name__ == "__main__":
    main()
