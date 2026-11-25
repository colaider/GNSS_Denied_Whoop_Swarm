# train - learns the z residual (what the throttle model misses) from training_z.csv:
#   input  x = [z, vz, tilt, vxy]  (normalised), output y = residualZ (m/s^2)
# Writes weights.npz (W, b, theta, input mean / std) and results.png. Then: python makeHeader.py
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from Network import Network

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "training_z.csv")
INPUTS = ["z", "vz", "tilt", "vxy"]
NEURONS = [4]                           # hidden layer sizes to try (both layers the same); was [3, 5, 8, 12]


def loadData(path=DATA):
    df = pd.read_csv(path)
    df["tilt"] = np.cos(np.radians(df["attRoll"])) * np.cos(np.radians(df["attPitch"]))
    df["vxy"] = np.hypot(df["vx"], df["vy"])
    return df.dropna(subset=INPUTS + ["residualZ"])


def split(df, seed=0):                  # 60 / 20 / 20 by flight segment: neighbouring rows stay together
    segs = np.random.default_rng(seed).permutation(df["segment"].unique())
    a, b = int(0.6 * len(segs)), int(0.8 * len(segs))
    return [df[df["segment"].isin(s)] for s in (segs[:a], segs[a:b], segs[b:])]


def addNoise(X, Y, sigma=0.05, copies=1, seed=0):   # X normalised: sigma in standard deviations
    rng = np.random.default_rng(seed)
    Xs = [X] + [X + rng.normal(0, sigma, X.shape) for _ in range(copies)]
    return np.concatenate(Xs), np.concatenate([Y] * (copies + 1))


def predict(net, theta, X):             # all rows at once (the same as forward, row by row)
    H = X
    for W, b in zip(net.W, net.b):
        H = np.tanh(H @ W.T + b)
    return np.column_stack([H, np.ones(len(H))]) @ theta


def mse(net, theta, X, Y):
    return float(np.mean((Y - predict(net, theta, X)) ** 2))


def fit(sizes, X, Y, lr, epochs, seed=0):
    net, theta = Network(sizes, seed), np.zeros(sizes[-1] + 1)
    rng = np.random.default_rng(seed)
    for epoch in range(epochs):
        order = rng.permutation(len(X))                   # shuffled every epoch
        theta = net.train(theta, X[order], Y[order], lr=lr, epochs=1)
    return net, theta


def runTraining(lr=0.001, epochs=20, noise=0.05, seed=0):
    tr, val, test = split(loadData(), seed)
    mean, std = tr[INPUTS].mean().to_numpy(), tr[INPUTS].std().to_numpy()
    X = [(d[INPUTS].to_numpy() - mean) / std for d in (tr, val, test)]
    Y = [d["residualZ"].to_numpy() for d in (tr, val, test)]
    Xn, Yn = addNoise(X[0], Y[0], noise, seed=seed)
    print("rows train / val / test:", [len(y) for y in Y])

    results = []                                          # (neurons, val mse, net, theta)
    for n in NEURONS:
        net, theta = fit((len(INPUTS), n, n), Xn, Yn, lr, epochs, seed)
        results.append((n, mse(net, theta, X[1], Y[1]), net, theta))
        print(f"{n:3d} neurons: val {results[-1][1]:.4f}")
    n, valMse, net, theta = min(results, key=lambda r: r[1])

    testMse = mse(net, theta, X[2], Y[2])                 # touched once, at the very end
    base = float(np.mean((Y[2] - Y[0].mean()) ** 2))      # "always the mean"
    print(f"best: {n} neurons, test {testMse:.4f}, baseline {base:.4f}, R2 {1 - testMse / base:.3f}")
    np.savez(os.path.join(HERE, "weights.npz"), W1=net.W[0], b1=net.b[0], W2=net.W[1], b2=net.b[1], theta=theta,
             mean=mean, std=std, inputs=np.array(INPUTS), testMse=testMse, baseMse=base)
    return net, theta, results, base, (test, X[2], Y[2])


def plotResults(net, theta, results, base, testSet):
    val, Xval, Yval = testSet
    yHat = predict(net, theta, Xval)
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))

    a = ax[0, 0]                                          # which size won
    a.plot([r[0] for r in results], [r[1] for r in results], "o-", label="validation")
    a.axhline(base, color="gray", ls="--", label="baseline: always the mean")
    a.set(xlabel="neurons per hidden layer", ylabel="MSE (m/s^2)^2", title="Network size")
    a.legend()

    a = ax[0, 1]                                          # every validation row
    a.scatter(Yval, yHat, s=2, alpha=0.2)
    lim = np.percentile(np.abs(Yval), 99)
    a.plot([-lim, lim], [-lim, lim], "k--", lw=1)
    a.set(xlim=(-lim, lim), ylim=(-lim, lim), xlabel="true residual", ylabel="predicted", title="Test rows")

    a = ax[1, 0]                                          # what it learned over height
    bins = np.arange(0.0, val["z"].max() + 0.1, 0.1)
    idx = np.digitize(val["z"], bins)
    zs = [bins[i - 1] + 0.05 for i in np.unique(idx)]
    a.plot(zs, [Yval[idx == i].mean() for i in np.unique(idx)], "o-", label="true (mean)")
    a.plot(zs, [yHat[idx == i].mean() for i in np.unique(idx)], "s-", label="predicted (mean)")
    a.set(xlabel="height z (m)", ylabel="residual (m/s^2)", title="Residual over height")
    a.legend()

    a = ax[1, 1]                                          # one flight piece in time
    seg = val["segment"].value_counts().index[0]
    m = (val["segment"] == seg).to_numpy()
    a.plot(val["t"].to_numpy()[m], Yval[m], label="true")
    a.plot(val["t"].to_numpy()[m], yHat[m], label="predicted")
    a.set(xlabel="t (s)", ylabel="residual (m/s^2)", title=f"Longest test segment ({seg})")
    a.legend()

    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "results.png"), dpi=110)
    plt.show()


def main():
    net, theta, results, base, testSet = runTraining()
    plotResults(net, theta, results, base, testSet)


if __name__ == "__main__":
    main()
