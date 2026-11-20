"""Generates every data file used by the figures in vla_wm_notes.tex.
All data is synthetic and seeded, so the numbers in the text match the plots.
Run:  ../.venv/bin/python make_data.py   (from VLA_WM_notes/)
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

rng = np.random.default_rng(7)
F = "figs/"


def save(name, cols, header):
    np.savetxt(F + name, np.column_stack(cols), header=header, comments="", fmt="%.5f")


# ---------------------------------------------------------------- 1. attention example
tok = ["pick", "the", "red", "cube"]
X = np.array([[1.0, 0.0], [0.2, 0.2], [0.0, 1.0], [0.6, 0.8]])
Wq = np.array([[2.0, 0.0], [0.0, 2.0]])
Wk = np.eye(2) * 1.5
Q, K = X @ Wq, X @ Wk
S = Q @ K.T / np.sqrt(2)
A = np.exp(S - S.max(1, keepdims=True)); A /= A.sum(1, keepdims=True)
with open(F + "attn.dat", "w") as f:
    f.write("x y w\n")
    for i in range(4):
        for j in range(4):
            f.write(f"{j} {i} {A[i, j]:.4f}\n")
        f.write("\n")
with open(F + "attn_meta.tex", "w") as f:
    for i, n in enumerate("abcd"):
        for j, m in enumerate("abcd"):
            f.write(f"\\def\\att{n}{m}{{{A[i, j]:.2f}}}\n")
            f.write(f"\\def\\sc{n}{m}{{{S[i, j]:.2f}}}\n")
print("attention row 'cube':", np.round(A[3], 3), "scores", np.round(S[3], 3))

# ---------------------------------------------------------------- 2. temperature
logits = np.array([3.0, 2.2, 1.0, 0.3, -0.5])
P = []
for T in (0.5, 1.0, 2.0):
    p = np.exp(logits / T); p /= p.sum(); P.append(p)
    print("T", T, np.round(p, 3))
save("temp.dat", [np.arange(5), *P], "i ta tb tc")

# ---------------------------------------------------------------- 3. bimodal actions
n = 400
side = rng.choice([-1, 1], n)
acts = side * 1.0 + rng.normal(0, 0.18, n)
mean_pred = acts.mean()
# samples from a "generative" policy = here simply a 2-component mixture fitted by EM-lite
mu = np.array([-1.0, 1.0]); sd = np.array([0.3, 0.3]); pi = np.array([0.5, 0.5])
for _ in range(50):
    r = pi * np.exp(-0.5 * ((acts[:, None] - mu) / sd) ** 2) / sd
    r /= r.sum(1, keepdims=True)
    pi = r.mean(0); mu = (r * acts[:, None]).sum(0) / r.sum(0)
    sd = np.sqrt((r * (acts[:, None] - mu) ** 2).sum(0) / r.sum(0))
gen = np.where(rng.random(n) < pi[0], rng.normal(mu[0], sd[0], n), rng.normal(mu[1], sd[1], n))
bins = np.linspace(-2, 2, 33)
h1, _ = np.histogram(acts, bins, density=True)
h2, _ = np.histogram(gen, bins, density=True)
save("bimodal.dat", [bins[:-1], h1, h2], "x demo gen")
with open(F + "bimodal_meta.tex", "w") as f:
    f.write(f"\\def\\bimean{{{mean_pred:.3f}}}\n")
print("bimodal mean", mean_pred, "mixture", mu, sd, pi)

# ---------------------------------------------------------------- 4. diffusion marginals (closed form)
xs = np.linspace(-3, 3, 241)
cols = [xs]
s0 = 0.15
for abar in (1.0, 0.7, 0.3, 0.0):
    m = np.sqrt(abar); v = abar * s0 ** 2 + (1 - abar)
    p = 0.5 * (np.exp(-0.5 * (xs - m) ** 2 / v) + np.exp(-0.5 * (xs + m) ** 2 / v)) / np.sqrt(2 * np.pi * v)
    cols.append(p)
save("diffusion.dat", cols, "x p0 p1 p2 p3")

# ---------------------------------------------------------------- 5. flow matching straight paths
k = 14
x0 = rng.normal(0, 1, k)
x1 = rng.choice([-1, 1], k) + rng.normal(0, 0.12, k)
with open(F + "flow_paths.tex", "w") as f:
    for a, b in zip(x0, x1):
        f.write(f"\\draw[flowp] (axis cs:0,{a:.3f}) -- (axis cs:1,{b:.3f});\n")
        f.write(f"\\fill[cA] (axis cs:0,{a:.3f}) circle (1.5pt);\n")
        f.write(f"\\fill[cB] (axis cs:1,{b:.3f}) circle (1.5pt);\n")

# ---------------------------------------------------------------- 6. covariate shift: BC vs DAgger
# 1D lane keeping: x_{t+1} = x_t + a_t + w_t ; expert a = -0.6 x (keeps the car centred)
def expert(x):
    return -0.6 * x

def fit(states):
    acts_ = expert(states) + rng.normal(0, 0.02, len(states))
    V = np.vander(states, 6)
    c, *_ = np.linalg.lstsq(V, acts_, rcond=None)
    return lambda x: np.polyval(c, np.clip(x, -3, 3))

def rollout(pol, T=40, x0=0.0, noise=0.05, seed=0):
    r = np.random.default_rng(seed)
    x = [x0]
    for _ in range(T):
        x.append(x[-1] + pol(x[-1]) + r.normal(0, noise))
        x[-1] = float(np.clip(x[-1], -3, 3))
    return np.array(x)

# expert data: expert rollouts stay in a narrow band
D = np.concatenate([rollout(expert, 40, 0.0, 0.05, s) for s in range(20)])
bc = fit(D)
# learner meets a gust at t=10 (+0.6) -> leaves the training band
def rollout_gust(pol, seed, T=40):
    r = np.random.default_rng(100 + seed)
    x = [0.0]
    for t in range(T):
        g = 0.6 if t == 10 else 0.0
        x.append(float(np.clip(x[-1] + pol(x[-1]) + g + r.normal(0, 0.05), -3, 3)))
    return np.array(x)

# DAgger: run learner, relabel visited states with the expert, refit
Dd = D.copy(); pol = bc
for it in range(5):
    vis = np.concatenate([rollout_gust(pol, s) for s in range(10)])
    Dd = np.concatenate([Dd, vis]); pol = fit(Dd)
dag = pol
t = np.arange(41)
cols = [t]
for s in range(4):
    cols.append(rollout_gust(bc, s));
for s in range(4):
    cols.append(rollout_gust(dag, s))
cols.append(rollout_gust(expert, 0))
save("covshift.dat", cols, "t b0 b1 b2 b3 d0 d1 d2 d3 e")
with open(F + "covshift_meta.tex", "w") as f:
    f.write(f"\\def\\bandlo{{{D.min():.2f}}}\\def\\bandhi{{{D.max():.2f}}}\n")
print("expert band", D.min(), D.max())
print("bc final", [round(c[-1], 2) for c in cols[1:5]], "dagger final", [round(c[-1], 2) for c in cols[5:9]])

# ---------------------------------------------------------------- 7. compounding model error
th = 0.25
Atrue = 1.0 * np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
Xs = rng.normal(0, 1, (25, 2))
Ys = Xs @ Atrue.T + rng.normal(0, 0.08, (25, 2))
Ahat = np.linalg.lstsq(Xs, Ys, rcond=None)[0].T
one_step = np.sqrt(((Xs @ Ahat.T - Xs @ Atrue.T) ** 2).sum(1)).mean()
H = 40
errs = []
for s in range(200):
    x = rng.normal(0, 1, 2); xt = x.copy(); xh = x.copy(); e = [0.0]
    for h in range(H):
        xt = Atrue @ xt; xh = Ahat @ xh; e.append(np.linalg.norm(xt - xh))
    errs.append(e)
errs = np.array(errs)
save("compound.dat", [np.arange(H + 1), errs.mean(0), np.percentile(errs, 90, axis=0)], "h mean p90")
with open(F + "compound_meta.tex", "w") as f:
    f.write(f"\\def\\onestep{{{one_step:.3f}}}\\def\\errten{{{errs.mean(0)[10]:.3f}}}\\def\\errforty{{{errs.mean(0)[40]:.3f}}}\n")
print("one-step err", one_step, "h10", errs.mean(0)[10], "h40", errs.mean(0)[40])

# ---------------------------------------------------------------- 8. CEM on a 2D action cost
goal = np.array([1.6, 1.2]); obs = np.array([0.8, 0.6]); R = 0.45
def cost(a):
    a = np.atleast_2d(a)
    c = ((a - goal) ** 2).sum(1)
    d = np.linalg.norm(a - obs, axis=1)
    return c + 6.0 * np.exp(-((d / R) ** 2) * 2)
g = np.linspace(-0.6, 2.4, 200)
GX, GY = np.meshgrid(g, g)
C = cost(np.column_stack([GX.ravel(), GY.ravel()])).reshape(GX.shape)
cs = plt.contour(GX, GY, C, levels=[0.3, 0.8, 1.5, 2.5, 4.0, 6.0])
with open(F + "cem_contours.tex", "w") as f:
    for lev, segs in zip(cs.levels, cs.allsegs):
        for seg in segs:
            if len(seg) < 3: continue
            pts = " ".join(f"({p[0]:.3f},{p[1]:.3f})" for p in seg[::3])
            f.write(f"\\addplot[cemcont] coordinates {{{pts}}};\n")
plt.close()
mu = np.array([0.0, 0.0]); sig = np.array([1.0, 1.0]); Ns, Ne = 50, 8
show = {0: 'a', 2: 'b', 4: 'c'}
with open(F + "cem_iters.tex", "w") as f:
    for it in range(6):
        smp = mu + sig * rng.normal(0, 1, (Ns, 2))
        c = cost(smp); el = smp[np.argsort(c)[:Ne]]
        if it in show:
            L = show[it]
            save(f"cem_all_{L}.dat", [smp[:, 0], smp[:, 1]], "x y")
            save(f"cem_el_{L}.dat", [el[:, 0], el[:, 1]], "x y")
            f.write(f"\\def\\cemmux{L}{{{mu[0]:.3f}}}\\def\\cemmuy{L}{{{mu[1]:.3f}}}\n")
        print("CEM it", it, "mu", np.round(mu, 3), "sig", np.round(sig, 3), "best cost", round(c.min(), 3))
        mu = el.mean(0); sig = el.std(0) + 0.02
    f.write(f"\\def\\cemfx{{{mu[0]:.3f}}}\\def\\cemfy{{{mu[1]:.3f}}}\n")
print("CEM final mu", mu)
