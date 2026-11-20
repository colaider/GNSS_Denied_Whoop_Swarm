"""Figures for dvs_math_guide.tex. Run: python3 figures.py (needs numpy + matplotlib only)."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MPoly, Circle

plt.rcParams.update({"font.size": 9, "axes.titlesize": 9})
COL = plt.cm.tab20(np.linspace(0, 1, 20))


# ---------------- small geometry helpers (same maths as in the guide) ----------------
def clip(poly, n, c):
    """Keep the part of convex polygon `poly` with n.q <= c (Sutherland-Hodgman, one half-plane)."""
    out = []
    for k in range(len(poly)):
        A, B = poly[k], poly[(k + 1) % len(poly)]
        da, db = n @ A - c, n @ B - c
        if da <= 0:
            out.append(A)
        if (da < 0 < db) or (db < 0 < da):
            t = da / (da - db)
            out.append(A + t * (B - A))
    return out


def voronoi_cells(P, domain, r=0.0):
    """Cells clipped to a convex domain; also returns which neighbour produced each edge (-1 = domain)."""
    cells, owners = [], []
    for i, p in enumerate(P):
        poly = [np.array(v, float) for v in domain]
        lab = [-1] * len(poly)  # label of edge starting at vertex k
        for j, q in enumerate(P):
            if i == j:
                continue
            n = q - p
            L = np.linalg.norm(n)
            n = n / L
            c = n @ (p + q) / 2 - r
            new, newlab = [], []
            for k in range(len(poly)):
                A, B = poly[k], poly[(k + 1) % len(poly)]
                da, db = n @ A - c, n @ B - c
                if da <= 0:
                    new.append(A); newlab.append(lab[k])
                if (da < 0 < db) or (db < 0 < da):
                    t = da / (da - db)
                    new.append(A + t * (B - A))
                    newlab.append(j if da < 0 else lab[k])
            poly, lab = new, newlab
            if len(poly) < 3:
                break
        cells.append(np.array(poly)); owners.append(lab)
    return cells, owners


def centroid(poly):
    x, y = poly[:, 0], poly[:, 1]
    xs, ys = np.roll(x, -1), np.roll(y, -1)
    w = x * ys - xs * y
    A = w.sum() / 2
    return np.array([((x + xs) * w).sum(), ((y + ys) * w).sum()]) / (6 * A)


def lloyd(P, domain, iters):
    hist = [P.copy()]
    for _ in range(iters):
        cells, _ = voronoi_cells(P, domain)
        P = np.array([centroid(c) for c in cells])
        hist.append(P.copy())
    return P, hist


def hull(pts):
    pts = sorted(map(tuple, pts))
    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and np.cross(np.subtract(h[-1], h[-2]), np.subtract(p, h[-2])) <= 0:
                h.pop()
            h.append(p)
        return h
    lo, up = half(pts), half(pts[::-1])
    return np.array(lo[:-1] + up[:-1])


def circle_poly(c, R, n=48):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[c[0] + R * np.cos(t), c[1] + R * np.sin(t)]


def draw_cells(ax, cells, P, colors=None, alpha=0.25, labels=True):
    for i, c in enumerate(cells):
        col = COL[i % 20] if colors is None else colors[i]
        ax.add_patch(MPoly(c, closed=True, fc=col, ec="0.35", lw=0.8, alpha=alpha))
        ax.add_patch(MPoly(c, closed=True, fill=False, ec="0.35", lw=0.8))
    ax.plot(P[:, 0], P[:, 1], "k.", ms=5)
    if labels:
        for i, p in enumerate(P):
            ax.annotate(str(i + 1), p, xytext=(3, 3), textcoords="offset points", fontsize=7)


def finish(ax, lim=None):
    ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
    if lim:
        ax.set_xlim(lim[0]); ax.set_ylim(lim[1])


# ---------------- Fig 1: Lloyd iterations ----------------
rng = np.random.default_rng(3)
dom = circle_poly((0, 0), 1.0, 40)
P0 = rng.uniform(-0.45, 0.45, (9, 2))
_, hist = lloyd(P0, dom, 30)
fig, axs = plt.subplots(1, 3, figsize=(7.2, 2.5))
for ax, k, tt in zip(axs, [0, 2, 30], ["start (iteration 0)", "after 2 iterations", "after 30: converged"]):
    cells, _ = voronoi_cells(hist[k], dom)
    draw_cells(ax, cells, hist[k], labels=False)
    if k < 30:
        nxt = np.array([centroid(c) for c in cells])
        for p, q in zip(hist[k], nxt):
            ax.annotate("", q, p, arrowprops=dict(arrowstyle="->", color="crimson", lw=0.9))
    ax.set_title(tt); finish(ax)
fig.tight_layout(); fig.savefig("fig_lloyd.pdf"); plt.close(fig)

# ---------------- Fig 2: squish and scale ----------------
P, _ = lloyd(rng.uniform(-0.5, 0.5, (8, 2)), dom, 40)
fig, axs = plt.subplots(1, 3, figsize=(7.2, 2.5))
for ax, (al, s, tt) in zip(axs, [(1, 1, r"$\alpha=1,\ r/r_0=1$"), (2, 1, r"$\alpha=2,\ r/r_0=1$ (area kept)"), (2, 0.7, r"$\alpha=2,\ r/r_0=0.7$")]):
    D = np.diag([al * s, s / al])
    ax.add_patch(MPoly(dom, fill=False, ls="--", ec="0.6"))
    ax.add_patch(MPoly(dom @ D.T, fc="tab:blue", alpha=0.15, ec="tab:blue"))
    Q = P @ D.T
    ax.plot(Q[:, 0], Q[:, 1], "ko", ms=3.5)
    ax.set_title(tt); finish(ax, ((-2.1, 2.1), (-1.2, 1.2)))
fig.tight_layout(); fig.savefig("fig_affine.pdf"); plt.close(fig)

# ---------------- Fig 3: minimum-jerk segment ----------------
T = 2.0
p0, v0, a0, pf, vf, af = 0.0, 0.0, 0.0, 3.0, 0.0, 0.0
M = np.array([[T**3, T**4, T**5], [3*T**2, 4*T**3, 5*T**4], [6*T, 12*T**2, 20*T**3]])
rhs = np.array([pf - (p0 + v0*T + a0*T**2/2), vf - (v0 + a0*T), af - a0])
c3, c4, c5 = np.linalg.solve(M, rhs)
t = np.linspace(0, T, 200)
pos = p0 + v0*t + a0/2*t**2 + c3*t**3 + c4*t**4 + c5*t**5
vel = v0 + a0*t + 3*c3*t**2 + 4*c4*t**3 + 5*c5*t**4
acc = a0 + 6*c3*t + 12*c4*t**2 + 20*c5*t**3
jer = 6*c3 + 24*c4*t + 60*c5*t**2
fig, axs = plt.subplots(1, 4, figsize=(7.2, 1.9))
for ax, y, tt in zip(axs, [pos, vel, acc, jer], ["position [m]", "velocity [m/s]", "acceleration [m/s$^2$]", "jerk [m/s$^3$]"]):
    ax.plot(t, y, "tab:blue"); ax.axhline(0, color="0.8", lw=0.6); ax.set_title(tt); ax.set_xlabel("t [s]")
fig.tight_layout(); fig.savefig("fig_minjerk.pdf"); plt.close(fig)

# ---------------- Fig 4: cell complex and boundary cancellation ----------------
box = [(-2, -1.3), (2, -1.3), (2, 1.3), (-2, 1.3)]
P = np.array([[-1.4, 0.6], [-0.5, 0.7], [0.5, 0.6], [1.4, 0.7], [-1.4, -0.6], [-0.5, -0.7], [0.5, -0.6], [1.4, -0.7]])
cells, owners = voronoi_cells(P, box)
group = {0, 1, 4, 5}
fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.6))
ax = axs[0]
draw_cells(ax, cells, P, colors=[COL[0] if i in group else COL[15] for i in range(len(P))])
ax.set_title("cells of all drones; group A = {1, 2, 5, 6} (blue)"); finish(ax)
ax = axs[1]
for i in range(len(P)):
    c, lab = cells[i], owners[i]
    for k in range(len(c)):
        A, B = c[k], c[(k + 1) % len(c)]
        j = lab[k]
        if i not in group:
            continue
        if j in group:
            ax.plot(*zip(A, B), color="0.6", ls=":", lw=1)       # shared inside A: cancels
        elif j == -1:
            ax.plot(*zip(A, B), color="k", lw=2.2)                # domain border
        else:
            ax.plot(*zip(A, B), color="crimson", lw=2.2)          # border with another group
ax.plot(P[list(group), 0], P[list(group), 1], "k.", ms=5)
ax.set_title(r"$\partial A$: dotted edges cancel, red = border with B")
finish(ax, ((-2.1, 2.1), (-1.4, 1.4)))
fig.tight_layout(); fig.savefig("fig_complex.pdf"); plt.close(fig)

# ---------------- Fig 5: Delaunay graph and a split cut ----------------
P, _ = lloyd(rng.uniform(-0.6, 0.6, (12, 2)) * [1.6, 1], circle_poly((0, 0), 1.0, 40) * [1.6, 1], 40)
dom2 = circle_poly((0, 0), 1.0, 40) * [1.6, 1]
cells, owners = voronoi_cells(P, dom2)
A = {i for i in range(len(P)) if P[i, 1] > 0}
fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.6))
ax = axs[0]
draw_cells(ax, cells, P, colors=[COL[0] if i in A else COL[6] for i in range(len(P))], alpha=0.2)
for i in range(len(P)):
    for j in set(owners[i]):
        if j > i:
            cut = (i in A) != (j in A)
            ax.plot(*zip(P[i], P[j]), color="crimson" if cut else "0.25", lw=1.6 if cut else 0.8, ls="--" if cut else "-")
ax.set_title("Delaunay graph; red dashed = cut edges"); finish(ax)
ax = axs[1]
for side, grp, col in [(+1, A, COL[0]), (-1, set(range(len(P))) - A, COL[6])]:
    off = np.array([0, 0.45 * side])
    for i in grp:
        ax.add_patch(MPoly(cells[i] + off, fc=col, alpha=0.3, ec="0.35"))
    pts = P[list(grp)] + off
    ax.plot(pts[:, 0], pts[:, 1], "k.", ms=5)
ax.add_patch(Circle((2.3, 0), 0.25, fc="0.3"))
ax.annotate("obstacle", (2.3, -0.42), ha="center", fontsize=7)
ax.set_title("after the split: two DVS bodies A and B"); finish(ax, ((-1.9, 2.7), (-1.6, 1.6)))
fig.tight_layout(); fig.savefig("fig_split.pdf"); plt.close(fig)

# ---------------- Fig 6: merge with the hull ----------------
cA, cB, R = np.array([-0.95, 0.0]), np.array([0.95, 0.0]), 1.0
blobA, blobB = circle_poly(cA, R, 60), circle_poly(cB, R, 60)
H = hull(np.vstack([blobA, blobB]))
fig, axs = plt.subplots(1, 2, figsize=(7.2, 2.4))
ax = axs[0]
ax.add_patch(MPoly(blobA, fc=COL[0], alpha=0.35, ec="k")); ax.add_patch(MPoly(blobB, fc=COL[6], alpha=0.35, ec="k"))
ax.annotate("waist", (0, 0.05), ha="center", fontsize=8)
ax.set_title(r"touching: $A\cup B$ has a narrow waist"); finish(ax, ((-2.2, 2.2), (-1.2, 1.2)))
ax = axs[1]
ax.add_patch(MPoly(H, fc="gold", alpha=0.45, ec="k", lw=1.2))
ax.add_patch(MPoly(blobA, fc=COL[0], alpha=0.35, ec="0.4")); ax.add_patch(MPoly(blobB, fc=COL[6], alpha=0.35, ec="0.4"))
ax.annotate("G", (0, 0.78), ha="center", fontsize=9, weight="bold"); ax.annotate("G", (0, -0.9), ha="center", fontsize=9, weight="bold")
ax.set_title(r"convex hull fills the waist: $T'=A\cup B\cup G$"); finish(ax, ((-2.2, 2.2), (-1.2, 1.2)))
fig.tight_layout(); fig.savefig("fig_merge.pdf"); plt.close(fig)

# ---------------- Fig 7: buffered Voronoi cells ----------------
P = np.array([[-0.9, 0.2], [0.0, 0.5], [0.8, 0.1], [-0.4, -0.6], [0.5, -0.7]])
box = [(-1.6, -1.3), (1.6, -1.3), (1.6, 1.3), (-1.6, 1.3)]
cells, _ = voronoi_cells(P, box)
bufs, _ = voronoi_cells(P, box, r=0.18)
fig, ax = plt.subplots(figsize=(3.4, 2.7))
draw_cells(ax, cells, P, alpha=0.15)
for b in bufs:
    ax.add_patch(MPoly(b, fill=False, ec="k", ls="--", lw=1))
ax.set_title(r"cells (solid) and buffered cells (dashed, shrunk by $l_s/2$)"); finish(ax)
fig.tight_layout(); fig.savefig("fig_buffered.pdf"); plt.close(fig)
print("figures written")
