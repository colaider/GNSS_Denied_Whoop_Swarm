import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


class Plotter:
    SURFACE = "#fcfcfb"
    INK = "#0b0b0b"
    INK_2 = "#52514e"
    MUTED = "#898781"
    GRID = "#e1e0d9"
    CELL = "#cde2fb"
    SPOT = "#2a78d6"
    DRONE = "#eb6834"

    def __init__(self, rows=1, cols=1, size=5):
        self.fig, axes = plt.subplots(rows, cols, figsize=(size*cols, size*rows), squeeze=False)
        self.fig.patch.set_facecolor(self.SURFACE)
        self.axes = axes.ravel()

    def plot(self, dvs, k=0, title=None):
        ax = self.axes[k]
        ax.set_facecolor(self.SURFACE)

        for cell in dvs.cells:
            ax.add_patch(Polygon(cell, closed=True, facecolor=self.CELL,
                                 edgecolor=self.SURFACE, linewidth=2, zorder=1))
        ax.add_patch(Polygon(dvs.q_w, closed=True, fill=False,
                             edgecolor=self.INK_2, linewidth=1.5, zorder=2))

        for p, s in zip(dvs.p_d, dvs.centroids):
            ax.plot([p[0], s[0]], [p[1], s[1]], color=self.MUTED,
                    linewidth=1, linestyle="--", zorder=3)

        ax.scatter(dvs.centroids[:, 0], dvs.centroids[:, 1], s=40, color=self.SPOT,
                   edgecolor=self.SURFACE, linewidth=1.5, zorder=4, label="spot (target)")
        ax.scatter(dvs.p_d[:, 0], dvs.p_d[:, 1], s=55, marker="^", color=self.DRONE,
                   edgecolor=self.SURFACE, linewidth=1.5, zorder=5, label="drone")
        ax.scatter(*np.ravel(dvs.P_w), s=60, marker="+", color=self.INK, zorder=6, label="centre $P_w$")

        if len(dvs.p_d) <= 12:
            for i, s in enumerate(dvs.centroids):
                ax.annotate(str(i), s, xytext=(5, 5), textcoords="offset points",
                            fontsize=8, color=self.INK_2, zorder=7)

        ax.set_aspect("equal")
        ax.autoscale_view()
        ax.grid(color=self.GRID, linewidth=0.5)
        ax.tick_params(colors=self.MUTED, labelsize=8)
        for side in ax.spines.values():
            side.set_color(self.GRID)
        ax.set_title(title or "", color=self.INK, fontsize=11, loc="left")
        ax.legend(loc="upper right", fontsize=8, frameon=False, labelcolor=self.INK_2)
        return ax

    def show(self):
        self.fig.tight_layout()
        plt.show()
