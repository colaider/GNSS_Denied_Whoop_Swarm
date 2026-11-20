from DVS.DVS import DVS
from DVS.plotter import Plotter
import numpy as np

#self organizing maps (add ?)
def main():
    bench = np.column_stack([np.linspace(0, 0.9, 7), np.zeros(7)])

    dvs = DVS(bench.mean(axis=0, keepdims=True))
    dvs.update_drone_vector(bench)

    plotter = Plotter(1, 2)
    plotter.plot(dvs, 0, "Startup: spots seeded from the bench")

    dvs.update(dvs.P_w + np.array([[2.0, 1.0]]), np.diag([1.6, 1/1.6]), 1.0, -np.pi/4)
    plotter.plot(dvs, 1, "Moved, squished, rotated -45°")

    plotter.show()

if __name__ == "__main__":
    main()
