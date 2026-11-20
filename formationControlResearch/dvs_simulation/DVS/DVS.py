import numpy as np
from scipy.optimize import linear_sum_assignment


class DVS:
    def __init__(self, P_w, A=np.eye(2), r=1.0, theta=0.0):
        self.P_w, self.A, self.r, self.theta = P_w, A, r, theta
        self.q_b = self.define_sahpe()
        self.q_w = self.map_to_world(self.q_b)


    def define_sahpe(self, N=100):
        t = np.linspace(0, 2*np.pi, N, endpoint=False)
        return np.column_stack([np.cos(t), np.sin(t)])


    def linear(self):
        c, s = np.cos(self.theta), np.sin(self.theta)
        R = np.array([[c, -s], [s, c]])
        return R @ self.A


    def map_to_world(self, points) -> np.ndarray:
        return self.P_w + self.r * points @ self.linear().T


    def map_to_body(self, points) -> np.ndarray:
        return (points - self.P_w) / self.r @ np.linalg.inv(self.linear()).T



    def create_cells(self, p_c:np.ndarray):
        # cnter_lp = lambda p1, p2: p1 + 0.5*(p2-p1)
        # homo_l = lambda p1, p2: np.cross(np.array([p1[0], p1[1], 1]), np.array([p2[0], p2[1], 1]))
        # perp_l = lambda h_l, p: np.cross(np.array([h_l[0], h_l[1], 0]), p)
        # np_l = lambda p1, p2: np.array([p2[0]-p1[0], p2[1]-p1[1]], (p2@p2 - p1@p1)/2)

        N = len(p_c)
        boundary = np.column_stack([self.q_b, np.ones(len(self.q_b))])

        cells = np.empty(N, dtype=object)
        for i in range(N):
            p2, p1 = p_c[np.arange(N) != i], p_c[i]
            bisectors = np.column_stack([p2-p1, -((p2**2).sum(axis=1) - p1@p1)/2])
            poly = boundary

            for j in range(len(bisectors)):
                keep = []
                s = poly @ bisectors[j]
                for k in range(len(poly)):
                    m = (k + 1) % len(poly)
                    if s[k] <= 0: keep.append(poly[k])
                    if s[k]*s[m] < 0:
                        X = np.cross(np.cross(poly[k], poly[m]), bisectors[j])
                        keep.append(X/X[2])

                poly = np.array(keep).reshape(-1, 3)
            cells[i] = poly[:, :2]

        return cells


    def update(self, P_w, A, r, theta):
        self.P_w, self.A, self.r, self.theta = P_w, A, r, theta
        self.q_w = self.map_to_world(self.q_b)
        self.centroids = self.map_to_world(self.p_c)
        self.cells = self.map_cells()


    def map_cells(self):
        cells = np.empty(len(self.cells_b), dtype=object)
        for i, cell in enumerate(self.cells_b):
            cells[i] = self.map_to_world(cell)
        return cells


    @staticmethod
    def centroid(cell):
        a, b, c = cell[0], cell[1:-1], cell[2:]
        u, v = b - a, c - a
        A = 0.5 * (u[:,0]*v[:,1] - u[:,1]*v[:,0])
        g = (a + b + c) / 3
        return A @ g / A.sum()


    @staticmethod
    def inertia(cell):
        x, y = cell[:,0], cell[:,1]
        x1, y1 = np.roll(x, -1), np.roll(y, -1)
        cr = x*y1 - x1*y
        I0 = (cr * (x*x + x*x1 + x1*x1 + y*y + y*y1 + y1*y1)).sum() / 12
        c = DVS.centroid(cell)
        return I0 - cr.sum()/2 * (c @ c)


    def lloyd(self, p_d, starts=5, iters=500, tol=1e-6):
        rng = np.random.default_rng(0)
        q = self.map_to_body(p_d)
        q = q - q.mean(axis=0)
        q = 0.8 * q / max(np.linalg.norm(q, axis=1).max(), 1e-9)

        best, best_J = None, np.inf
        for _ in range(starts):
            p_c = q + rng.normal(0, 0.02, q.shape)
            for _ in range(iters):
                new = np.array([self.centroid(cell) for cell in self.create_cells(p_c)])
                moved = np.abs(new - p_c).max()
                p_c = new
                if moved < tol: break

            J = sum(self.inertia(cell) for cell in self.create_cells(p_c))
            if J < best_J: best, best_J = p_c, J

        return best


    def vector_asigment(self):
        spots = self.map_to_world(self.p_c)
        C = ((self.p_d[:, None, :] - spots[None, :, :])**2).sum(axis=2)
        _, v = linear_sum_assignment(C)
        self.p_c, self.cells_b = self.p_c[v], self.cells_b[v]


    def update_drone_vector(self, p_d):
        self.p_d = p_d
        self.p_c = self.lloyd(p_d)
        self.cells_b = self.create_cells(self.p_c)
        self.vector_asigment()
        self.update(self.P_w, self.A, self.r, self.theta)
