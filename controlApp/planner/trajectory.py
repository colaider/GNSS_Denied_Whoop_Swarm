# Trajectory - where a drone should be at time t: at(t) -> (x, y, z, yaw, vx, vy, vz), m, rad, m/s,
# relative to the hover point (StateMachine::origin), start frame. The drone link samples it every
# 0.2 s and the drone interpolates between the points (cubic Hermite: positions + velocities). Go-tos are min-jerk.
# Before 0 it is at the start, after `duration` at the end (and stays there).
#
#   traj = Trajectory.goTo(start=(0, 0, 0), goal=(0.4, 0.2, 0), speed=0.3)
#   traj.at(1.0), traj.duration, traj.end
#   traj.points(0.1)                    # [(x, y, z)] every 0.1 s, to draw it
#
# New kinds of trajectory: another @staticmethod that makes Trajectory(fn, duration).

import math

from communication.protocol import SM

MAX_DIST = SM.defaults["maxDistM"]      # the drone keeps its setpoint within this of the hover point (x, y)...
MAX_HEIGHT = SM.defaults["maxHeightM"]  # ...and below this (start frame); a target outside is cut there
MAX_SPEED = SM.defaults["maxSpeed"]     # faster than this the drone does not follow


class Trajectory:
    def __init__(self, fn, duration):
        self.fn, self.duration = fn, duration
        self.end = self.at(duration)[:3]

    def at(self, t):
        return tuple(float(v) for v in self.fn(min(max(t, 0.0), self.duration)))

    def points(self, dt=0.1):
        n = max(1, int(self.duration / dt))
        return [self.at(k * self.duration / n)[:3] for k in range(n + 1)]

    @staticmethod
    def hold(p):                        # stay at p
        x, y, z = p
        return Trajectory(lambda t: (x, y, z, 0.0, 0.0, 0.0, 0.0), 0.0)

    @staticmethod
    def goTo(start, goal, speed=0.3):   # straight line, min-jerk, peak speed `speed`
        # Min-jerk: speed AND acceleration 0 at both ends (the half cosine used before jumped to its full
        # acceleration at the start and from full braking to 0 at the end: the drone lagged, then overshot).
        d = [g - s for g, s in zip(goal, start)]
        dist = math.sqrt(sum(v * v for v in d))
        if dist < 1e-3:
            return Trajectory.hold(start)
        T = max(1.875 * dist / min(speed, MAX_SPEED), 1.0)          # peak speed of min-jerk: 1.875 dist / T

        def fn(t):
            k, dk = minJerk(t / T)
            return (*(s + k * e for s, e in zip(start, d)), 0.0, *(dk / T * e for e in d))
        return Trajectory(fn, T)

    @staticmethod
    def forwardBack(dist=0.4, moveS=1.5, stopS=5.0, times=1):   # hoverTest.py's T: forward, stop, back, stop
        period = 2 * (moveS + stopS)

        def fn(t):
            t %= period
            if t < moveS:
                k = t / moveS
                return dist * (1 - math.cos(math.pi * k)) / 2, 0, 0, 0, dist * math.pi / (2 * moveS) * math.sin(math.pi * k), 0, 0
            if t < moveS + stopS:
                return dist, 0, 0, 0, 0, 0, 0
            if t < 2 * moveS + stopS:
                k = (t - moveS - stopS) / moveS
                return dist * (1 + math.cos(math.pi * k)) / 2, 0, 0, 0, -dist * math.pi / (2 * moveS) * math.sin(math.pi * k), 0, 0
            return 0, 0, 0, 0, 0, 0, 0
        return Trajectory(fn, times * period - 1e-6)


def minJerk(tau):                       # 0 -> 1 over tau 0 -> 1: (s, ds/dtau); s', s'' = 0 at both ends
    tau = min(max(tau, 0.0), 1.0)
    return tau ** 3 * (10 - 15 * tau + 6 * tau ** 2), 30 * tau ** 2 * (1 - tau) ** 2


def clampGoal(goal, origin):            # a goal (start frame) inside what the drone accepts
    x, y, z = goal
    ox, oy, _ = origin
    x = min(max(x, ox - MAX_DIST), ox + MAX_DIST)
    y = min(max(y, oy - MAX_DIST), oy + MAX_DIST)
    return x, y, min(max(z, 0.1), MAX_HEIGHT)
