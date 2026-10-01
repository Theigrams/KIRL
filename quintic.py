"""Quintic motion primitives (paper Sec. III).

Each trajectory segment is the minimum-jerk motion between two boundary
kinematic states (Theorem 1): a fifth-degree polynomial per axis,

    s(t) = sum_{k=0}^{5} c_k t^k,    t in [0, T],

whose six coefficient pairs are fixed by the position, velocity and
acceleration at both endpoints.
"""

import numpy as np


class KinematicState:
    """The tool's kinematic state x = (q, v, a) at one instant, in 2D."""

    def __init__(self, position, velocity, acceleration):
        self.position = np.asarray(position, dtype=float)
        self.velocity = np.asarray(velocity, dtype=float)
        self.acceleration = np.asarray(acceleration, dtype=float)

    def transform(self, R, translation):
        """Rigid change of frame: rotate by R; only the position is translated."""
        return KinematicState(R @ self.position + translation,
                              R @ self.velocity,
                              R @ self.acceleration)


def boundary_conditions(x_start, x_end):
    """Stack two boundary states into the matrix B = [q0 v0 a0 q1 v1 a1]^T (6x2)."""
    return np.array([x_start.position, x_start.velocity, x_start.acceleration,
                     x_end.position, x_end.velocity, x_end.acceleration])


def coefficients(B, T):
    """Coefficient matrix C = A(T)^{-1} B of the segment quintic.

    A(T) expresses the boundary conditions of the quintic and its first two
    derivatives at t = 0 and t = T; its inverse is known in closed form
    (Supplementary Material, Sec. S2).
    """
    t2, t3, t4, t5 = T ** 2, T ** 3, T ** 4, T ** 5
    A_inv = np.zeros((6, 6))
    A_inv[0, 0], A_inv[1, 1], A_inv[2, 2] = 1.0, 1.0, 0.5
    A_inv[3] = (-10 / t3, -6 / t2, -1.5 / T, 10 / t3, -4 / t2, 0.5 / T)
    A_inv[4] = (15 / t4, 8 / t3, 1.5 / t2, -15 / t4, 7 / t3, -1 / t2)
    A_inv[5] = (-6 / t5, -3 / t4, -0.5 / t3, 6 / t5, -3 / t4, 0.5 / t3)
    return A_inv @ B


# Differentiating s(t) = sum c_k t^k term by term brings down these factors:
# k for velocity, k(k-1) for acceleration, k(k-1)(k-2) for jerk.
_VEL_FACTORS = np.array([[1], [2], [3], [4], [5]])
_ACC_FACTORS = np.array([[2], [6], [12], [20]])
_JERK_FACTORS = np.array([[6], [24], [60]])


def time_grid(T, dt):
    """Sample times covering [0, T] every dt, endpoint included, so that
    every instant lies within dt/2 of a sample."""
    return np.append(np.arange(0, T, dt), T)


def power_basis(t):
    """Stacked rows [t^0, ..., t^5] for a vector of sample times."""
    return np.array([t ** k for k in range(6)])


def position(powers, C):
    return powers.T @ C


def velocity(powers, C):
    return (_VEL_FACTORS * powers[:5]).T @ C[1:]


def acceleration(powers, C):
    return (_ACC_FACTORS * powers[:4]).T @ C[2:]


def jerk(powers, C):
    return (_JERK_FACTORS * powers[:3]).T @ C[3:]


def sample(C, T, dt):
    """Position, velocity, acceleration and jerk along the segment, every dt."""
    powers = power_basis(time_grid(T, dt))
    return (position(powers, C), velocity(powers, C),
            acceleration(powers, C), jerk(powers, C))
