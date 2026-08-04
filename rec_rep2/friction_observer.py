"""
Model-free friction observer for the Kinova Gen3 7-DoF arm.

Implements the rigid-joint formulation of the observer from:

    "Model-Free Friction Observers for Flexible Joint Robots
     with Torque Measurements"
    Gaz, Cognetti, Oliva, Giordano, De Luca
    IEEE Transactions on Robotics, 2019
    DOI: 10.1109/TRO.2019.2926915  (IEEE 8781838)
    arXiv: 1907.00553

Observer equation (discrete time, per joint):

    residual[k] = tau_measured[k] - g_q[k] - b_qdot[k]
    xi[k+1] = xi[k] + dt * ( -(L + Lp) * xi[k]  +  L * residual[k] )

Friction estimate:

    tau_hat_friction[k] = xi[k]

The passivity-preserving property comes from feeding back xi (the
filtered estimate) rather than the raw residual: noise in tau_measured
is attenuated by the (L + Lp) pole before it can re-enter the loop.

Tuning guide
------------
L (rad/s, per joint)
    Observer bandwidth: how fast xi tracks friction changes.
    Start at 2.0 rad/s. Increase until friction feel improves.
    Back off immediately if oscillation or vibration appears.

Lp (rad/s, per joint)
    Proportional correction gain.  Adds a restoring force proportional
    to the current estimate, stiffening the convergence without raising
    the noise floor as sharply as L alone.
    Start at 0.5 * L.  Increase if stiction remains noticeable at rest.
"""

from typing import List, Optional

import numpy as np


class FrictionObserver:
    """Per-joint model-free friction observer (rigid-joint formulation)."""

    def __init__(
        self,
        n_joints: int = 7,
        L: Optional[List[float]] = None,
        Lp: Optional[List[float]] = None,
    ) -> None:
        """
        Construct a FrictionObserver.

        Parameters
        ----------
        n_joints : number of joints (default 7 for Gen3)
        L        : dynamic observer gains, length n_joints (rad/s)
        Lp       : static proportional gains, length n_joints (rad/s)
        """
        self.n = n_joints
        self.L = (
            np.array(L, dtype=float)
            if L is not None
            else np.full(n_joints, 2.0)
        )
        self.Lp = (
            np.array(Lp, dtype=float)
            if Lp is not None
            else np.full(n_joints, 1.0)
        )
        self._xi = np.zeros(n_joints)

    def reset(self) -> None:
        """Reset observer state to zero (call on mode entry)."""
        self._xi[:] = 0.0

    def update(
        self,
        tau_measured: np.ndarray,
        g_q: np.ndarray,
        b_qdot: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """
        Advance the observer one step and return the friction estimate.

        Parameters
        ----------
        tau_measured : joint torque sensor readings, shape (n,), Nm
        g_q          : gravity compensation torques from Pinocchio,
                       shape (n,), Nm
        b_qdot       : viscous damping term  b * q_dot, shape (n,), Nm
        dt           : elapsed time since last call, seconds

        Returns
        -------
        tau_hat_friction : friction torque estimate, shape (n,), Nm.
                           Subtract this from the commanded torque to
                           cancel friction.
        """
        residual = tau_measured - g_q - b_qdot
        d_xi = -(self.L + self.Lp) * self._xi + self.L * residual
        self._xi += dt * d_xi
        return self._xi.copy()
