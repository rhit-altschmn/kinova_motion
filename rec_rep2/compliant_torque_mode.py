"""
Compliant torque posing mode for the Kinova Gen3 7-DoF arm.

Manages the LOW_LEVEL_SERVOING control loop that gives the arm a
gravity-compensated, friction-cancelled feel during kinesthetic teaching.

see readme for more notes
"""

import math
import os
import subprocess
import tempfile
import threading
import time
from typing import Dict, List, Optional

import numpy as np

from .friction_observer import FrictionObserver
from .safety_monitor import SafetyMonitor

FAKE_HARDWARE = os.environ.get('FAKE_HARDWARE', '0').lower() in ('1', 'true', 'yes')

_ROBOT_IP = os.environ.get('ROBOT_IP', '192.168.0.10')
_ROBOT_PORT = 10000
_ROBOT_PORT_RT = 10001   # UDP realtime port for BaseCyclic
_USERNAME = 'admin'
_PASSWORD = 'admin'

N_JOINTS = 7
_DEG2RAD = math.pi / 180.0

# Gen3 7-DoF rated output torques (Nm).
# Large modules (joints 1-3): ~56 Nm. Small modules (joints 4-7): ~28 Nm.
_RATED_NM = [56.0, 56.0, 56.0, 28.0, 28.0, 28.0, 28.0]
DEFAULT_TAU_LIMITS: List[float] = [0.4 * t for t in _RATED_NM]

# Velocity watchdog: ~45 deg/s (generous for hand-guiding; well below max).
DEFAULT_VEL_LIMITS: List[float] = [0.8] * N_JOINTS

if not FAKE_HARDWARE:
    from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import (
        ActuatorConfigClient,
    )
    from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
    from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
    from kortex_api.autogen.messages import (
        ActuatorConfig_pb2,
        Base_pb2,
        BaseCyclic_pb2,
        Session_pb2,
    )
    from kortex_api.RouterClient import RouterClient
    from kortex_api.SessionManager import SessionManager
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.UDPTransport import UDPTransport


# ── URDF / Pinocchio helpers ──────────────────────────────────────────────────

def _build_q_pin(angles_rad: np.ndarray) -> np.ndarray:
    """
    Convert 7 joint angles (radians) to the 11-element Pinocchio config.

    Gen3 URDF joint layout (idx_q in the 11-element q vector):
      joint_1 (RUBZ) → q[0:2]  = [cos θ₁, sin θ₁]
      joint_2 (RZ)   → q[2]    = θ₂
      joint_3 (RUBZ) → q[3:5]  = [cos θ₃, sin θ₃]
      joint_4 (RZ)   → q[5]    = θ₄
      joint_5 (RUBZ) → q[6:8]  = [cos θ₅, sin θ₅]
      joint_6 (RZ)   → q[8]    = θ₆
      joint_7 (RUBZ) → q[9:11] = [cos θ₇, sin θ₇]
    """
    q = np.empty(11)
    # Joints 0,2,4,6 (0-indexed) are RUBZ
    for joint_i, q_start in zip([0, 2, 4, 6], [0, 3, 6, 9]):
        q[q_start] = math.cos(angles_rad[joint_i])
        q[q_start + 1] = math.sin(angles_rad[joint_i])
    # Joints 1,3,5 (0-indexed) are RZ
    q[2] = angles_rad[1]
    q[5] = angles_rad[3]
    q[8] = angles_rad[5]
    return q


def _q_pin_to_angles(q: np.ndarray) -> np.ndarray:
    """
    Inverse of _build_q_pin: extract 7 joint angles (radians) from the
    11-element Pinocchio configuration vector.
    """
    angles = np.empty(7)
    # RUBZ joints (0-indexed: 0,2,4,6) → atan2(sin, cos)
    angles[0] = math.atan2(q[1], q[0])
    angles[2] = math.atan2(q[4], q[3])
    angles[4] = math.atan2(q[7], q[6])
    angles[6] = math.atan2(q[10], q[9])
    # RZ joints (0-indexed: 1,3,5) → direct
    angles[1] = q[2]
    angles[3] = q[5]
    angles[5] = q[8]
    return angles


# ── Main class ────────────────────────────────────────────────────────────────

class CompliantTorqueMode:
    """
    Manage the kinesthetic teaching torque control loop.

    Usage::

        mode = CompliantTorqueMode(node, params)
        if mode.enter():
            # arm is now compliant; user guides it
            ...
        mode.exit()   # blocks until safe shutdown
    """

    def __init__(self, node: object, params: Dict) -> None:
        """
        Construct a CompliantTorqueMode.

        Parameters
        ----------
        node   : rclpy Node — used for logging and robot_description param
        params : dict of configuration values (see compliant_params.yaml)
        """
        self._node = node
        self._log = node.get_logger()

        rate_hz = float(params.get('control_rate_hz', 200))
        self._dt_nominal = 1.0 / rate_hz

        self._observer = FrictionObserver(
            n_joints=N_JOINTS,
            L=params.get('observer_L', [2.0] * N_JOINTS),
            Lp=params.get('observer_Lp', [1.0] * N_JOINTS),
        )
        self._b = np.array(
            params.get('damping_b', [0.5] * N_JOINTS), dtype=float
        )
        self._gravity_sign = np.array(
            params.get('gravity_sign', [1.0] * N_JOINTS), dtype=float
        )

        self._safety = SafetyMonitor(
            tau_limits=params.get('torque_limits', DEFAULT_TAU_LIMITS),
            vel_limits=params.get('velocity_limits_rad_s', DEFAULT_VEL_LIMITS),
            max_missed_cycles=int(params.get('max_missed_cycles', 10)),
            dt_nominal=self._dt_nominal,
            logger=self._log,
        )

        self._stop_event = threading.Event()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self._pin_model = None
        self._pin_data = None
        self._urdf_path: str = params.get('urdf_path', '')

        if FAKE_HARDWARE:
            self._log.info('[FAKE] CompliantTorqueMode: skipping robot connections.')
            return

        # TCP connection — used for mode-switching (BaseClient, ActuatorConfigClient)
        self._tcp_transport = TCPTransport()
        self._tcp_transport.connect(_ROBOT_IP, _ROBOT_PORT)
        self._tcp_router = RouterClient(self._tcp_transport, lambda x: None)
        self._tcp_session = SessionManager(self._tcp_router)
        sess = Session_pb2.CreateSessionInfo()
        sess.username = _USERNAME
        sess.password = _PASSWORD
        sess.session_inactivity_timeout = 60000
        sess.connection_inactivity_timeout = 2000
        self._tcp_session.CreateSession(sess)
        self._base = BaseClient(self._tcp_router)
        self._act_cfg = ActuatorConfigClient(self._tcp_router)

        # UDP connection: used for BaseCyclic.Refresh() at ~200 Hz
        self._udp_transport = UDPTransport()
        self._udp_transport.connect(_ROBOT_IP, _ROBOT_PORT_RT)
        self._udp_router = RouterClient(self._udp_transport, lambda x: None)
        self._cyclic = BaseCyclicClient(self._udp_router)

        self._log.info('CompliantTorqueMode: TCP + UDP connections established.')

    # ── public interface ──────────────────────────────────────────────────

    def enter(self) -> bool:
        """
        Switch to torque mode and start the control loop.

        Return True on success, False if anything fails (robot will remain
        in its previous mode).
        """
        if self._running:
            self._log.warn('CompliantTorqueMode.enter() called while already running.')
            return True

        self._observer.reset()
        self._safety.reset()
        self._stop_event.clear()

        try:
            self._ensure_pinocchio()
        except Exception as exc:
            self._log.error(f'Failed to load URDF for Pinocchio: {exc}')
            return False

        if FAKE_HARDWARE:
            self._running = True
            self._log.info('[FAKE] Compliant torque mode ON (simulated).')
            self._thread = threading.Thread(target=self._fake_loop, daemon=True)
            self._thread.start()
            return True

        try:
            self._enter_low_level_torque()
        except Exception as exc:
            self._log.error(f'Failed to enter LOW_LEVEL_SERVOING/TORQUE mode: {exc}')
            return False

        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
        self._log.info('Compliant torque mode ON.')
        return True

    def exit(self) -> None:
        """
        Stop the control loop and return the arm to position control.

        Block until the loop thread has exited (up to 3 seconds).
        """
        if not self._running:
            return
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self._running = False

        if FAKE_HARDWARE:
            self._log.info('[FAKE] Compliant torque mode OFF (simulated).')
            return

        # The loop calls _safe_exit() itself, but call again defensively
        # in case exit() is invoked from outside while the thread is gone.
        try:
            self._safe_exit()
        except Exception as exc:
            self._log.error(f'Defensive safe-exit failed: {exc}')

        self._log.info('Compliant torque mode OFF.  Position control restored.')

    @property
    def is_faulted(self) -> bool:
        """Return True if the safety monitor has a latched fault."""
        return self._safety.is_faulted

    def reset_fault(self) -> None:
        """Clear the latched safety fault; call via ~/reset_fault service."""
        self._safety.reset()

    def close(self) -> None:
        """Release all Kortex connections (call on node shutdown)."""
        self.exit()
        if FAKE_HARDWARE:
            return
        try:
            self._tcp_session.CloseSession()
        except Exception:
            pass
        try:
            self._tcp_transport.disconnect()
        except Exception:
            pass
        try:
            self._udp_transport.disconnect()
        except Exception:
            pass

    # ── Kortex mode transitions ───────────────────────────────────────────

    def _enter_low_level_torque(self) -> None:
        """Switch to LOW_LEVEL_SERVOING and set all actuators to TORQUE mode."""
        mode = Base_pb2.ServoingModeInformation()
        mode.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
        self._base.SetServoingMode(mode)

        for dev_id in range(1, N_JOINTS + 1):  # device IDs are 1-indexed
            cmi = ActuatorConfig_pb2.ControlModeInformation()
            cmi.control_mode = ActuatorConfig_pb2.TORQUE
            self._act_cfg.SetControlMode(cmi, device_id=dev_id)

        self._log.info('LOW_LEVEL_SERVOING + TORQUE mode active on all actuators.')

    def _safe_exit(self) -> None:
        """
        Zero torques, switch to POSITION mode at current angles, return to
        SINGLE_LEVEL_SERVOING.

        Call on clean exit AND on any fault or exception — this is the
        gravity-drop protection.
        """
        try:
            feedback = self._cyclic.RefreshFeedback()

            # Build zero-torque command
            zero_cmd = BaseCyclic_pb2.Command()
            for i, fb in enumerate(feedback.actuators):
                act = zero_cmd.actuators.add()
                act.command_id = int(fb.command_id)
                act.flags = 0
                act.position = fb.position
                act.velocity = 0.0
                act.torque_joint = 0.0

            # Send a handful of zero-torque frames so the arm comes to rest
            for _ in range(5):
                feedback = self._cyclic.Refresh(zero_cmd)
                time.sleep(0.005)

            # Switch all actuators to POSITION mode
            for dev_id in range(1, N_JOINTS + 1):
                cmi = ActuatorConfig_pb2.ControlModeInformation()
                cmi.control_mode = ActuatorConfig_pb2.POSITION
                self._act_cfg.SetControlMode(cmi, device_id=dev_id)

            # Hold at current angles
            hold_cmd = BaseCyclic_pb2.Command()
            for i, fb in enumerate(feedback.actuators):
                act = hold_cmd.actuators.add()
                act.command_id = int(fb.command_id) + 1
                act.flags = 0
                act.position = fb.position
            self._cyclic.Refresh(hold_cmd)
            time.sleep(0.05)

        except Exception as exc:
            self._log.error(f'_safe_exit inner error: {exc}')

        # Always attempt to return to high-level mode, even if above failed
        try:
            mode = Base_pb2.ServoingModeInformation()
            mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self._base.SetServoingMode(mode)
        except Exception as exc:
            self._log.error(f'_safe_exit: SetServoingMode failed: {exc}')

    # ── control loop ──────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        """
        Run the torque control loop at ~200 Hz in a dedicated thread.

        On any exit path (clean stop, fault, or exception) the arm is
        returned to position hold before the thread terminates.
        """
        # ── initialise command structure with current feedback ─────────────
        try:
            feedback = self._cyclic.RefreshFeedback()
        except Exception as exc:
            self._log.error(f'Control loop: initial feedback failed: {exc}')
            self._safe_exit()
            self._running = False
            return

        cmd = BaseCyclic_pb2.Command()
        for fb in feedback.actuators:
            act = cmd.actuators.add()
            act.command_id = int(fb.command_id)
            act.flags = 0
            act.position = fb.position
            act.velocity = 0.0
            act.torque_joint = 0.0

        last_t = time.perf_counter()

        try:
            while not self._stop_event.is_set():
                t0 = time.perf_counter()
                dt = t0 - last_t
                last_t = t0

                # Guard against zero or negative dt on first iteration
                dt = max(dt, 1e-6)

                # ── read state from feedback ──────────────────────────────
                # Kortex positions → degrees, velocities → deg/s, torque → Nm
                q_deg = np.array([fb.position for fb in feedback.actuators])
                v_degs = np.array([fb.velocity for fb in feedback.actuators])
                tau_meas = np.array([fb.torque for fb in feedback.actuators])

                q_rad = q_deg * _DEG2RAD
                v_rad = v_degs * _DEG2RAD

                # ── gravity compensation ──────────────────────────────────
                g_q = self._gravity(q_rad)

                # ── viscous damping ───────────────────────────────────────
                b_qdot = self._b * v_rad

                # ── friction observer ─────────────────────────────────────
                tau_hat_f = self._observer.update(tau_meas, g_q, b_qdot, dt)

                # ── control law ───────────────────────────────────────────
                tau_cmd = g_q - tau_hat_f - b_qdot

                # ── safety: clamp and check ───────────────────────────────
                tau_safe, faulted, reason = self._safety.check_and_clamp(
                    tau_cmd, v_rad, dt
                )
                if faulted:
                    self._log.error(f'Safety fault: {reason}')
                    self._safe_exit()
                    break

                # ── send command ──────────────────────────────────────────
                for i, act in enumerate(cmd.actuators):
                    act.command_id = int(feedback.actuators[i].command_id)
                    act.position = feedback.actuators[i].position
                    act.velocity = 0.0
                    act.torque_joint = float(tau_safe[i])

                feedback = self._cyclic.Refresh(cmd)

                # ── rate limiting ─────────────────────────────────────────
                elapsed = time.perf_counter() - t0
                sleep_time = self._dt_nominal - elapsed
                if sleep_time > 1e-4:
                    time.sleep(sleep_time)

            else:
                # Clean stop (stop_event was set)
                self._safe_exit()

        except Exception as exc:
            self._safety.trip_exception(exc)
            self._log.error(f'Control loop exception: {exc}')
            try:
                self._safe_exit()
            except Exception as inner:
                self._log.error(f'Safe exit after exception failed: {inner}')

        finally:
            self._running = False

    def _fake_loop(self) -> None:
        """
        Physics-simulated control loop for FAKE_HARDWARE mode.

        Runs the same control law as _control_loop against a Pinocchio
        forward-dynamics (ABA) plant.  Publishes /joint_states so RViz
        animates the result.

        Synthetic plant friction (Coulomb + viscous) gives the friction
        observer something non-trivial to estimate and cancel.

        Subscribe to /fake_disturbance (std_msgs/Float64MultiArray, 7 floats,
        Nm) to inject external joint torques and simulate hand-guiding.
        External torques drive the forward dynamics but are NOT fed to the
        observer, so the arm responds to pushes rather than cancelling them.
        """
        import pinocchio as pin
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray

        # Synthetic plant friction per joint: rough Gen3 values.
        # Coulomb (Nm) uses tanh smoothing to avoid chattering at v≈0.
        _coulomb = np.array([0.30, 0.30, 0.20, 0.20, 0.15, 0.15, 0.10])
        _viscous_plant = np.array([0.05] * N_JOINTS)
        _v_eps = 0.01  # rad/s — tanh smoothing threshold

        # External disturbance (hand-push simulation via /fake_disturbance)
        _tau_ext = np.zeros(N_JOINTS)
        _tau_ext_lock = threading.Lock()

        def _disturbance_cb(msg: Float64MultiArray) -> None:
            if len(msg.data) == N_JOINTS:
                with _tau_ext_lock:
                    _tau_ext[:] = msg.data

        _pub = self._node.create_publisher(JointState, '/joint_states', 10)
        self._node.create_subscription(
            Float64MultiArray, '/fake_disturbance', _disturbance_cb, 10
        )

        # Start at neutral configuration (all joints at 0 rad)
        q = pin.neutral(self._pin_model)
        v = np.zeros(self._pin_model.nv)

        last_t = time.perf_counter()

        try:
            while not self._stop_event.is_set():
                t0 = time.perf_counter()
                dt = max(t0 - last_t, 1e-6)
                last_t = t0

                # ── state ─────────────────────────────────────────────────
                q_rad = _q_pin_to_angles(q)
                v_rad = v.copy()

                with _tau_ext_lock:
                    tau_ext = _tau_ext.copy()

                # ── gravity compensation ──────────────────────────────────
                g_q = self._gravity(q_rad)

                # ── synthetic plant friction ──────────────────────────────
                # tanh smoothing avoids chattering at v ≈ 0.
                tau_fric_plant = (
                    -_coulomb * np.tanh(v_rad / _v_eps)
                    - _viscous_plant * v_rad
                )

                # ── synthesise torque-sensor reading ──────────────────────
                # External disturbances are excluded so the observer tracks
                # friction rather than cancelling hand-pushes.
                b_qdot = self._b * v_rad
                tau_measured = g_q + b_qdot + tau_fric_plant

                # ── friction observer ─────────────────────────────────────
                tau_hat_f = self._observer.update(tau_measured, g_q, b_qdot, dt)

                # ── control law ───────────────────────────────────────────
                tau_cmd = g_q - tau_hat_f - b_qdot

                # ── safety check ──────────────────────────────────────────
                tau_safe, faulted, reason = self._safety.check_and_clamp(
                    tau_cmd, v_rad, dt
                )
                if faulted:
                    self._log.error(f'[FAKE] Safety fault: {reason}')
                    break

                # ── forward dynamics ──────────────────────────────────────
                # ABA subtracts g(q) and C(q,v)*v internally; pass the full
                # generalised force so the plant sees friction + disturbance.
                tau_aba = tau_safe + tau_ext + tau_fric_plant
                q_ddot = pin.aba(
                    self._pin_model, self._pin_data, q, v, tau_aba
                )

                # ── integrate (pin.integrate handles RUBZ wrapping) ───────
                v = v + q_ddot * dt
                q = pin.integrate(self._pin_model, q, v * dt)

                # ── publish /joint_states ─────────────────────────────────
                q_rad_out = _q_pin_to_angles(q)
                js = JointState()
                js.header.stamp = self._node.get_clock().now().to_msg()
                js.name = [f'joint_{i + 1}' for i in range(N_JOINTS)]
                js.position = q_rad_out.tolist()
                js.velocity = v_rad.tolist()
                js.effort = tau_safe.tolist()
                _pub.publish(js)

                # ── rate limiting ─────────────────────────────────────────
                elapsed = time.perf_counter() - t0
                sleep_time = self._dt_nominal - elapsed
                if sleep_time > 1e-4:
                    time.sleep(sleep_time)

        finally:
            self._running = False

    # ── gravity compensation ──────────────────────────────────────────────

    def _gravity(self, q_rad: np.ndarray) -> np.ndarray:
        """
        Return gravity compensation torques via Pinocchio.

        Parameters
        ----------
        q_rad : joint angles in radians, shape (7,)

        Returns
        -------
        g_q : gravity torques, shape (7,), Nm

        Notes
        -----
        gravity_sign (per joint, default +1) corrects sign mismatches
        between Kortex encoder conventions and the URDF definition.
        Verify on hardware: with gravity_sign all +1, commanding only
        g_q with no motion should hold each joint stationary.  Flip the
        sign for any joint that drifts.
        """
        import pinocchio as pin  # lazy import — Pinocchio not needed at import time

        q_pin = _build_q_pin(q_rad)
        g = pin.computeGeneralizedGravity(self._pin_model, self._pin_data, q_pin)
        return self._gravity_sign * g

    def _ensure_pinocchio(self) -> None:
        """Load the Pinocchio Gen3 model if not already loaded."""
        if self._pin_model is not None:
            return

        import pinocchio as pin

        urdf_content = self._get_urdf()
        if urdf_content is None:
            raise RuntimeError(
                'Cannot load Gen3 URDF.  Set urdf_path param or ensure '
                'kortex_description is installed and xacro is on PATH.'
            )

        with tempfile.NamedTemporaryFile(
            suffix='.urdf', mode='w', delete=False
        ) as fh:
            fh.write(urdf_content)
            tmp = fh.name

        try:
            self._pin_model = pin.buildModelFromUrdf(tmp)
            self._pin_data = self._pin_model.createData()
        finally:
            try:
                os.unlink(tmp)
            except OSError:
                pass

        self._log.info(
            f'Pinocchio model loaded: nv={self._pin_model.nv}, '
            f'njoints={self._pin_model.njoints - 1}.'
        )

    def _get_urdf(self) -> Optional[str]:
        """
        Obtain the Gen3 URDF as a string.

        Priority
        --------
        1. robot_description ROS parameter on this node (set by launch file)
        2. urdf_path parameter (explicit filesystem path)
        3. Run xacro on kortex_description (requires xacro on PATH)
        """
        # Option 1: robot_description ROS parameter
        try:
            from rclpy.parameter import Parameter
            p = self._node.get_parameter('robot_description')
            if p.type_ != Parameter.Type.NOT_SET and p.value:
                self._log.info('Pinocchio URDF: using robot_description param.')
                return str(p.value)
        except Exception:
            pass

        # Option 2: explicit urdf_path
        if self._urdf_path and os.path.isfile(self._urdf_path):
            self._log.info(f'Pinocchio URDF: using urdf_path={self._urdf_path}')
            with open(self._urdf_path) as fh:
                return fh.read()

        # Option 3: run xacro on kortex_description share directory
        try:
            from ament_index_python.packages import get_package_share_directory
            kortex_share = get_package_share_directory('kortex_description')
            xacro_file = os.path.join(kortex_share, 'robots', 'gen3.xacro')
            if os.path.isfile(xacro_file):
                result = subprocess.run(
                    ['xacro', xacro_file, 'dof:=7', 'vision:=false'],
                    capture_output=True,
                    text=True,
                    timeout=15,
                )
                if result.returncode == 0:
                    self._log.info('Pinocchio URDF: generated via xacro.')
                    return result.stdout
                self._log.warn(f'xacro failed (rc={result.returncode}): {result.stderr[:200]}')
        except Exception as exc:
            self._log.warn(f'xacro fallback failed: {exc}')

        return None
