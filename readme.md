## recorder.py


## replayer.py
### MotionReplayer class
Reads a JSON trajectory produced by MotionRecorder and
    publishes it to the joint_trajectory_controller.

    The controller accepts trajectory_msgs/JointTrajectory
    over a topic or via the FollowJointTrajectory action.
    We use the simpler topic interface here.

    Reference:
      control.ros.org/humble/doc/ros2_controllers/
        joint_trajectory_controller/doc/userdoc.html

## compliant_torque_mode.py

### Control per joint/cycle

    tau_cmd = g(q)  -  tau_hat_friction  -  b * q_dot

  g(q)             gravity compensation torques (Pinocchio, Gen3 URDF)
  tau_hat_friction  friction observer estimate (FrictionObserver)
  b * q_dot        viscous damping to damp free swinging after release

### Kortex API sequence
Enter
    1. SetServoingMode -> LOW_LEVEL_SERVOING
    2. SetControlMode  -> TORQUE per actuator w/ device_id 1-7
    3. Start control-loop thread; call BaseCyclic.Refresh() at ~200 Hz

Exit (clean or fault)
    1. Send zero-torque frames (stop moving)
    2. SetControlMode -> POSITION  (per actuator)
    3. Refresh with current positions (position hold)
    4. SetServoingMode -> SINGLE_LEVEL_SERVOING

On any unhandled exception the same exit sequence runs from the
except clause to avoid arm crashing due to gravity

FAKE_HARDWARE=1 skips all gRPC calls; the control loop runs as a no-op
thread so the rest of the code behaves identically.

### Gravity-compensation notes

The Gen3 URDF has alternating RUBZ (unbounded revolute) and RZ joints.
Pinocchio's nq=11, nv=7 for this model.  Positions in RUBZ joints are
stored as [cos θ, sin θ] to avoid wrapping. 
The helper _build_q_pin() converts the 7-element angle vector accordingly.

Kortex API positions and velocities are in degrees / degrees-per-second.
They are converted to radians before being fed into Pinocchio or the
observer.

The gravity_sign parameter (per joint, default all +1.0) corrects for
any mismatch between Kortex encoder sign conventions and the URDF

## safety_monitor.py

Safety monitor for the compliant torque control loop.

All checks execute inside the control loop before every torque command.

After Any safety trip, this will execute the following steps:
  1. Returns a zero-torque vector (arm goes to gravity + inertia).
  2. Latches the monitor into a faulted state.
  3. Logs the offending joint index, measured value, and threshold.

The faulted state must be explicitly cleared via reset() (which is exposed as the `~/reset_fault` service on the recorder node).  This makes sure a human confirms the fault before the arm can re-enter torque mode.

Fault conditions
----------------
VELOCITY_TRIP
    Any joint's angular velocity exceeds its per-joint watchdog limit.
    Will be caused by the user letting go and a joint(s) dropping under gravity, or some kind of instability developing.

LOOP_OVERRUN:
    The control loop missed N consecutive cycles (each cycle took longer than dt_nominal * overrun_factor). Potential likely cause: OS scheduling
    jitter, or Python GIL contention. The Kortex firmware does also fault the arm independently if refresh calls stop for ~100 ms, but this
    fires first.

EXCEPTION:
    An unhandled Python exception inside the control loop. This is recorded so that the fault message is visible in the GUI log.


Torque saturation
-----------------
    Per-joint |tau_cmd| > tau_limit is silently clamped and doesnt cause a fault.
    Conservative 40 % of rated torques used by default.

### SafetyMonitor class
Check torque commands and joint velocities/clamp torques/latch faults.

    Parameters
    ----------
    tau_limits        : per-joint torque saturation limits (Nm, positive),
                        length n_joints
    vel_limits        : per-joint velocity watchdog thresholds (rad/s,
                        positive), length n_joints
    max_missed_cycles : consecutive overrun cycles that trigger a fault
    dt_nominal        : expected control loop period (seconds)
    overrun_factor    : dt > dt_nominal * overrun_factor counts as a miss
    logger            : rclpy Logger or stdlib Logger (optional)