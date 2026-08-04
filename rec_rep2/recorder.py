import os
import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from classic_bags import Bag
from .compliant_mode import CompliantModeManager
from .compliant_torque_mode import CompliantTorqueMode

class MotionRecorder(Node):
    """
    Records Gen3 joint positions during hand-guided movement.

    Services exposed:
      ~/start_recording  (std_srvs/Trigger) <-- enables admittance,
                                                starts buffering
      ~/stop_recording   (std_srvs/Trigger) <-- stops buffering,
                                                saves JSON, re-enables
                                                position control
    """

    def __init__(self):
        super().__init__('motion_recorder')
        self.recording      = False
        self.trajectory     = []   # list of (t_ns: int, JointState msg)
        self.t0             = None
        self._last_js_stamp = None  # updated every /joint_states message

        # Declare parameters consumed by CompliantTorqueMode.
        # All have safe defaults; override via a --params-file yaml.
        self.declare_parameter('robot_description', '')
        self.declare_parameter('compliant_control_rate_hz', 200)
        self.declare_parameter('compliant_observer_L',
                               [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0])
        self.declare_parameter('compliant_observer_Lp',
                               [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('compliant_damping_b',
                               [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
        self.declare_parameter('compliant_torque_limits',
                               [22.4, 22.4, 22.4, 11.2, 11.2, 11.2, 11.2])
        self.declare_parameter('compliant_velocity_limits_rad_s',
                               [0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8])
        self.declare_parameter('compliant_max_missed_cycles', 10)
        self.declare_parameter('compliant_gravity_sign',
                               [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('compliant_urdf_path', '')

        # Subscribe to joint states published by the driver
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_cb,
            qos_profile=qos_profile_sensor_data,  # BEST_EFFORT matches ros2_controllers
        )

        # Expose start/stop as ROS2 Trigger services
        self.start_srv = self.create_service(
            Trigger, '~/start_recording', self._start_cb,
        )
        self.stop_srv = self.create_service(
            Trigger, '~/stop_recording', self._stop_cb,
        )

        # Posing mode selector: True → compliant_torque, False → admittance
        self.set_mode_srv = self.create_service(
            SetBool, '~/set_posing_mode', self._set_mode_cb,
        )

        # Torque-sensor bias confirmation gate
        self.confirm_bias_srv = self.create_service(
            Trigger, '~/confirm_bias_zeroed', self._confirm_bias_cb,
        )

        # Safety-fault reset
        self.reset_fault_srv = self.create_service(
            Trigger, '~/reset_fault', self._reset_fault_cb,
        )

        self._posing_mode = 'admittance'   # 'admittance' | 'compliant_torque'
        self._bias_zeroed = False          # reset each node restart

        self.mode_mgr = CompliantModeManager()
        self.torque_mode = CompliantTorqueMode(self, self._load_compliant_params())
        self.get_logger().info('Motion Recorder ready.')

    def _joint_cb(self, msg: JointState):
        """Buffer one waypoint per received JointState message."""
        self._last_js_stamp = time.time()
        if not self.recording:
            return
        t_ns = int((time.time() - self.t0) * 1e9)
        self.trajectory.append((t_ns, msg))

    def _load_compliant_params(self):
        """Build the params dict consumed by CompliantTorqueMode."""
        g = self.get_parameter
        return {
            'control_rate_hz':       g('compliant_control_rate_hz').value,
            'observer_L':            g('compliant_observer_L').value,
            'observer_Lp':           g('compliant_observer_Lp').value,
            'damping_b':             g('compliant_damping_b').value,
            'torque_limits':         g('compliant_torque_limits').value,
            'velocity_limits_rad_s': g('compliant_velocity_limits_rad_s').value,
            'max_missed_cycles':     g('compliant_max_missed_cycles').value,
            'gravity_sign':          g('compliant_gravity_sign').value,
            'urdf_path':             g('compliant_urdf_path').value,
        }

    def _set_mode_cb(self, req, resp):
        """Switch posing mode: True → compliant_torque, False → admittance."""
        if self.recording:
            resp.success = False
            resp.message = 'Cannot switch posing mode while recording is active.'
            return resp
        self._posing_mode = 'compliant_torque' if req.data else 'admittance'
        resp.success = True
        resp.message = f'Posing mode set to: {self._posing_mode}'
        self.get_logger().info(resp.message)
        return resp

    def _confirm_bias_cb(self, req, resp):
        """Confirm that torque sensor bias has been zeroed via web app."""
        self._bias_zeroed = True
        resp.success = True
        resp.message = 'Torque sensor bias confirmed zeroed for this session.'
        self.get_logger().info(resp.message)
        return resp

    def _reset_fault_cb(self, req, resp):
        """Clear latched safety fault so torque mode can be re-entered."""
        self.torque_mode.reset_fault()
        resp.success = True
        resp.message = 'Safety fault cleared.'
        self.get_logger().info(resp.message)
        return resp

    def _start_cb(self, req, resp):
        # Refuse to start if /joint_states has gone stale (controller crash).
        stale = (
            self._last_js_stamp is None
            or (time.time() - self._last_js_stamp) > 1.0
        )
        if stale:
            resp.success = False
            resp.message = (
                '/joint_states not arriving (no message in >1 s). '
                'Check that joint_state_broadcaster is active: '
                'ros2 topic hz /joint_states'
            )
            return resp

        if self._posing_mode == 'compliant_torque':
            if not self._bias_zeroed:
                resp.success = False
                resp.message = (
                    'Torque sensor bias not confirmed.  '
                    'Put arm in candlestick pose, zero via Kinova web app, '
                    'then call ~/confirm_bias_zeroed.'
                )
                return resp
            if self.torque_mode.is_faulted:
                resp.success = False
                resp.message = (
                    'Torque mode has a latched safety fault.  '
                    'Investigate the fault, then call ~/reset_fault.'
                )
                return resp
            if not self.torque_mode.enter():
                resp.success = False
                resp.message = 'Failed to enter compliant torque mode.  Check robot connection.'
                return resp
        else:
            self.mode_mgr.enable()    # arm becomes compliant here

        self.trajectory.clear()
        self.t0 = time.time()
        self.recording = True
        resp.success = True
        resp.message = (
            f'Recording started ({self._posing_mode} mode). '
            'Move the arm by hand!'
        )
        return resp

    def _stop_cb(self, req, resp):
        self.recording = False
        if self._posing_mode == 'compliant_torque':
            self.torque_mode.exit()   # safe shutdown: position hold first
        else:
            self.mode_mgr.disable()   # back to position control

        save_dir = '/home/horrorfry/ros2_ws/src/rec_rep2/bags'
        os.makedirs(save_dir, exist_ok=True)

        fname = os.path.join(save_dir, f'recorded_motion_{int(time.time())}.bag')
        with Bag(fname, 'w') as bag:
            for t_ns, msg in self.trajectory:
                bag.write('/joint_states', msg, t_ns)

        n = len(self.trajectory)
        resp.success = True
        resp.message = f'Saved {n} waypoints to {fname}'
        self.get_logger().info(resp.message)

        # Return to home in the background so the service response is not blocked.
        # 1 s delay lets the ros2_kortex driver re-establish position control
        # before the issue of a new high-level gRPC command.
        threading.Thread(target=self._go_home_deferred, daemon=True).start()

        return resp

    def _go_home_deferred(self):
        time.sleep(1.0)
        try:
            self.mode_mgr.go_home()
        except Exception as exc:
            self.get_logger().warn(f'go_home failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = MotionRecorder()
    try:
        rclpy.spin(node)
    finally:
        # Ensure torque mode exits cleanly (position hold) before shutdown.
        node.torque_mode.close()
        node.destroy_node()
    rclpy.shutdown()