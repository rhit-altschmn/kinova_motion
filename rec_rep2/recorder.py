import rclpy
import json
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from .compliant_mode import CompliantModeManager

# Joint names must match exactly what /joint_states publishes.
# Verify with: ros2 topic echo /joint_states --once
JOINT_NAMES = [
    'joint_1', 'joint_2', 'joint_3', 'joint_4',
    'joint_5', 'joint_6', 'joint_7',
]

class MotionRecorder(Node):
    """
    Records Gen3 joint positions during hand-guided movement.

    Services exposed:
      ~/start_recording  (std_srvs/Trigger) -- enables admittance,
                                               starts buffering
      ~/stop_recording   (std_srvs/Trigger) -- stops buffering,
                                               saves JSON, re-enables
                                               position control
    """

    def __init__(self):
        super().__init__('motion_recorder')
        self.recording  = False
        self.trajectory = []   # list of {t: float, q: [float x7]}
        self.t0         = None

        # Subscribe to joint states published by the driver
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_cb,
            qos_profile=10,  # keep-last 10
        )

        # Expose start/stop as ROS2 Trigger services
        self.start_srv = self.create_service(
            Trigger, '~/start_recording', self._start_cb,
        )
        self.stop_srv = self.create_service(
            Trigger, '~/stop_recording', self._stop_cb,
        )

        self.mode_mgr = CompliantModeManager()
        self.get_logger().info('Motion Recorder ready.')

        def _joint_cb(self, msg: JointState):
            """Buffer one waypoint per received JointState message."""
            if not self.recording:
                return
            t = time.time() - self.t0
            # msg.position is a tuple of 7 floats in radians
            self.trajectory.append({
                't': t,
                'q': list(msg.position),
        })

    def _start_cb(self, req, resp):
        self.mode_mgr.enable()    # arm becomes compliant here
        self.trajectory.clear()
        self.t0 = time.time()
        self.recording = True
        resp.success = True
        resp.message = 'Recording started -- move the arm by hand!'
        return resp

    def _stop_cb(self, req, resp):
        self.recording = False
        self.mode_mgr.disable()   # back to position control

        fname = f'/tmp/recorded_motion_{int(time.time())}.json'
        with open(fname, 'w') as f:
            json.dump(
                {'joints': JOINT_NAMES, 'trajectory': self.trajectory},
                f, indent=2,
            )

        n = len(self.trajectory)
        resp.success = True
        resp.message = f'Saved {n} waypoints to {fname}'
        self.get_logger().info(resp.message)
        return resp


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MotionRecorder())
    rclpy.shutdown()