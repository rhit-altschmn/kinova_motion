"""
fake_joint_states.py

This publishes synthetic /joint_states for offline testing.

Each of the 7 joints follows a slow sinusoidal trajectory so the recorder
captures a non-trivial waypoint sequence even without a physical robot.

Usage:
  FAKE_HARDWARE=1 ros2 run rec_rep2 fake_joint_states
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Must match JOINT_NAMES in recorder.py exactly.
JOINT_NAMES = [
    'joint_1', 'joint_2', 'joint_3', 'joint_4',
    'joint_5', 'joint_6', 'joint_7',
]

# (amplitude_rad, period_s): each joint oscillates independently
_MOTION = [
    (0.30, 4.0),
    (0.40, 5.0),
    (0.20, 3.5),
    (0.50, 6.0),
    (0.30, 4.5),
    (0.20, 3.0),
    (0.10, 7.0),
]

PUBLISH_HZ = 50


class FakeJointStates(Node):
    """Publishes sinusoidal joint positions to /joint_states at PUBLISH_HZ."""

    def __init__(self):
        super().__init__('fake_joint_states')
        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self._t0 = time.time()
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self.get_logger().info(
            f'Fake joint states publishing at {PUBLISH_HZ} Hz on /joint_states'
        )

    def _publish(self):
        t = time.time() - self._t0
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [
            amp * math.sin(2.0 * math.pi * t / period)
            for amp, period in _MOTION
        ]
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FakeJointStates())
    rclpy.shutdown()
