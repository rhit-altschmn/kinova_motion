"""
fake_replayer.py
This animates a recorded trajectory on /joint_states.

Used in FAKE_HARDWARE mode for no ros2_control stack: robot_state_publisher
consumes /joint_states and RViz2 animates the robot model in real time.

Usage:
  ros2 run rec_rep2 fake_replayer <path_to_bag> [speed_factor]
"""

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from classic_bags import Bag


class FakeReplayer(Node):
    """Publishes time-sequenced JointState messages from a recorded JSON."""

    def __init__(self):
        super().__init__('fake_replayer')
        self._pub = self.create_publisher(JointState, '/joint_states', 10)

    def replay(self, filepath: str, speed: float = 1.0):
        """
        filepath : path to bag directory from recorder
        speed    : 1.0 = real-time, 2.0 = double speed, etc.
        """
        waypoints = []
        t0_ns = None
        with Bag(filepath) as bag:
            for _, msg, ts in bag.read_messages('/joint_states'):
                if t0_ns is None:
                    t0_ns = int(ts)
                waypoints.append(((int(ts) - t0_ns) / 1e9, msg))

        if not waypoints:
            self.get_logger().error('Empty bag. There is nothing to replay!')
            return

        duration = waypoints[-1][0] / speed
        self.get_logger().info(
            f'Replaying {len(waypoints)} waypoints '
            f'(duration: {duration:.1f}s at {speed}x) ...'
        )

        t_start = time.time()
        for t_elapsed, src in waypoints:
            target = t_start + t_elapsed / speed
            now = time.time()
            if target > now:
                time.sleep(target - now)

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(src.name)
            msg.position = list(src.position)
            msg.velocity = [0.0] * len(src.name)
            msg.effort = [0.0] * len(src.name)
            self._pub.publish(msg)

        self.get_logger().info('Replay complete.')


def main(args=None):
    rclpy.init(args=args)
    node = FakeReplayer()
    if len(sys.argv) < 2:
        node.get_logger().error(
            'Usage: fake_replayer <path_to_bag> [speed_factor]'
        )
        rclpy.shutdown()
        return
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    time.sleep(0.5)  # allow publisher to connect
    node.replay(sys.argv[1], speed)
    rclpy.shutdown()
