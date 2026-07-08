import rclpy
import json
import sys
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# This topic is consumed by the joint_trajectory_controller.
# Verify with: ros2 topic list | grep trajectory
# Some kortex setups prefix with /gen3/ -- check your setup.
CONTROLLER_TOPIC = (
    '/joint_trajectory_controller/joint_trajectory'
)

class MotionReplayer(Node):
    """
    Reads a JSON trajectory produced by MotionRecorder and
    publishes it to the joint_trajectory_controller.

    The controller accepts trajectory_msgs/JointTrajectory
    over a topic or via the FollowJointTrajectory action.
    We use the simpler topic interface here.

    Reference:
      control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html 
      https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html 
    """

    def __init__(self):
        super().__init__('motion_replayer')
        self.pub = self.create_publisher(
            JointTrajectory,
            CONTROLLER_TOPIC,
            10,
        )
        self.get_logger().info('Motion Replayer ready.')

        # --- Continued inside MotionReplayer class ---

    def replay(self, filepath: str, speed: float = 1.0):
        """
        filepath : path to JSON file from recorder
        speed    : 1.0 = real-time, 0.5 = half, 2.0 = double
                   Speed scaling works by dividing each waypoint
                   timestamp by the speed factor.
        """
        with open(filepath) as f:
            data = json.load(f)

        traj = JointTrajectory()
        traj.joint_names = data['joints']

        for wp in data['trajectory']:
            pt = JointTrajectoryPoint()
            pt.positions = wp['q']   # 7 joint angles in radians

            # Scale timestamp
            t = wp['t'] / speed
            pt.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t - int(t)) * 1_000_000_000),
            )
            traj.points.append(pt)

        self.get_logger().info(
            f'Publishing {len(traj.points)} waypoints '
            f'(duration: {traj.points[-1].time_from_start.sec:.1f}s '
            f'at {speed}x speed)...'
        )
        self.pub.publish(traj)
        self.get_logger().info('Trajectory published.')


def main(args=None):
    rclpy.init(args=args)
    node = MotionReplayer()
    if len(sys.argv) < 2:
        node.get_logger().error(
            'Usage: replayer <path_to_json> [speed_factor]'
        )
        return
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    import time; time.sleep(1)  # wait for publisher to connect
    node.replay(sys.argv[1], speed)
    rclpy.shutdown()