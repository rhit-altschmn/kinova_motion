import rclpy
import sys
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from classic_bags import Bag

# This topic is consumed by the joint_trajectory_controller.
# Verify with: ros2 topic list | grep trajectory
# Apparently some kortex setups prefix with /gen3/ so todo=double check setup.
CONTROLLER_TOPIC = (
    '/joint_trajectory_controller/joint_trajectory'
)

class MotionReplayer(Node):
<<<<<<< HEAD
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

=======
    # see readme for notes on class
    
>>>>>>> cad5274a15c99311e866ab9d185d34479f091ba4
    def __init__(self):
        super().__init__('motion_replayer')
        self.pub = self.create_publisher(
            JointTrajectory,
            CONTROLLER_TOPIC,
            10,
        )
        self.get_logger().info('Motion Replayer ready.')

    def replay(self, filepath: str, speed: float = 1.0):
        """
        filepath : path to bag directory from recorder
        speed    : 1.0 = real-time, 0.5 = half, 2.0 = double
                   Speed scaling works by dividing each waypoint
                   timestamp by the speed factor.
        """
        traj = JointTrajectory()
        t0_ns = None

        with Bag(filepath) as bag:
            for _, msg, ts in bag.read_messages('/joint_states'):
                if t0_ns is None:
                    t0_ns = int(ts)
                    traj.joint_names = list(msg.name)
                t = (int(ts) - t0_ns) / (speed * 1e9)
                pt = JointTrajectoryPoint()
                pt.positions = list(msg.position)
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
            'Usage: replayer <path_to_bag> [speed_factor]'
        )
        return
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    import time; time.sleep(1)  # wait for publisher to connect
    node.replay(sys.argv[1], speed)
    rclpy.shutdown()