import sys
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class GoalSender(Node):
    def __init__(self):
        Node.__init__(self, "send_goal_actionclient")
        self._action_client = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_position_controller/follow_joint_trajectory")
    
    def send_goal(self, angles):
        goal_msg = FollowJointTrajectory.Goal()

        joint_names = ["base_to_1", "1_to_2", "2_to_3"]

        points = []
        point1 = JointTrajectoryPoint()
        point1.time_from_start = Duration(seconds=5, nanoseconds=0).to_msg()    #reach goal in 5s
        point1.positions = angles
        points.append(point1)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self.get_logger().info("Waiting for action server to be ready...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server ready!")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    node = GoalSender()

    try:
        angles = [float(sys.argv[i]) for i in range(1, 4)]
        node.send_goal(angles)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()