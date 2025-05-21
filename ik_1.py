import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Vector3
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class IK(Node):
    def __init__(self):
        Node.__init__(self, "ik_1")
        self.goal_received = False

        self._action_client = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_position_controller/follow_joint_trajectory")
        self.get_logger().info("Waiting for action server to be ready...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server ready!")

        self.load_params()
        self.create_subscription(Vector3, "/goal", self.goal_cb, 1)
        self.create_timer(0.1, self.execute)
    
    def load_params(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("arm_lengths", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("lower_limits", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("upper_limits", rclpy.Parameter.Type.DOUBLE_ARRAY),
            ]
        )
        self.l1, self.l2, self.l3, self.l4 = self.get_parameter("arm_lengths").value
        self.lower_limits = self.get_parameter("lower_limits").value
        self.upper_limits = self.get_parameter("upper_limits").value
    
    def goal_cb(self, msg):
        self.x_d, self.y_d, self.z_d = msg.x, msg.y, msg.z
        self.goal_received = True
    
    def check_limits(self):
        if not (self.lower_limits[0] <= self.t1 <= self.upper_limits[0]):
            return False
        if not (self.lower_limits[1] <= self.t2 <= self.upper_limits[1]):
            return False
        if not (self.lower_limits[2] <= self.t3 <= self.upper_limits[2]):
            return False
        return True
    
    def execute(self):
        if self.goal_received:
            self.ik()
            self.goal_received = False
    
    def ik(self):
        x, y, z = self.x_d, self.y_d, self.z_d

        # Compute theta1
        if x == 0:
            self.t1 = np.pi / 2 if y > 0 else -np.pi / 2
        else:
            self.t1 = np.arctan2(y, x)

        # Compute theta3
        L34 = self.l3 + self.l4
        numerator = x**2 + y**2 + z**2 - L34**2 - self.l2**2
        denominator = 2 * L34 * self.l2
        if denominator == 0:
            self.get_logger().error("Denominator for theta3 is zero, cannot compute.")
            return
        cos_theta3 = numerator / denominator
        if cos_theta3 < -1 or cos_theta3 > 1:
            self.get_logger().error(f"cos(theta3) = {cos_theta3} is out of range [-1, 1], position may be unreachable.")
            return
        self.t3 = -np.arccos(cos_theta3)

        # Compute theta2
        term1 = x * np.cos(self.t1) + y * np.sin(self.t1)
        denominator = self.l2 + L34 * np.cos(self.t3)
        if denominator == 0:
            self.get_logger().error("Denominator for theta2 is zero, cannot compute.")
            return

        sin_theta2 = (z - (self.l1 + L34 * np.sin(self.t3))) / denominator
        if sin_theta2 < -1 or sin_theta2 > 1:
            self.get_logger().error(f"sin(theta2) = {sin_theta2} is out of range [-1, 1], position may be unreachable.")
            return
        self.t2 = np.arcsin(sin_theta2)

        self.get_logger().info(f"t1: {self.t1:.2f}, t2: {self.t2:.2f}, t3: {self.t3:.2f}")

        if self.check_limits():
            self.get_logger().info("Sending joint values")
            self.send_joints()
    
    def send_joints(self):
        goal_msg = FollowJointTrajectory.Goal()

        joint_names = ["base_to_1", "1_to_2", "2_to_3"]

        points = []
        point1 = JointTrajectoryPoint()
        point1.time_from_start = Duration(seconds=5, nanoseconds=0).to_msg()
        point1.positions = [self.t1, self.t2, self.t3]
        points.append(point1)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = IK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()