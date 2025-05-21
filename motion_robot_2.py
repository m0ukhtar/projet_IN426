import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class Robot(Node):
    def __init__(self):
        Node.__init__(self, "motion_robot_2")

        self._action_client = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_position_controller/follow_joint_trajectory")
        self.get_logger().info("Waiting for action server to be ready...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server ready!")

        self.load_params()
    
    def load_params(self):
        self.declare_parameters(
            namespace="",
            parameters=[("arm_lengths", rclpy.Parameter.Type.DOUBLE_ARRAY)]
        )
        self.d0, self.d1, self.d4 = self.get_parameter("arm_lengths").value
    
    def execute(self):
        points = []

        # MOVE TO YELLOW TAG: (x_d, y_d, z_d) = (0.08, 0.252, 0.55)
        self.ik(0.08, 0.252, 0.55)
        point1 = JointTrajectoryPoint()
        point1.time_from_start = Duration(seconds=5, nanoseconds=0).to_msg()
        point1.positions = [self.q1, self.q2, self.q3]
        points.append(point1)

        # MOVE TO GREEN TAG: (x_d, y_d, z_d) = (0.341, -0.084, 0.811)
        self.ik(0.341, -0.084, 0.811)
        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=10, nanoseconds=0).to_msg()
        point2.positions = [self.q1, self.q2, self.q3]
        points.append(point2)

        # MOVE TO BLACK TAG: (x_d, y_d, z_d) = (0.123, -0.243, 0.688)
        self.ik(0.123, -0.243, 0.688)
        point3 = JointTrajectoryPoint()
        point3.time_from_start = Duration(seconds=15, nanoseconds=0).to_msg()
        point3.positions = [self.q1, self.q2, self.q3]
        points.append(point3)

        # RESTING POSE: (x_d, y_d, z_d) = (0.15, 0, 0.53)
        self.ik(0.15, 0, 0.53)
        point4 = JointTrajectoryPoint()
        point4.time_from_start = Duration(seconds=20, nanoseconds=0).to_msg()
        point4.positions = [self.q1, self.q2, self.q3]
        points.append(point4)

        self.send_joints(points)

    def ik(self, x_d, y_d, z_d):
        # Compute theta1 (q1)
        self.q1 = np.arctan2(y_d, x_d)

        # Check if the position is reachable
        r = np.sqrt(x_d**2 + y_d**2)
        expected_r = self.d1 + self.d4
        if not np.isclose(r, expected_r, atol=1e-2):
            self.get_logger().error(f"Position ({x_d}, {y_d}, {z_d}) is unreachable: sqrt(x^2 + y^2) = {r}, expected {expected_r}")
            return

        # Compute d2 + d3
        d2_plus_d3 = z_d - self.d0
        # Assume d3 = 0 for simplicity, so d2 takes the full translation
        self.q2 = d2_plus_d3
        self.q3 = 0.0

        self.get_logger().info(f"IK for ({x_d}, {y_d}, {z_d}): q1={self.q1:.2f}, q2={self.q2:.2f}, q3={self.q3:.2f}")

    def send_joints(self, points):
        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = ["base_link_to_1", "1_to_2", "2_to_3"]
        goal_msg.trajectory.points = points

        self.get_logger().info("Performing motion...")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_cb)
    
    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected!")
            return
        
        self.get_logger().info("Goal accepted")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_cb)
    
    def get_result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result}")
        self.get_logger().info("Mission finished!")
        self._action_client.destroy()

def main():
    rclpy.init()

    node = Robot()
    try:
        node.execute()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()