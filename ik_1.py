import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Vector3

import numpy as np


class IK(Node):
    def __init__(self):
        #DO NOT TOUCH
        Node.__init__(self, "ik_1")
        self.goal_received = False  #whether or not a new goal has been received

        self._action_client = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_position_controller/follow_joint_trajectory")
        self.get_logger().info("Waiting for action server to be ready...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server ready!")

        self.load_params()
        self.create_subscription(Vector3, "/goal", self.goal_cb, 1)
        self.create_timer(0.1, self.execute)
    

    def load_params(self):
        """ Load parameters from YAML file """
        #DO NOT TOUCH
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
        """ Get goal coordinates published on topic '/goal' """
        #DO NOT TOUCH
        self.x_d, self.y_d, self.z_d = msg.x, msg.y, msg.z
        self.goal_received = True
    

    def check_limits(self):
        """ Check if the computed joint values are within the defined limits of the respective joints """
        #DO NOT TOUCH
        if not (self.lower_limits[0] <= self.t1 <= self.upper_limits[0]):
            return False
        if not (self.lower_limits[1] <= self.t2 <= self.upper_limits[1]):
            return False
        if not (self.lower_limits[2] <= self.t3 <= self.upper_limits[2]):
            return False
        return True
    

    def execute(self):
        """ Run the inverse kinematics whenever a new message is received """
        #DO NOT TOUCH
        if self.goal_received:
            self.ik()
            self.goal_received = False
    

    def ik(self):
        """ Compute the inverse kinematics """
        #TODO: Write the inverse kinematics here
        #self.t1 corresponds to theta1
        #self.t2 corresponds to theta2
        #self.t3 corresponds to theta3

        
        self.get_logger().info(f"t1: {self.t1:.2f}, t2: {self.t2:.2f}, t3: {self.t3:.2f}")

        #DO NOT TOUCH
        if self.check_limits():
            self.get_logger().info("Sending joint values")
            self.send_joints()  #send goal to action server

    
    def send_joints(self):
        """ Send desired joints positions to the related action server """
        #DO NOT TOUCH
        goal_msg = FollowJointTrajectory.Goal()

        joint_names = ["base_to_1", "1_to_2", "2_to_3"]

        points = []
        point1 = JointTrajectoryPoint()
        point1.time_from_start = Duration(seconds=5, nanoseconds=0).to_msg()    #5 seconds to reach the desired position
        point1.positions = [self.t1, self.t2, self.t3]
        points.append(point1)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)


#DO NOT TOUCH
def main():
    rclpy.init()

    node = IK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()