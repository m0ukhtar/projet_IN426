import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import numpy as np

class Robot(Node):
    def __init__(self):
        #DO NOT TOUCH
        Node.__init__(self, "motion_robot_2")   #this name should be specified in the yaml file!!!

        self._action_client = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_position_controller/follow_joint_trajectory")
        self.get_logger().info("Waiting for action server to be ready...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server ready!")

        self.load_params()
    

    def load_params(self):
        """ Load parameters from YAML file """
        #DO NOT TOUCH
        self.declare_parameters(
            namespace="",
            parameters=[("arm_lengths", rclpy.Parameter.Type.DOUBLE_ARRAY)]
        )
        self.d0, self.d1, self.d4 = self.get_parameter("arm_lengths").value
    

    def execute(self):
        """ Perform the task """
        points = []

        #MOVE TO YELLOW TAG: (x_d, y_d, z_d) = (0.08, 0.252, 0.55)
        

        #MOVE TO GREEN TAG: (x_d, y_d, z_d) = (0.341, -0.084, 0.811)
        

        #MOVE TO BLACK TAG: (x_d, y_d, z_d) = (0.123, -0.243, 0.688)
        

        #RESTING POSE: (x_d, y_d, z_d) = (0.15, 0, 0.53)
        

        self.send_joints(points)


    def ik(self, x_d, y_d, z_d):
        """ Implement the inverse kinematics for this robot considering a desired pose (x_d, y_d, z_d) """
        #TODO: add the ik
        #self.q1 corresponds to theta1
        #self.q2 corresponds to d2
        #self.q3 corresponds to d3


    def send_joints(self, points):
        """ Send desired joints positions to the related action server """
        #DO NOT TOUCH
        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = ["base_link_to_1", "1_to_2", "2_to_3"]
        goal_msg.trajectory.points = points

        self.get_logger().info("Performing motion...")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)  #sending goal message
        self._send_goal_future.add_done_callback(self.goal_response_cb) #callback for response from action server
    

    def goal_response_cb(self, future):
        """ Check whether or not the task has been accepted by the action server """
        #DO NOT TOUCH
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected!")
            return
        
        self.get_logger().info("Goal accepted")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_cb)   #callback for result from action server
    

    def get_result_cb(self, future):
        """ Get the final result once the action server completes the task """
        #DO NOT TOUCH
        result = future.result().result
        self.get_logger().info(f"Result: {result}")
        self.get_logger().info("Mission finished!")
        self._action_client.destroy()


#DO NOT TOUCH
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