#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

import threading

from robot_manager_interfaces.action import JointGoal
from robot_manager_interfaces.action import PoseGoal
from geometry_msgs.msg import Point, Quaternion
from robot_manager_interfaces.srv import Home

class TestToolChangerReference(Node):
    def __init__(self):
        super().__init__('test_tool_changer_reference')

        self.declare_parameter('ns', '')
        self.declare_parameter('tc', '')
        self.ns = str(self.get_parameter("ns").value)
        self.tc = str(self.get_parameter("tc").value)
        
        self.pose_goal_client = ActionClient(self, PoseGoal, self.ns + "/" + "pose_goal")
        self.pose_goal_client.wait_for_server()
        self.joint_goal_client = ActionClient(self, JointGoal, self.ns + "/" + "joint_goal")
        self.joint_goal_client.wait_for_server()
        self.home_client = self.create_client(Home, self.ns + "/" + "home")
        while not self.home_client.wait_for_service(): continue

    def run_action(self, action_client: ActionClient, goal_msg, show_progress = False):
        if show_progress:
            result = action_client.send_goal(goal_msg, feedback_callback=lambda msg: self.get_logger().info(f'Progress: {msg.feedback.progress:.1f}%'))
        else:
            result = action_client.send_goal(goal_msg)
        status = result.status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(result.result.message)
            exit(1)

def main(args=None):
    rclpy.init(args=args)
    node = TestToolChangerReference()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    goal_msg = PoseGoal.Goal()
    goal_msg.acceleration_scaling = 0.1
    goal_msg.frame_id = node.tc
    goal_msg.target_id = node.ns + "_tc_hook"
    goal_msg.target_pose.position = Point()
    goal_msg.target_pose.orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    try:
        # home robot
        req = Home.Request()
        req.speed = 0.3
        node.home_client.call(req)

        # move above reference
        goal_msg.method = "PTP"
        goal_msg.velocity_scaling = 0.2
        goal_msg.target_pose.position = Point(x=0.0, y=0.0, z=0.1)
        node.run_action(node.pose_goal_client, goal_msg)

        input("Proceed?")
        goal_msg.method = "LIN"
        goal_msg.velocity_scaling = 0.05
        goal_msg.target_pose.position = Point(x=0.0, y=0.0, z=0.006)
        node.run_action(node.pose_goal_client, goal_msg)

        input("Proceed?")
        goal_msg.target_pose.position = Point(x=0.0, y=0.0, z=0.0)
        node.run_action(node.pose_goal_client, goal_msg)

        input("Move up?")
        goal_msg.velocity_scaling = 0.1
        goal_msg.target_pose.position = Point(x=0.0, y=0.0, z=0.1)
        node.run_action(node.pose_goal_client, goal_msg)

        # home robot
        node.home_client.call(req)

    except KeyboardInterrupt:
        node.get_logger().info("Script interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()
