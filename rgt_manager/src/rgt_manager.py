#!/usr/bin/env python3
import os
import yaml
import rclpy
import time
from rclpy.action.client import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Transform
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from robot_manager_interfaces.action import PoseGoal
from robot_manager_interfaces.srv import Home
from geometry_msgs.msg import Point, Quaternion
from rgt_interfaces.srv import TakeTool, ReturnTool, OverrideToolLocation, ChangeTool

class RgtManager(Node):
    def __init__(self):
        super().__init__('rgt_manager')
        self.declare_parameter('package', 'rgt_manager')
        self.declare_parameter('config', 'config.yaml')
        self.declare_parameter('urdf', 'rgt.urdf')

        package = str(self.get_parameter("package").value)
        config = str(self.get_parameter("config").value)
        urdf = str(self.get_parameter("urdf").value)

        self.get_logger().info("Starting rgt_manager...")

        package_share_directory = get_package_share_directory(package)

        # read urdf
        urdf_path = os.path.join(package_share_directory, 'urdf', urdf)
        self.urdf = self.read_urdf(urdf_path)
        
        # publish urdf
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.robot_description_publisher = self.create_publisher(String, '/rgt/robot_description', qos_profile)
        msg = String()
        msg.data = self.urdf
        self.robot_description_publisher.publish(msg)
        self.get_logger().info("Successfully published urdf!")

        # read config
        config_path = os.path.join(package_share_directory, 'config', config)
        self.config = self.read_config(config_path)

        # Static Transforms
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        # Dynamic Transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.1, self.publish_dynamic_transforms)

        # Initial Config
        self.initial_config()
        self.get_logger().info("Successfully initialized rgt_manager!")

        # Tool Change Service
        self.cb_group = ReentrantCallbackGroup()
        self.take_tool_server = self.create_service(
            TakeTool,
            '/rgt_manager/take_tool',
            self.take_tool_callback,
            callback_group=self.cb_group
        )
        self.return_tool_server = self.create_service(
            ReturnTool,
            '/rgt_manager/return_tool',
            self.return_tool_callback,
            callback_group=self.cb_group
        )
        self.override_tool_location_server = self.create_service(
            OverrideToolLocation,
            '/rgt_manager/override_tool_location',
            self.override_tool_location_callback,
            callback_group=self.cb_group
        )
        self.change_tool_server = self.create_service(
            ChangeTool,
            '/rgt_manager/change_tool',
            self.change_tool_callback,
            callback_group=self.cb_group
        )

    ### --- Override Tool Location Callback
    async def override_tool_location_callback(self, request, response):
        self.get_logger().info(f"Received override_tool_location request for tool: {request.tool} and location: {request.location}")
        # Check whether tool is recognized
        if request.tool not in self.config["tools"]:
            self.get_logger().error(f"Tool '{request.tool}' is not recognized in the configuration.")
            response.success = False
            return response
        # If desired location is tray
        if request.location == "tray":
            self.config["tools"][request.tool]["location"] = request.location
            self.get_logger().info(f"Successfully overrode {request.tool} location to tray.")
            # TODO set master payload
        # If desired location is a known robot
        elif request.location in self.config["robots"]:
            self.config["tools"][request.tool]["location"] = request.location
            self.get_logger().info(f"Successfully overrode {request.tool} location to robot: {request.location}.")
            # TODO set tool payload
        # otherwise error out and return False as success
        else:
            self.get_logger().error(f"Location '{request.location}' is neither 'tray' nor a known robot name.")
            response.success = False
        return response


    ### --- Dynamic Transform (Tools only)
    def publish_dynamic_transforms(self):
        current_time = self.get_clock().now().to_msg()
        for tool_name, tool_config in self.config["tools"].items():
            location = tool_config["location"]
            tray_name = tool_config["tray_name"]
            if location == "tray":
                for robot_name in self.config["trays"][tray_name]["references"]:
                    t = TransformStamped()
                    t.header.stamp = current_time
                    t.header.frame_id = robot_name + "_" + tray_name + "_tc_connector"
                    t.child_frame_id = tool_name
                    self.tf_broadcaster.sendTransform(t)
                    break
            elif location in self.config["robots"]:
                t = TransformStamped()
                t.header.stamp = current_time
                t.header.frame_id = location + "_tc_connector"
                t.child_frame_id = tool_name
                self.tf_broadcaster.sendTransform(t)

    ### --- Initial Config
    def initial_config(self):
        # table
        self.publish_static_transform("world", "table", {})
        # robots
        for robot_name, robot_config in self.config["robots"].items():
            # publish mount_point transform
            parent_frame = "world"
            child_frame = robot_name + "_mount_point"
            transform = robot_config["mount_point"]
            self.publish_static_transform(parent_frame, child_frame, transform)
            # publish tool0 transforms
            parent_frame = robot_name + "_tool0"
            if "tool0_transforms" in robot_config:
                for frame_name, transform in robot_config["tool0_transforms"].items():
                    self.publish_static_transform(parent_frame, robot_name + "_" + frame_name, transform)
        # trays
        for tray_name, tray_config in self.config["trays"].items():
            # publish references
            if "references" in tray_config:
                for robot_name, transform in tray_config["references"].items():
                    parent_frame = robot_name + "_mount_point"
                    child_frame = robot_name + "_" + tray_name
                    self.publish_static_transform(parent_frame, child_frame, transform)
                    # publish tc_master
                    parent_frame = child_frame
                    child_frame = robot_name + "_" + tray_name + "_tc_connector"
                    self.publish_static_transform(parent_frame, child_frame, {"x": -0.046, "z": -0.0066})
        # tools
        for tool_name, tool_config in self.config["tools"].items():
            # publish static child transform
            if "child_transforms" in tool_config:
                for child_frame, transform in tool_config["child_transforms"].items():
                    self.publish_static_transform(tool_name, child_frame, transform)

    ### --- Helper Functions
    # Run Action
    def run_action(self, action_client: ActionClient, goal_msg, show_progress = False):
        if show_progress:
            result = action_client.send_goal(goal_msg, feedback_callback=lambda msg: self.get_logger().info(f'Progress: {msg.feedback.progress:.1f}%'))
        else:
            result = action_client.send_goal(goal_msg)
        status = result.status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(result.result.message)
            exit(1)
    # Read Config
    def read_config(self, path):
        with open(path, 'r') as f:
                data = yaml.safe_load(f)
        if not data:
            self.get_logger().error(f'Config file at {path} is empty or invalid.')
            return dict()
        else:
            return data
    # Read URDF
    def read_urdf(self, path):
        try:
            with open(path, 'r') as file:
                urdf_content = file.read()
                return urdf_content
        except Exception as e:
            self.get_logger().error(f'Failed to read URDF file: {e}')
            return ""
    # Publish Static Transform
    def publish_static_transform(self, parent_frame, child_frame, transform):
        try:
            # self.get_logger().info(f"Publishing transform with parent_frame: {parent_frame} and child_frame: {child_frame}")
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = str(parent_frame)
            t.child_frame_id = str(child_frame)
            t.transform.translation.x = float(transform.get('x', 0.0))
            t.transform.translation.y = float(transform.get('y', 0.0))
            t.transform.translation.z = float(transform.get('z', 0.0))
            t.transform.rotation.x = float(transform.get('ax', 0.0))
            t.transform.rotation.y = float(transform.get('ay', 0.0))
            t.transform.rotation.z = float(transform.get('az', 0.0))
            t.transform.rotation.w = float(transform.get('w', 1.0))
            self.tf_static_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f'Failed to publish transform: {e}')

    ### --- Take Tool
    def take_tool_from_tray(self, robot_name, tray_name, tool_name):
        # Create clients
        pose_goal_client = ActionClient(self, PoseGoal, robot_name + "/" + "pose_goal", callback_group=self.cb_group)
        home_client = self.create_client(Home, robot_name + "/" + "home", callback_group=self.cb_group)
        pose_goal_client.wait_for_server()
        while not home_client.wait_for_service(): continue
        # attach tool tf to tray tf
        self.config["tools"][tool_name]["location"] = "tray"
        # move to home
        req = Home.Request()
        req.speed = 0.2
        home_client.call(req)
        # Setup reusable goal_msg
        goal_msg = PoseGoal.Goal()
        goal_msg.velocity_scaling = 0.2
        goal_msg.acceleration_scaling = 0.1
        goal_msg.frame_id = robot_name + "_" + tray_name
        goal_msg.target_id = robot_name + "_tc_hook"
        goal_msg.target_pose.position = Point()
        goal_msg.target_pose.orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        goal_msg.method = "PTP"
        # Move to tray
        goal_msg.target_pose.position = Point(x=0.0, y=0.0, z=0.025)
        self.run_action(pose_goal_client, goal_msg)
        time.sleep(0.1)
        goal_msg.method = "LIN"
        goal_msg.velocity_scaling = 0.01
        goal_msg.target_pose.position = Point(x=0.0, y=0.0, z=0.0)
        self.run_action(pose_goal_client, goal_msg)
        # Attach tool to robot
        self.config["tools"][tool_name]["location"] = robot_name
        time.sleep(0.1)
        # Tool Change Sequence
        goal_msg.target_pose.position = Point(x=0.0088, y=0.0, z=0.0)
        self.run_action(pose_goal_client, goal_msg)                   
        time.sleep(0.1)
        goal_msg.target_pose.position = Point(x=0.0093, y=0.0, z=0.0)
        self.run_action(pose_goal_client, goal_msg)
        time.sleep(0.1)
        goal_msg.target_pose.position = Point(x=0.0088, y=0.0, z=0.0)
        self.run_action(pose_goal_client, goal_msg)                   
        time.sleep(0.1)
        goal_msg.target_pose.position = Point(x=0.0088, y=0.0, z=0.0095)
        self.run_action(pose_goal_client, goal_msg)
        time.sleep(0.1)
        goal_msg.velocity_scaling = 0.1
        goal_msg.target_pose.position = Point(x=-0.17, y=0.0, z=0.0095)
        self.run_action(pose_goal_client, goal_msg)
        time.sleep(0.1)
        goal_msg.velocity_scaling = 0.4
        goal_msg.target_pose.position = Point(x=-0.17, y=0.0, z=0.15)
        self.run_action(pose_goal_client, goal_msg)
        # move to home
        req.speed = 0.6
        home_client.call(req)

    ### --- Take Tool Callback
    async def take_tool_callback(self, request, response):
        self.get_logger().info(f"Received take_tool request for robot: {request.robot_name} and tray: {request.tray_name}")
        tool_name = ""
        for name, tool_config in self.config["tools"].items():
            if tool_config["tray_name"] == request.tray_name:
                tool_name = name
                break
        if tool_name == "":
            self.get_logger().error(f"Tray '{request.tray_name}' isn't associated with any tool!")
            response.success = False
            return response
        self.take_tool_from_tray(request.robot_name, request.tray_name, tool_name)
        response.success = True
        self.get_logger().info(f"Successfully finished take_tool operation")
        return response

    ### --- Return Tool
    def return_tool_to_tray(self, robot_name, tray_name, tool_name):
        # Create clients
        pose_goal_client = ActionClient(self, PoseGoal, robot_name + "/" + "pose_goal", callback_group=self.cb_group)
        home_client = self.create_client(Home, robot_name + "/" + "home", callback_group=self.cb_group)
        pose_goal_client.wait_for_server()
        while not home_client.wait_for_service(): continue
        # attach tool tf to robot tf
        self.config["tools"][tool_name]["location"] = robot_name
        # move to home
        req = Home.Request()
        req.speed = 0.2
        home_client.call(req)
        # Setup reusable goal_msg
        goal_msg = PoseGoal.Goal()
        goal_msg.velocity_scaling = 0.2
        goal_msg.acceleration_scaling = 0.15
        goal_msg.frame_id = robot_name + "_" + tray_name
        goal_msg.target_id = robot_name + "_tc_hook"
        goal_msg.target_pose.position = Point()
        goal_msg.target_pose.orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        goal_msg.method = "PTP"
        # Move to tray
        goal_msg.target_pose.position = Point(x=-0.17, y=0.0, z=0.15)
        self.run_action(pose_goal_client, goal_msg)
        time.sleep(0.1)
        goal_msg.target_pose.position = Point(x=-0.17, y=0.0, z=0.0095)
        self.run_action(pose_goal_client, goal_msg)
        time.sleep(0.1)
        # Tool Change Sequence
        goal_msg.method = "LIN"
        goal_msg.velocity_scaling = 0.05
        goal_msg.target_pose.position = Point(x=0.0088, y=0.0, z=0.0095)
        self.run_action(pose_goal_client, goal_msg)
        time.sleep(0.1)
        goal_msg.velocity_scaling = 0.01
        goal_msg.target_pose.position = Point(x=0.0088, y=0.0, z=0.0)
        self.run_action(pose_goal_client, goal_msg)
        time.sleep(0.1)
        goal_msg.target_pose.position = Point(x=0.0, y=0.0, z=0.0)
        self.run_action(pose_goal_client, goal_msg)
        # attach tool to tray
        self.config["tools"][tool_name]["location"] = "tray"
        time.sleep(0.1)
        # Move back
        goal_msg.target_pose.position = Point(x=-0.0005, y=0.0, z=0.0)
        self.run_action(pose_goal_client, goal_msg)
        time.sleep(0.1)
        goal_msg.target_pose.position = Point(x=0.0, y=0.0, z=0.0)
        self.run_action(pose_goal_client, goal_msg)
        time.sleep(0.1)
        goal_msg.velocity_scaling = 0.05
        goal_msg.target_pose.position = Point(x=0.0, y=0.0, z=0.025)
        self.run_action(pose_goal_client, goal_msg)
        time.sleep(0.1)
        # move to home
        req.speed = 0.6
        home_client.call(req)

    ### --- Return Tool Callback
    async def return_tool_callback(self, request, response):
        self.get_logger().info(f"Received return_tool request for robot: {request.robot_name} and tray: {request.tray_name}")
        tool_name = ""
        for name, tool_config in self.config["tools"].items():
            if tool_config["tray_name"] == request.tray_name:
                tool_name = name
                break
        if tool_name == "":
            self.get_logger().error(f"Tray '{request.tray_name}' isn't associated with any tool!")
            response.success = False
            return response
        self.return_tool_to_tray(request.robot_name, request.tray_name, tool_name)
        response.success = True
        self.get_logger().info(f"Successfully finished return_tool operation")
        return response

    ### --- Change Tool Callback
    async def change_tool_callback(self, request, response):
        self.get_logger().info(f"Received change_tool request for robot: {request.robot_name} and tool: '{request.tool_name}'")
        # Check if robot currently has a tool attached
        current_tool_attached = None
        for name, tool_config in self.config["tools"].items():
            if tool_config["location"] == request.robot_name:
                current_tool_attached = name
                break
        # If requested tool is empty, only return the current tool (if one is attached)
        if request.tool_name == "":
            if current_tool_attached is not None:
                tray_name = self.config["tools"][current_tool_attached]["tray_name"]
                self.get_logger().info(f"Robot '{request.robot_name}' currently has '{current_tool_attached}' attached. Returning it to tray '{tray_name}'.")
                self.return_tool_to_tray(request.robot_name, tray_name, current_tool_attached)
            else:
                self.get_logger().info(f"No tool currently attached to '{request.robot_name}'. Nothing to return.")
            
            response.success = True
            self.get_logger().info(f"Successfully finished change_tool operation (return only)")
            return response
        # Check if requested tool exists
        if request.tool_name not in self.config["tools"]:
            self.get_logger().error(f"Tool '{request.tool_name}' does not exist in the configuration.")
            response.success = False
            return response
        # Check if requested tool is already attached to the requesting robot
        if self.config["tools"][request.tool_name]["location"] == request.robot_name:
            self.get_logger().info(f"Tool '{request.tool_name}' is already attached to robot '{request.robot_name}'.")
            response.success = True
            return response
        # Check if requested tool is attached to another robot
        target_tool_location = self.config["tools"][request.tool_name]["location"]
        if target_tool_location in self.config["robots"] and target_tool_location != request.robot_name:
            other_robot = target_tool_location
            self.get_logger().error(f"Tool '{request.tool_name}' is currently attached to another robot: '{other_robot}'. Aborting.")
            response.success = False
            return response
        # If the requesting robot has a different tool attached, return it to its tray first
        if current_tool_attached is not None:
            tray_name = self.config["tools"][current_tool_attached]["tray_name"]
            self.get_logger().info(f"Robot '{request.robot_name}' currently has '{current_tool_attached}' attached. Returning it to tray '{tray_name}'.")
            self.return_tool_to_tray(request.robot_name, tray_name, current_tool_attached)
        # Take new tool from associated tray
        target_tray_name = self.config["tools"][request.tool_name]["tray_name"]
        self.get_logger().info(f"Taking requested tool '{request.tool_name}' from tray '{target_tray_name}'.")
        self.take_tool_from_tray(request.robot_name, target_tray_name, request.tool_name)
        response.success = True
        self.get_logger().info(f"Successfully finished change_tool operation")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RgtManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

