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
from rgt_interfaces.srv import TakeTool, ReturnTool, OverrideToolLocation

class RgtManager(Node):
    def __init__(self):
        super().__init__('rgt_manager')
        self.declare_parameter('package', 'rgt_manager')
        self.declare_parameter('config', 'config.yaml')
        self.declare_parameter('urdf', 'rgt.urdf')

        package = str(self.get_parameter("package").value)
        config = str(self.get_parameter("config").value)
        urdf = str(self.get_parameter("urdf").value)

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

        # read config
        config_path = os.path.join(package_share_directory, 'config', config)
        self.config = self.read_config(config_path)

        # publish static transforms
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.static_transforms = {}
        self.publish_static_transforms()

        # Setup dynamic transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tools = {}
        self.tf_timer = self.create_timer(0.1, self.publish_dynamic_transforms)

        # Load initial tool config
        self.load_initial_tool_config()

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
        self.override_tool_location = self.create_service(
            OverrideToolLocation,
            '/rgt_manager/override_tool_location',
            self.override_tool_location_callback,
            callback_group=self.cb_group
        )

    def run_action(self, action_client: ActionClient, goal_msg, show_progress = False):
        if show_progress:
            result = action_client.send_goal(goal_msg, feedback_callback=lambda msg: self.get_logger().info(f'Progress: {msg.feedback.progress:.1f}%'))
        else:
            result = action_client.send_goal(goal_msg)
        status = result.status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(result.result.message)
            exit(1)

    def publish_dynamic_transforms(self):
        current_time = self.get_clock().now().to_msg()
        for child_frame, parent_frame in self.tools.items():
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            self.tf_broadcaster.sendTransform(t)

    def publish_static_transforms(self):
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

    def load_initial_tool_config(self):
        # tools
        for tool_name, tool_config in self.config["tools"].items():
            robot_name = tool_config["robot_name"]
            tray_name = tool_config["tray_name"]
            # start on robot:
            if tool_config["start_on_robot"]:
                self.tools[tool_name] = robot_name + "_tc_connector"
            # start on tray
            else:
                self.tools[tool_name] = robot_name + "_" + tray_name + "_tc_connector"

    def read_config(self, path):
        with open(path, 'r') as f:
                data = yaml.safe_load(f)
        if not data:
            self.get_logger().error(f'Config file at {path} is empty or invalid.')
            return dict()
        else:
            return data

    def read_urdf(self, path):
        try:
            with open(path, 'r') as file:
                urdf_content = file.read()
                return urdf_content
        except Exception as e:
            self.get_logger().error(f'Failed to read URDF file: {e}')
            return ""

    def publish_static_transform(self, parent_frame, child_frame, transform):
        try:
            self.get_logger().info(f"Publishing transform with parent_frame: {parent_frame} and child_frame: {child_frame}")
            t = TransformStamped()
            # header
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = str(parent_frame)
            t.child_frame_id = str(child_frame)
            # set translation
            t.transform.translation.x = float(transform.get('x', 0.0))
            t.transform.translation.y = float(transform.get('y', 0.0))
            t.transform.translation.z = float(transform.get('z', 0.0))
            # set rotation
            t.transform.rotation.x = float(transform.get('ax', 0.0))
            t.transform.rotation.y = float(transform.get('ay', 0.0))
            t.transform.rotation.z = float(transform.get('az', 0.0))
            t.transform.rotation.w = float(transform.get('w', 1.0))
            # publish transform
            self.static_transforms[str(child_frame)] = t
            self.tf_static_broadcaster.sendTransform(list(self.static_transforms.values()))
        except Exception as e:
            self.get_logger().error(f'Failed to publish transform: {e}')

    async def take_tool_callback(self, request, response):
        robot_ns = request.robot_ns
        tool_changer_id = request.tool_changer_id
        self.get_logger().info(f"Received take_tool request for robot: {robot_ns} and tool_changer_id: {tool_changer_id}")

        pose_goal_client = ActionClient(self, PoseGoal, robot_ns + "/" + "pose_goal", callback_group=self.cb_group)
        home_client = self.create_client(Home, robot_ns + "/" + "home", callback_group=self.cb_group)
        pose_goal_client.wait_for_server()
        while not home_client.wait_for_service(): continue

        # Check wheter tray is associated with tool
        tool_name = ""
        for name, tool_config in self.config["tools"].items():
            if int(tool_config["tray_name"][-1]) == tool_changer_id:
                tool_name = name
        # attach tool to tray
        if tool_name != "":
            self.tools[tool_name] = robot_ns + "_tray_" + str(tool_changer_id) + "_tc_connector"

        # move to home
        req = Home.Request()
        req.speed = 0.2
        home_client.call(req)
        # Setup reusable goal_msg
        goal_msg = PoseGoal.Goal()
        goal_msg.velocity_scaling = 0.2
        goal_msg.acceleration_scaling = 0.1
        goal_msg.frame_id = robot_ns + "_tray_" + str(tool_changer_id)
        goal_msg.target_id = robot_ns + "_tc_hook"
        goal_msg.target_pose.position = Point()
        goal_msg.target_pose.orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        goal_msg.method = "PTP"

        # Start Take Tool Sequence
        goal_msg.target_pose.position = Point(x=0.0, y=0.0, z=0.025)
        self.run_action(pose_goal_client, goal_msg)
        time.sleep(0.1)
        goal_msg.method = "LIN"
        goal_msg.velocity_scaling = 0.01
        goal_msg.target_pose.position = Point(x=0.0, y=0.0, z=0.0)
        self.run_action(pose_goal_client, goal_msg)
        # Attach tool to robot
        if tool_name != "":
            self.tools[tool_name] = robot_ns + "_tc_connector"
        time.sleep(0.1)
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
        goal_msg.target_pose.position = Point(x=-0.15, y=0.0, z=0.0095)
        self.run_action(pose_goal_client, goal_msg)
        time.sleep(0.1)

        # move to home
        req.speed = 0.4
        home_client.call(req)
        
        response.success = True
        self.get_logger().info(f"Successfully finished take_tool operation")
        return response

    def return_tool_callback(self, request, response):
        robot_ns = request.robot_ns
        tool_changer_id = request.tool_changer_id
        self.get_logger().info(f"Received return_tool request for robot: {robot_ns} and tool_changer_id: {tool_changer_id}")

        pose_goal_client = ActionClient(self, PoseGoal, robot_ns + "/" + "pose_goal", callback_group=self.cb_group)
        home_client = self.create_client(Home, robot_ns + "/" + "home", callback_group=self.cb_group)
        pose_goal_client.wait_for_server()
        while not home_client.wait_for_service(): continue

        # Check wheter tray is associated with tool
        tool_name = ""
        for name, tool_config in self.config["tools"].items():
            if int(tool_config["tray_name"][-1]) == tool_changer_id:
                tool_name = name
        # Attach tool to robot
        if tool_name != "":
            self.tools[tool_name] = robot_ns + "_tc_connector"

        # move to home
        req = Home.Request()
        req.speed = 0.2
        home_client.call(req)

        # Setup reusable goal_msg
        goal_msg = PoseGoal.Goal()
        goal_msg.velocity_scaling = 0.2
        goal_msg.acceleration_scaling = 0.1
        goal_msg.frame_id = robot_ns + "_tray_" + str(tool_changer_id)
        goal_msg.target_id = robot_ns + "_tc_hook"
        goal_msg.target_pose.position = Point()
        goal_msg.target_pose.orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        goal_msg.method = "PTP"

        # Start Take Tool Sequence
        goal_msg.target_pose.position = Point(x=-0.15, y=0.0, z=0.0095)
        self.run_action(pose_goal_client, goal_msg)
        time.sleep(0.1)
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
        if tool_name != "":
            self.tools[tool_name] = robot_ns + "_tray_" + str(tool_changer_id) + "_tc_connector"
        time.sleep(0.1)
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
        req.speed = 0.4
        home_client.call(req)
        
        response.success = True
        self.get_logger().info(f"Successfully finished return_tool operation")
        return response

    async def override_tool_location_callback(self, request, response):
        self.get_logger().info(f"Received override_tool_location request for tool: {request.tool} and location: {request.location}")

        # Check whether tool is recognized
        if request.tool not in self.config["tools"]:
            self.get_logger().error(f"Tool '{request.tool}' is not recognized in the configuration.")
            response.success = False
            return response

        # If request.location = tray: attach tool to tray from config
        if request.location == "tray":
            tool_config = self.config["tools"][request.tool]
            robot_name = tool_config["robot_name"]
            tray_name = tool_config["tray_name"]
            self.tools[request.tool] = robot_name + "_" + tray_name + "_tc_connector"
            response.success = True
            self.get_logger().info(f"Successfully overrode {request.tool} location to tray.")

        # else if check if the location is a known robot name
        # if yes, attach to that robot
        elif request.location in self.config["robots"]:
            self.tools[request.tool] = request.location + "_tc_connector"
            response.success = True
            self.get_logger().info(f"Successfully overrode {request.tool} location to robot: {request.location}.")

        # otherwise error out and return False as success
        else:
            self.get_logger().error(f"Location '{request.location}' is neither 'tray' nor a known robot name.")
            response.success = False

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

