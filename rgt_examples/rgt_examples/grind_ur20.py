import math
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from action_msgs.msg import GoalStatus
from robot_manager_interfaces.action import PoseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler
from robot_manager_interfaces.msg import PoseServo
from robot_manager_interfaces.srv import StartAdmittancePoseServo, SetIo
from rgt_interfaces.srv import OverrideToolLocation
from robot_manager_interfaces.srv import Home
from std_srvs.srv import Trigger

class GrindUR20(Node):
    def __init__(self):
        super().__init__('grind_ur20')
        self.ns = "ur20/"

        self.get_logger().info("[INIT] Connecting to ROS 2 services...")

        self.publisher_ = self.create_publisher(
            PoseServo, self.ns + 'admittance_pose_servo', 10
        )
        self.pose_goal_client = ActionClient(self, PoseGoal, self.ns + "pose_goal")
        self.pose_goal_client.wait_for_server()
        
        self.override_tool_location_client = self.create_client(
            OverrideToolLocation, "/rgt_manager/override_tool_location"
        )
        while not self.override_tool_location_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "override_tool_location" service...')
            
        self.home_client = self.create_client(Home, self.ns + "home")
        while not self.home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "home" service...')
            
        self.start_admittance_pose_servo_client = self.create_client(
            StartAdmittancePoseServo, self.ns + 'start_admittance_pose_servo'
        )
        while not self.start_admittance_pose_servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "start_admittance_pose_servo" service...')
            
        self.stop_admittance_pose_servo_client = self.create_client(
            Trigger, self.ns + 'stop_admittance_pose_servo'
        )
        while not self.stop_admittance_pose_servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "stop_admittance_pose_servo" service...')

        self.set_io_client = self.create_client(SetIo, self.ns + 'set_io')
        while not self.set_io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "set_io" service...')

        self.get_logger().info("[INIT] All interfaces connected successfully.")

        # Default Parameters
        self.frame_id = "world"
        self.target_id = "grinder_tooltip"
        self.start_position = [0.0, 0.0, 0.02]
        self.end_position = [0.0, 0.10, 0.02]
        self.grind_speed = 0.01  # m/s
        self.orientation = quaternion_from_euler(
            math.radians(0), math.radians(20), math.radians(-90)
        )
        self.admittance_mass = 10.0
        self.admittance_damping = 6000.0
        self.admittance_stiffness = 0.01
        self.admittance_velocity_limit = 0.1
        self.admittance_force = 10.0

    def override_tool(self, tool="grinder", location="ur20"):
        req = OverrideToolLocation.Request()
        req.tool = tool
        req.location = location
        future = self.override_tool_location_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def home(self, speed=0.1):
        req = Home.Request()
        req.speed = speed
        future = self.home_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def set_io(self, pin: int, state: int):
        req = SetIo.Request()
        req.pin = pin
        req.state = state
        future = self.set_io_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def move_to_pose(self, position, orientation, velocity_scaling=0.04, acceleration_scaling=0.01):
        goal_msg = PoseGoal.Goal()
        goal_msg.target_pose = Pose(
            position=Point(x=position[0], y=position[1], z=position[2]),
            orientation=Quaternion(
                x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]
            )
        )
        goal_msg.velocity_scaling = velocity_scaling
        goal_msg.acceleration_scaling = acceleration_scaling
        goal_msg.frame_id = self.frame_id
        goal_msg.target_id = self.target_id
        goal_msg.method = "PTP"
        return self.run_action(self.pose_goal_client, goal_msg)

    def start_admittance_pose_servo(self):
        request = StartAdmittancePoseServo.Request()
        request.target_wrench.force.z = float(self.admittance_force)
        request.selection_vector = [False, False, True, False, False, False]
        request.mass = [0.0, 0.0, self.admittance_mass, 0.0, 0.0, 0.0]
        request.damping = [0.0, 0.0, self.admittance_damping, 0.0, 0.0, 0.0]
        request.stiffness = [0.0, 0.0, self.admittance_stiffness, 0.0, 0.0, 0.0]
        request.max_velocity = [0.0, 0.0, self.admittance_velocity_limit, 0.0, 0.0, 0.0]
        future = self.start_admittance_pose_servo_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def stop_admittance_pose_servo(self):
        request = Trigger.Request()
        future = self.stop_admittance_pose_servo_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def execute_linear_grind(self, start_pos, end_pos, speed, orientation, rate_hz=100.0):
        dx = end_pos[0] - start_pos[0]
        dy = end_pos[1] - start_pos[1]
        dz = end_pos[2] - start_pos[2]
        total_dist_m = math.sqrt(dx * dx + dy * dy + dz * dz)
        if total_dist_m == 0.0:
            self.get_logger().warn("[GRIND] Start/End positions identical. Skipping pass.")
            return

        duration = total_dist_m / speed
        dt = 1.0 / rate_hz
        steps = int(duration * rate_hz)
        total_dist_mm = total_dist_m * 1000.0

        bar_length = 30
        for i in range(steps + 1):
            alpha = i / steps if steps > 0 else 1.0
            msg = PoseServo()
            msg.frame_id = self.frame_id
            msg.target_id = self.target_id
            msg.target_pose.position = Point(
                x=start_pos[0] + alpha * dx,
                y=start_pos[1] + alpha * dy,
                z=start_pos[2] + alpha * dz
            )
            msg.target_pose.orientation = Quaternion(
                x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]
            )

            self.publisher_.publish(msg)

            # Distance-based progress bar readout
            curr_dist_mm = alpha * total_dist_mm
            filled_len = int(bar_length * alpha)
            bar = '█' * filled_len + '-' * (bar_length - filled_len)

            sys.stdout.write(
                f"\r  [PROGRESS] |{bar}| {alpha * 100:5.1f}% | {curr_dist_mm:5.1f} / {total_dist_mm:5.1f} mm"
            )
            sys.stdout.flush()

            time.sleep(dt)
            rclpy.spin_once(self, timeout_sec=0.0)

        print()  # Move past terminal progress line

    def countdown_delay(self, seconds: float):
        step = 0.5
        elapsed = 0.0
        while elapsed < seconds:
            time.sleep(step)
            elapsed += step
            remaining = max(0.0, seconds - elapsed)
            sys.stdout.write(f"\r  [WAIT] {remaining:3.1f}s remaining ")
            sys.stdout.flush()
        print()

    def run_action(self, action_client: ActionClient, goal_msg):
        send_goal_future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[ERROR] Action goal rejected by server")
            exit(1)
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"[ERROR] Action failed with status {result.status}")
            exit(1)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GrindUR20()

    # Parameters
    node.frame_id = "world"
    node.target_id = "grinder_tooltip"
    node.start_position = [0.0, 0.0, 0.02]
    node.end_position = [0.0, 0.15, 0.02]
    node.grind_speed = 0.01  # m/s
    node.orientation = quaternion_from_euler(
        math.radians(0), math.radians(20), math.radians(-90)
    )
    node.admittance_mass = 10.0
    node.admittance_damping = 6000.0
    node.admittance_stiffness = 0.01
    node.admittance_velocity_limit = 0.1
    node.admittance_force = 10.0

    try:
        node.get_logger().info("[1/8] Overriding tool location...")
        node.override_tool("grinder", "ur20")
        time.sleep(0.5)

        node.get_logger().info("[2/8] Homing robot...")
        node.home(0.1)
        time.sleep(0.5)

        node.get_logger().info("[3/8] Moving to start pose...")
        node.move_to_pose(node.start_position, node.orientation)
        time.sleep(0.5)

        node.get_logger().info("[4/8] Enabling admittance force control...")
        node.start_admittance_pose_servo()
        time.sleep(0.5)

        node.get_logger().info("[5/8] Powering ON grinder...")
        node.set_io(pin=1, state=1)
        time.sleep(0.5)

        node.countdown_delay(6.0)

        node.get_logger().info("[6/8] Executing linear grind pass...")
        node.execute_linear_grind(
            node.start_position,
            node.end_position,
            node.grind_speed,
            node.orientation
        )

        node.get_logger().info("[7/8] Disabling admittance & retracting...")
        node.stop_admittance_pose_servo()
        time.sleep(0.5)
        node.move_to_pose(node.end_position, node.orientation)
        time.sleep(0.5)

        node.get_logger().info("[8/8] Powering OFF grinder & homing...")
        node.set_io(pin=1, state=0)
        time.sleep(0.5)
        node.home(0.1)

        node.get_logger().info("[SUCCESS] Grinding cycle complete.")

    except KeyboardInterrupt:
        node.get_logger().warn("\n[ABORTED] Process interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
