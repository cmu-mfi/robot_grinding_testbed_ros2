#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
import pyspacemouse
import time
import threading

class SpaceMouseNode(Node):
    def __init__(self):
        super().__init__('space_mouse_node')
        self.declare_parameter('ns', '')
        self.ns = str(self.get_parameter("ns").value)
        if self.ns != "":
            self.frame_id = self.ns + "_tool0"
        else:
            self.frame_id = "tool0"

        # Twist Publisher
        self.twist_publisher = self.create_publisher(
            TwistStamped, 
            self.ns + '/servo_node/delta_twist_cmds', 
            10
        )

        # Enable Servo
        self.servo_client = self.create_client(ServoCommandType, self.ns + '/servo_node/switch_command_type')
        while not self.servo_client.wait_for_service(timeout_sec=1.0): 
            self.get_logger().info("Waiting for service: " + self.ns + '/servo_node/switch_command_type')
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        # Spacemouse
        self.smoothing_factor = 0.2  # Range: 0.0 to 1.0. Lower = smoother/slower response. Higher = sharper/faster response.
        self.smoothed_v = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Tracks [x, y, z, ax, ay, az]
        self.current_t = 0.0
        self.previous_t = 0.0
        self.skipped_ts = 0
        self.linear_speed = 0.0
        self.tilt_speed = 0.0
        self.yaw_speed = 0.0
        self.current_buttons = [0,0]
        self.previous_buttons = [0,0]
        self.left_button_pressed = False
        self.right_button_pressed = False
        self.state = None
        success = pyspacemouse.open()
        if success:
            self.get_logger().info("Space Mouse connected succesfully!")
        else:
            self.get_logger().error("Failed to connect to Space Mouse!")
            exit()
        self.spacemouse_loop = self.create_timer((1 / 200), self.spacemouse_loop_callback)

        self.get_logger().info("Navigating with spacemouse! Press left button to change speed and right button to print transform!")

    #### Spacemouse
    def spacemouse_loop_callback(self):
        self.state = pyspacemouse.read()
        if type(self.state) == pyspacemouse.pyspacemouse.SpaceNavigator:
            self.current_buttons = [self.state.buttons[0], self.state.buttons[14]]
            if self.current_buttons[0] == 1 and self.previous_buttons[0] == 0:
                self.left_button_pressed = True
            if self.current_buttons[1] == 1 and self.previous_buttons[1] == 0:
                self.right_button_pressed = True
            self.previous_buttons = self.current_buttons
    def navigate(self):
        if type(self.state) == pyspacemouse.pyspacemouse.SpaceNavigator:
            self.current_t = self.state.t
            diff = abs(self.current_t - self.previous_t)
            
            if diff == 0.0:
                self.skipped_ts += 1
            else:
                self.skipped_ts = 0

            if self.skipped_ts >= 10:
                self.stop()
                # Reset the smoothing history so it doesn't jerk on the next movement
                self.smoothed_v = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
                return
                
            self.previous_t = self.current_t
            
            # 1. Gather raw target velocities
            target_x = self.state.x * self.linear_speed
            target_y = -self.state.y * self.linear_speed
            target_z = -self.state.z * self.linear_speed
            
            target_ax = -self.state.pitch * self.tilt_speed
            target_ay = -self.state.roll * self.tilt_speed
            target_az = self.state.yaw * self.yaw_speed

            # 2. Apply Exponential Moving Average (EMA) smoothing
            alpha = self.smoothing_factor
            self.smoothed_v[0] = (alpha * target_x) + ((1 - alpha) * self.smoothed_v[0])
            self.smoothed_v[1] = (alpha * target_y) + ((1 - alpha) * self.smoothed_v[1])
            self.smoothed_v[2] = (alpha * target_z) + ((1 - alpha) * self.smoothed_v[2])
            self.smoothed_v[3] = (alpha * target_ax) + ((1 - alpha) * self.smoothed_v[3])
            self.smoothed_v[4] = (alpha * target_ay) + ((1 - alpha) * self.smoothed_v[4])
            self.smoothed_v[5] = (alpha * target_az) + ((1 - alpha) * self.smoothed_v[5])

            # 3. Publish the smoothed velocities
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = self.frame_id
            
            twist_msg.twist.linear.x = self.smoothed_v[0]
            twist_msg.twist.linear.y = self.smoothed_v[1]
            twist_msg.twist.linear.z = self.smoothed_v[2]
            twist_msg.twist.angular.x = self.smoothed_v[3]
            twist_msg.twist.angular.y = self.smoothed_v[4]
            twist_msg.twist.angular.z = self.smoothed_v[5]
            
            self.twist_publisher.publish(twist_msg)

    def stop(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.frame_id
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        self.twist_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    slow = False
    locked = False
    
    try:
        node.get_logger().info("Navigating!")
        while rclpy.ok():
            if not slow:
                node.linear_speed = 0.4
                node.tilt_speed = 1.2
                node.yaw_speed = 1.2
            if slow:
                node.linear_speed = 0.04
                node.tilt_speed = 0.12
                node.yaw_speed = 0.12
            if locked:
                node.tilt_speed = 0.0
            node.navigate()
            if node.left_button_pressed:
                node.left_button_pressed = False
                slow = not slow
                if slow:
                    node.get_logger().info("Navigation Mode: Slow")
                else:
                    node.get_logger().info("Navigation Mode: Fast")
            if node.right_button_pressed:
                node.right_button_pressed = False
                locked = not locked
                if locked:
                    node.get_logger().info("Navigation Mode: 4DOF")
                else:
                    node.get_logger().info("Navigation Mode: 6DOF")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print('Keyboard interrupt, shutting down.')
    finally:
        node.stop()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
