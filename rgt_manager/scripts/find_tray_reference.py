#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
import pyspacemouse
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import threading
import numpy as np

class ProbeNode(Node):
    def __init__(self):
        super().__init__('probe_node')
        self.declare_parameter('ns', '')
        self.ns = str(self.get_parameter("ns").value)
        self.frame_id = self.ns + "_tool0"
        self.force = 0.0
        self.threshold = 0.5
        self.offset = 0.0

        # Twist Publisher
        self.twist_publisher = self.create_publisher(
            TwistStamped, 
            self.ns + '/servo_node/delta_twist_cmds', 
            10
        )

        # Enable Servo
        self.servo_client = self.create_client(ServoCommandType, self.ns + '/servo_node/switch_command_type')
        while not self.servo_client.wait_for_service(timeout_sec=1.0): pass
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Spacemouse
        self.smoothing_factor = 0.2  # Range: 0.0 to 1.0. Lower = smoother/slower response. Higher = sharper/faster response.
        self.smoothed_v = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Tracks [x, y, z, ax, ay, az]
        self.current_t = 0.0
        self.previous_t = 0.0
        self.skipped_ts = 0
        self.speed = 0.0
        self.turn = 0.0
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
    def navigate(self, locked=False):
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
            target_x = self.state.x * self.speed
            target_y = -self.state.y * self.speed
            target_z = -self.state.z * self.speed
            
            if locked:
                target_ax = 0.0
                target_ay = 0.0
            else:
                target_ax = -self.state.pitch * self.turn
                target_ay = -self.state.roll * self.turn
                
            target_az = self.state.yaw * self.turn

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

    #### Transforms
    def get_current_tf(self, parent, child):
        try:
            now = rclpy.time.Time()
            tf = self.tf_buffer.lookup_transform(
                parent,
                child,
                now)
            position = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
            rotation = np.array([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])
            return position, rotation
        except:
            position = np.array([np.NaN, np.NaN, np.NaN])
            rotation = np.array([np.NaN, np.NaN, np.NaN, np.NaN])
            return position, rotation

def main(args=None):
    rclpy.init(args=args)
    node = ProbeNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    mode = 0
    
    try:
        node.get_logger().info("Navigating!")
        while rclpy.ok():
            if mode == 0:
                node.speed = 0.4
                node.turn = 1.2
            if mode == 1:
                node.speed = 0.04
                node.turn = 0.12
            node.navigate()
            if node.left_button_pressed:
                node.left_button_pressed = False
                mode = not mode
            if node.right_button_pressed:
                node.right_button_pressed = False
                [x,y,z], [ax, ay, az, w] = node.get_current_tf(node.ns + "_base_link", node.ns + "_tc_hook")
                print("tool_changer_reference:")
                print("   parent_frame: " + node.ns + "_base_link")
                print("   x: " + str(x))
                print("   y: " + str(y))
                print("   z: " + str(z))
                print("   ax: " + str(ax))
                print("   ay: " + str(ay))
                print("   az: " + str(az))
                print("   w: " + str(w))
            time.sleep(0.002)

    except KeyboardInterrupt:
        print('Keyboard interrupt, shutting down.')
    finally:
        node.stop()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
