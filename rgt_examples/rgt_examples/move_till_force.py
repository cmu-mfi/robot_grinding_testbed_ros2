import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import ServoCommandType
import pyspacemouse
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations
import threading
import numpy as np
import tf_transformations

class MoveTillForce(Node):
    def __init__(self):
        super().__init__('move_till_force')
        self.declare_parameter('ns', '')
        self.ns = str(self.get_parameter("ns").value)
        self.frame_id = self.ns + "_tool0"
        self.force = 0.0
        self.threshold = 0.5
        self.offset = 0.0

        # Force Subscriber
        self.force_subscriber = self.create_subscription(
            WrenchStamped, 
            self.ns + '/force_torque_sensor_broadcaster/wrench_filtered',
            self.force_callback, 
            10
        )

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

    #### Movement
    def move_z(self, speed):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.frame_id
        twist_msg.twist.linear.z = speed
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

    #### Force Sensor
    def force_callback(self, msg: WrenchStamped):
        force = 0
        # force += abs(msg.wrench.force.x)
        # force += abs(msg.wrench.force.y)
        force += abs(msg.wrench.force.z)
        # force += abs(msg.wrench.torque.x)
        # force += abs(msg.wrench.torque.y)
        # force += abs(msg.wrench.torque.z)
        self.force = abs(abs(force) - abs(self.offset))
    def calibrate_sensor(self):
        self.get_logger().info("Starting sensor calibration")
        self.offset = 0.0
        data = []
        time.sleep(0.1)
        for _ in range(500):
            data.append(self.force)
            time.sleep(0.01)
        self.offset = sum(data) / len(data)
        time.sleep(0.1)
        self.get_logger().info(f"Calibration finished! Offset: {self.offset}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveTillForce()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        node.calibrate_sensor()
        while True:
            node.move_z(0.02)
            # print(node.force)
            if node.force >= 4:
                node.stop()
                return
            time.sleep(0.005)
    except KeyboardInterrupt:
        print('Keyboard interrupt, shutting down.')
    finally:
        node.stop()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()

