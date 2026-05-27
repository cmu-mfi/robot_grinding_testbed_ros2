from rclpy.node import Node
import rclpy
from rgt_interfaces.srv import TakeTool, ReturnTool

class ToolChangeExample(Node):
    def __init__(self):
        super().__init__('tool_change_examples')
        self.take_tool_client = self.create_client(TakeTool, "/rgt_manager/take_tool")
        self.return_tool_client = self.create_client(ReturnTool, "/rgt_manager/return_tool")
        while not self.take_tool_client.wait_for_service(timeout_sec=1.0) or not self.return_tool_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def take_tool(self, robot_ns: str, tool_changer_id: int):
        req = TakeTool.Request()
        req.robot_ns = robot_ns
        req.tool_changer_id = tool_changer_id
        future = self.take_tool_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def return_tool(self, robot_ns: str, tool_changer_id: int):
        req = ReturnTool.Request()
        req.robot_ns = robot_ns
        req.tool_changer_id = tool_changer_id
        future = self.return_tool_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    node = ToolChangeExample()

    node.take_tool("nex10", 1)
    node.return_tool("nex10", 1)
    node.take_tool("nex10", 2)
    node.return_tool("nex10", 2)
    node.take_tool("ur20", 1)
    node.return_tool("ur20", 1)
    node.take_tool("ur20", 2)
    node.return_tool("ur20", 2)

if __name__ == '__main__':
    main()
