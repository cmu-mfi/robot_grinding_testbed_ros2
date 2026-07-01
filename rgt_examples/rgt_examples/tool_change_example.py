from rclpy.node import Node
import rclpy
from rgt_interfaces.srv import ChangeTool

class ToolChangeExample(Node):
    def __init__(self):
        super().__init__('tool_change_examples')
        self.change_tool_client = self.create_client(ChangeTool, "/rgt_manager/change_tool")
        while not self.change_tool_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def change_tool(self, robot: str, tool: str):
        req = ChangeTool.Request()
        req.robot = robot
        req.tool = tool
        future = self.change_tool_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    node = ToolChangeExample()

    node.change_tool("ur20", "sander")
    node.change_tool("nex10", "grinder")
    node.change_tool("nex10", "")
    node.change_tool("ur20", "")

if __name__ == '__main__':
    main()
