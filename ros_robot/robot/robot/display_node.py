import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

from .display_lib import LCDMenu, DISP_WAIT, DISP_MOVE

class LCDMenuNode(Node):
    def __init__(self):
        super().__init__('lcd_menu_node')

        #LCD menu setup
        self.menu = LCDMenu([
            "Destination point1", "Destination point2", "Destination point3",
            "Destination point4", "Destination point5", "Destination point6",
            "Destination point7", "Destination point8", "Destination point9",
            "Destination point10", "Destination point11", "Destination point12",
            "Destination point13"
        ])
        self.menu.set_mode(DISP_WAIT)

        #Publisher: to publish interface commands
        self.pub_cmd = self.create_publisher(
            String,
            'lcd_command', #topic name
            10
        )

        #Subscriber: get menu mode from fsm (DISP_WAIT/DISP_MOVE)
        self.sub_mode = self.create_subscription(
            Int32,
            'lcd_mode', #topic name
            self.mode_callback,
            10
        )

        #Timer
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("LCDMenu node started.")

    def mode_callback(self, msg: Int32):
        #called whenever navigation publishes a new display mode
        mode = msg.data #DISP_WAIT or DISP_MOVE
        if mode in (DISP_WAIT, DISP_MOVE):
            self.menu.set_mode(mode)
        else:
            self.get_logger().warn(f"Received invalid LCD mode: {mode}")

    def timer_callback(self): #publishes commands

        self.menu.run()

        cmd = self.menu.get_command()
        if cmd is not None:
            msg = String()
            msg.data = cmd
            self.pub_cmd.publish(msg)
            self.get_logger().info(f"LCD command: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = LCDMenuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
