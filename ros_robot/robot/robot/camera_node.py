import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

from .camera_lib import YOLOTracker, DETECTION_STATIC, DETECTION_MOV

class YoloMovementNode(Node):
    def __init__(self):
        super().__init__('yolo_movement_node')

        self.tracker = YOLOTracker(
            model_path="yolo_models/yolov8n.pt",
            target_class="person" #person : for testing with yolo model not specifically trained
        )

        #Publisher : to publish movement status
        self.pub_status = self.create_publisher(String, 'obstacle_movement', 10)

        #Subscriber : to get the detection mode (depends on if robot moving or static)
        self.sub_mode = self.create_subscription(Int32, 'detection_mode', self.mode_callback, 10)
        self.detection_mode = DETECTION_STATIC  # default until first message
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Camera node started')

    def mode_callback(self, msg: Int32):
        self.detection_mode = msg.data  #store latest detection mode

    def timer_callback(self): #publishes movement_status
        try:
            status = self.tracker.yolo_loop(self.detection_mode) #"approaching_left"/"approaching_right"/"receding"/"stable"/"none"
            if status is not None:
                msg = String()
                msg.data = status
                self.pub_status.publish(msg)
                self.get_logger().info(f"Obstacle detection status: {status}")
        except Exception as e:
            self.get_logger().error(f"Exception in YOLO loop: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloMovementNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
