import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

from .config_api import *
from .API_func import *
from .lib import *
from .display_lib import DISP_WAIT, DISP_MOVE
from .camera_lib import DETECTION_STATIC, DETECTION_MOV
from .localization import *

# Robot states
WAITING = 100
TRAVELING = 101
AVOIDING = 102
AVOIDING_W = 103

LOOK_AHEAD = 0.2 #[m] add forward offset when searching safe point

# Navigation
class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')

        # API/robot init
        self.access_token = get_access_token()
        self.robot_id = get_robot_id(self.access_token)
        self.robot = init_robot(self.access_token, self.robot_id)

        floor_info = "1"
        case_type = "food"

        self.scene_list = get_scene_list(self.access_token)
        self.point_list = get_point_list(self.access_token, self.robot.scene_code)
        self.target_list = get_map_info(self.access_token, self.robot.scene_code, floor_info, case_type)
        for p in self.target_list:
            if p.type == "charge":
                self.charge_posX = p.posX
                self.charge_posY = p.posY
        filter_normal_targets(self.target_list)
        self.merged_point_list = merge_points(self.point_list, self.target_list)
        self.left_backward_points, self.left_forward_points, self.right_backward_points, self.right_forward_points = sort_merged_points(self.merged_point_list)

        # Initial position
        self.robot.posX = self.charge_posX
        self.robot.posY = self.charge_posY
        self.robot.floor = '1'

        self.loc = DeadReckoningLoc(self.robot.posX, self.robot.posY, theta0=0.0)

        # FSM state
        self.robot_state = WAITING
        self.current_taskNo = None
        self.avoidance_task_No = None
        self.main_task_No = None

        # Goal identifiers + coordinates
        self.goal_uuid = None
        self.goal_id = None
        self.goal_pos_x = None
        self.goal_pos_y = None

        # Data coming from other nodes
        self.movement_status = "none" # "approaching_left"/"approaching_right"/"receding"/"stable"/"none"
        self.lcd_cmd = None         # "stop" or destination string

        # Suscribers : to get obstacle status and user commands
        self.sub_camera = self.create_subscription(String, 'obstacle_movement', self.camera_callback, 10)
        self.sub_lcd = self.create_subscription(String, 'lcd_command', self.lcd_callback, 10)

        # Publisher : to control LCD mode (WAIT or MOVE)
        self.pub_lcd_mode = self.create_publisher(Int32, 'lcd_mode', 10)

        # Publisher : to share robot motion state (static/moving) with camera node
        self.pub_detection_mode = self.create_publisher(Int32, 'detection_mode', 10)

        # Timer
        self.last_print = 0.0
        self.timer = self.create_timer(0.1, self.main_loop)

        self.get_logger().info("FSM node started.")

    def camera_callback(self, msg: String):
        self.movement_status = msg.data

    def lcd_callback(self, msg: String):
        self.lcd_cmd = msg.data

    def loc_step(self):
        x, y, theta, dt = self.loc.update()
        self.robot.posX = x
        self.robot.posY = y
        self.robot.theta = theta
        return x, y, theta, dt
    
    def find_avoidance_point(self, status):
        orientation = (self.robot.theta <= math.pi and self.robot.theta >= 0)
        if(status == "approaching_left"):
            merged_points= self.right_forward_points if orientation else self.left_backward_points
        else:
            merged_points = self.left_forward_points if orientation else self.right_backward_points

        x_front = self.loc.x + LOOK_AHEAD * math.cos(self.robot.theta)
        y_front = self.loc.y + LOOK_AHEAD * math.sin(self.robot.theta)

        return(find_closest_merged_point(x_front, y_front, merged_points))

    def main_loop(self):
        try:
            status = self.movement_status

            #update pose
            pos_x, pos_y, theta, dt = self.loc_step()

            #print every second
            # now = time.time()
            # if now - self.last_print > 1.0:
                # self.get_logger().info(
                #     f"Movement: {status} | Loc: x={pos_x:.1f} y={pos_y:.1f} th={theta:.2f} dt={dt:.3f} robot_state:{self.robot_state}"
                # )
                # self.last_print = now
            
            #print every step
            self.get_logger().info(
                    f"Movement: {status} | Loc: x={pos_x:.1f} y={pos_y:.1f} th={theta:.2f} dt={dt:.3f} robot_state:{self.robot_state}"
                )

            # LCD
            cmd = self.lcd_cmd
            self.lcd_cmd = None

            if cmd == "stop":
                self.get_logger().info("Stopped robot task")
                cancel_robot_call(self.access_token, self.current_taskNo)
                self.loc.stopped()
                self.current_taskNo = None
                self.robot_state = WAITING
                self.loc.clear_goal()
                self.goal_uuid = None
                self.goal_id = None
                self.goal_pos_x = None
                self.goal_pos_y = None

            elif cmd:
                self.get_logger().info(f"Received new command : {cmd}")
                if self.current_taskNo:
                    self.get_logger().info("Cancelled old task to replace it")
                    cancel_robot_call(self.access_token, self.current_taskNo)
                self.goal_uuid, self.goal_id, self.goal_pos_x, self.goal_pos_y = point_info_from_name(cmd, self.merged_point_list)
                self.loc.set_goal(self.goal_pos_x, self.goal_pos_y)
                self.main_task_No, main_task_waitQ = initiate_remote_call(self.access_token, self.goal_uuid, self.goal_id, self.robot_id)
                self.loc.moving()
                self.current_taskNo = self.main_task_No
                self.get_logger().info("New task given")
                self.robot_state = TRAVELING

            if self.current_taskNo:
                task_status = query_task_status(self.access_token, self.current_taskNo)
                if task_status == TASK_COMPLETED_CODE:
                    self.current_taskNo = None
                    if self.robot_state == TRAVELING:
                        self.get_logger().info("destination reached")

                        #Localization
                        if self.goal_pos_x is not None and self.goal_pos_y is not None:
                            self.loc.set_pose(self.goal_pos_x, self.goal_pos_y, theta=None)

                        self.loc.clear_goal()
                        self.robot_state = WAITING
                    if self.robot_state in (AVOIDING, AVOIDING_W):
                        if status == "approaching_left" or status == "approaching_right":
                            #skip everything else
                            return

            # waiting + obstacle approaching -> avoidance while waiting
            if self.robot_state == WAITING and (status == "approaching_left" or status == "approaching_right"):                
                closest_merged_point = self.find_avoidance_point(status)
                if closest_merged_point is not None :
                    self.get_logger().info("closest point (WAITING):")
                    closest_merged_point.pretty_print()

                    #set dead reckonning goal to avoidance point
                    avoid_x = closest_merged_point.target.posX
                    avoid_y = closest_merged_point.target.posY
                    self.loc.set_goal(avoid_x, avoid_y)

                    self.avoidance_task_No, avoidance_task_waitQ = initiate_remote_call(self.access_token, closest_merged_point.point.uuid, closest_merged_point.point.pointId, self.robot_id)
                    self.loc.moving()
                    self.current_taskNo = self.avoidance_task_No
                    self.robot_state = AVOIDING_W
                    self.get_logger().info("(WAITING -> AVOIDING_W)")

            # traveling + obstacle approaching -> cancel main task, start avoidance
            if (self.robot_state == TRAVELING and (status == "approaching_left" or status == "approaching_right")):
                if self.current_taskNo:
                    self.get_logger().info("cancelling main task for avoidance")
                    cancel_robot_call(self.access_token, self.main_task_No)
                closest_merged_point = self.find_avoidance_point(status)
                if closest_merged_point is not None :
                    self.get_logger().info("closest point (TRAVELING):")
                    closest_merged_point.pretty_print()

                    #set dead reckonning goal to avoidance point
                    avoid_x = closest_merged_point.target.posX
                    avoid_y = closest_merged_point.target.posY
                    self.loc.set_goal(avoid_x, avoid_y)

                    self.avoidance_task_No, avoidance_task_waitQ = initiate_remote_call(self.access_token, closest_merged_point.point.uuid, closest_merged_point.point.pointId, self.robot_id)
                    self.current_taskNo = self.avoidance_task_No
                    self.robot_state = AVOIDING
                    self.get_logger().info("(TRAVELING -> AVOIDING)")

            # avoiding + obstacle no longer approaching -> resume main task
            if self.robot_state == AVOIDING and (status == "stable" or status == "receding" or status == "none"):
                if self.current_taskNo:
                    self.get_logger().info("cancelling avoidance")
                    cancel_robot_call(self.access_token, self.avoidance_task_No)

                #restore dead reckonning goal to main destination
                if self.goal_pos_x is not None and self.goal_pos_y is not None:
                    self.loc.set_goal(self.goal_pos_x, self.goal_pos_y)
                else:
                    self.loc.clear_goal()

                self.main_task_No, main_task_waitQ = initiate_remote_call(self.access_token, self.goal_uuid, self.goal_id, self.robot_id)
                self.current_taskNo = self.main_task_No
                self.robot_state = TRAVELING
                self.get_logger().info("(AVOIDING -> TRAVELING)")

            # avoiding_w + obstacle gone -> back to waiting
            if self.robot_state == AVOIDING_W and (status == "stable" or status == "receding" or status == "none"):
                if self.current_taskNo:
                    self.get_logger().info("cancelling avoidance (w)")
                    cancel_robot_call(self.access_token, self.avoidance_task_No)
                self.current_taskNo = None
                self.robot_state = WAITING

                #if no main travel goal, clear dead reckonning goal too
                if self.goal_pos_x is None:
                    self.loc.clear_goal()

                self.get_logger().info("(AVOIDING_W -> WAITING)")

            #Sync LCD mode
            mode_msg = Int32()
            if self.robot_state == WAITING and self.current_taskNo is None:
                mode_msg.data = DISP_WAIT
            else:
                mode_msg.data = DISP_MOVE
            self.pub_lcd_mode.publish(mode_msg)

            #Sync motion state for obstacle detection
            mot_msg = Int32()
            if self.robot_state in (TRAVELING, AVOIDING, AVOIDING_W):
                mot_msg.data = DETECTION_MOV
            else:
                mot_msg.data = DETECTION_STATIC
            self.pub_detection_mode.publish(mot_msg)

        except Exception:
            self.get_logger().error("!!!!! EXCEPTION IN MAIN LOOP !!!!!")
            import traceback
            traceback.print_exc()
            return

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
