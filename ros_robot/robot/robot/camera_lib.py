import time
import sys
import numpy as np
from collections import deque
from ultralytics import YOLO
from picamera2 import Picamera2

MAX_MISSING = 3 #admissible number of empty frames between detections
MAX_TEMP = 80 #°C, to check the Pi is not overheating
CAM_W = 320
CAM_H = 240
FOV_Horizontal = 66.0 #°
FOV_Vertical = 41.0 #°
f_length_pix = CAM_W / (2.0 * np.tan(np.deg2rad(FOV_Horizontal / 2.0)))

REAL_OBJECT_W = 0.4  #m (real obstacle width)
ROBOT_SPEED = 0.8 #m/s

# States
DETECTION_STATIC = 300
DETECTION_MOV = 301

def get_cpu_temp():
    with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
        return int(f.read().strip()) / 1000 #°C

class YOLOTracker:

    def __init__(self, model_path="yolo_models/yolov8n.pt", target_class="person"): #person : for testing
        self.target_class = target_class.lower()

        # Tracking state
        self.history = deque(maxlen=10)
        self.prev_box_coords = None #previous tracked box
        self.missing_frames = 0
        self.last_temp_check = 0
        self.temp_interval = 1.0 #sec
        self.cpu_temp = get_cpu_temp()
        self.last_loop_time = time.time()
        self.last_detection_mode = None

        # Load model
        self.model = YOLO(model_path)

        # Setup camera
        self.picam2 = Picamera2()
        self.init_yolo()

    def init_yolo(self):
        self.model.fuse()
        self.model.overrides['conf'] = 0.0 # confidence threshold
        self.model.overrides['iou'] = 0.45 # overlap threshold for NMS (Non-Maximum Suppression), removes duplicate overlapping boxes
        self.model.overrides['max_det'] = 50 # max boxes per frame

        config = self.picam2.create_video_configuration(
            main={"format": "RGB888", "size": (CAM_W, CAM_H)}
        )
        self.picam2.configure(config)
        self.picam2.start()

    @staticmethod
    def box_center(box):
        x1, y1, x2, y2 = box
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        return cx, cy

    @staticmethod
    def boxes_close(box1, box2, max_distance=50):
        cx1, cy1 = YOLOTracker.box_center(box1)
        cx2, cy2 = YOLOTracker.box_center(box2)
        dist = ((cx1 - cx2)**2 + (cy1 - cy2)**2)**0.5
        return dist < max_distance

    @staticmethod
    def compute_distance_from_width(real_width_m, box_width_px, f_length_px):
        return (f_length_px * real_width_m) / max(1.0, box_width_px)
    
    def yolo_loop(self, detection_mode):

        # Reset tracking when mode changes (STATIC <-> MOVING)
        if self.last_detection_mode is None:
            self.last_detection_mode = detection_mode
        elif detection_mode != self.last_detection_mode:
            self.history.clear()
            self.prev_box_coords = None
            self.missing_frames = 0
            self.last_detection_mode = detection_mode

        if detection_mode == DETECTION_MOV:
            speed = ROBOT_SPEED
        else:
            speed = 0

        # Time handling
        curr_time = time.time()
        dt = curr_time - self.last_loop_time
        self.last_loop_time = curr_time

        # Capture frame
        frame = self.picam2.capture_array()
        results = self.model.predict(frame, verbose=False)[0]

        # Filter target class
        detected = []
        for box in results.boxes:
            cls_id = int(box.cls[0])
            if self.model.names[cls_id].lower() == self.target_class:
                detected.append(box)

        movement_status = "none"

        if detected:
            biggest_box = None
            biggest_area = 0.0
            best_far_box = None
            best_far_area = 0.0

            # Choose appropriate box
            for box in detected:
                x1, y1, x2, y2 = box.xyxy[0].numpy()
                area = max(1.0, (x2 - x1) * (y2 - y1)) #at least 1 squared pixel to avoid 0

                if self.prev_box_coords is not None:
                    if self.boxes_close((x1, y1, x2, y2), self.prev_box_coords, max_distance=50):
                        if area > biggest_area:
                            biggest_box = (x1, y1, x2, y2)
                            biggest_area = area
                    else:
                        if area > best_far_area:
                            best_far_box = (x1, y1, x2, y2)
                            best_far_area = area
                else:
                    #no previous box -> choose biggest normally
                    if area > biggest_area:
                        biggest_box = (x1, y1, x2, y2)
                        biggest_area = area

            # Choose far box if it becomes the largest
            if (biggest_box is None and best_far_box is not None) or (best_far_area > biggest_area):
                biggest_box = best_far_box
                biggest_area = best_far_area
                self.prev_box_coords = None
                self.history.clear()

            if biggest_box is None:
                self.missing_frames += 1
                if self.missing_frames > MAX_MISSING:
                    self.history.clear()
                    self.prev_box_coords = None
                return None
            self.missing_frames = 0
            x1, y1, x2, y2 = biggest_box

            w_px = max(1.0, x2 - x1)
            h_px = max(1.0, y2 - y1)
            area = w_px * h_px

            #Compute real distance using known object width and horizontal focal length
            dist = self.compute_distance_from_width(REAL_OBJECT_W, w_px, f_length_pix)

            self.history.append({
                'area': area,
                'box': biggest_box,
                'dist': dist,
                'w_px': w_px,
                'h_px': h_px,
                't': curr_time
            })

            self.prev_box_coords = biggest_box

           # Movement detection
            if len(self.history) >= 3:
                oldest = self.history[0]
                newest = self.history[-1]

                A_old = oldest['area']
                A_new = newest['area']
                D_old = oldest['dist']

                #robot displacement this frame (meters)
                dz = speed * (newest['t'] - oldest['t'])

                #predicted object distance after camera moves forward by dz
                D_pred = max(0.05, D_old - dz)

                #expected area after robot motion
                expected_A = A_old * (D_old / D_pred)**2

                #measured vs expected
                growth = A_new / max(1e-12, expected_A)

                if growth > 1.20:
                    cx = (x1 + x2) / 2.0  #box center in pix
                    if cx < CAM_W / 2.0:
                        movement_status = "approaching_left"
                    else:
                        movement_status = "approaching_right"
                elif growth < 0.80:
                    movement_status = "receding"
                else:
                    movement_status = "stable"
            else:
                movement_status = "none"

        else:
            #no box detected
            self.missing_frames += 1
            
            if self.missing_frames > MAX_MISSING:
                self.history.clear()
                self.prev_box_coords = None
                movement_status = "none"
            else:
                movement_status = "none"

        if dt > 0:
            fps = 1.0 / dt #frames per second

       # Check CPU temp every temp_interval seconds
        if time.time() - self.last_temp_check > self.temp_interval:
            self.cpu_temp = get_cpu_temp()
            self.last_temp_check = time.time()
            #stop program if temp exceeds threshold
            if self.cpu_temp > MAX_TEMP:
                print(f"CPU Temp too high: {self.cpu_temp:.1f}°C! Stopping program.")
                self.picam2.stop()
                sys.exit(1)

        return movement_status
