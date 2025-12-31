import cv2
import time
import threading
import sys
import numpy as np
from collections import deque
from flask import Flask, Response
from ultralytics import YOLO
from picamera2 import Picamera2

ROBOT_SPEED = 0.06

CAM_W = 320
CAM_H = 240
f_length_mm = 4.74
FOV_Horizontal = 66.0 #°
FOV_Vertical = 41.0 #°
REAL_OBJECT_W = 0.15  #m (real object width)

f_length_pix = CAM_W / (2.0 * np.tan(np.deg2rad(FOV_Horizontal / 2.0)))

model = YOLO("yolov8n.pt")
# model = YOLO("custom_model.pt")
model.fuse()

model.overrides['conf'] = 0.0 # confidence threshold
model.overrides['iou'] = 0.45 # overlap threshold for NMS (Non-Maximum Suppression), removes duplicate overlapping boxes
model.overrides['max_det'] = 50 # max boxes per frame

MAX_MISSING = 3
MAX_TEMP = 80 #°C

#setup Picamera2
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"format": "RGB888", "size": (CAM_W, CAM_H)})
picam2.configure(config)
picam2.start()

#global frame for streaming
frame_to_stream = None

#flask app (for streaming)
app = Flask(__name__)

def gen_frames():
    global frame_to_stream
    while True:
        if frame_to_stream is None:
            time.sleep(0.01)
            continue
        ret, buffer = cv2.imencode('.jpg', frame_to_stream, [cv2.IMWRITE_JPEG_QUALITY, 50])

        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

#CPU temperature
def get_cpu_temp():
    with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
        temp_str = f.read().strip()
    return int(temp_str) / 1000  #convert millidegree to °C

#movement tracking
history = deque(maxlen=10)
prev_box_coords = None #previous tracked box

def box_center(box):
    x1, y1, x2, y2 = box
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    return cx, cy

def boxes_close(box1, box2, max_distance=50):
    cx1, cy1 = box_center(box1)
    cx2, cy2 = box_center(box2)
    dist = ((cx1 - cx2)**2 + (cy1 - cy2)**2)**0.5
    return dist < max_distance

def compute_distance_from_width(real_width_m, box_width_px, focal_length_px):
    return (focal_length_px * real_width_m) / box_width_px

def yolo_loop():
    global frame_to_stream, prev_box_coords, MAX_TEMP, MAX_MISSING, history
    missing_frames = 0
    last_temp_check = 0
    temp_interval = 1.0  #sec
    loop_start = 0.0
    cpu_temp = get_cpu_temp()

    while True:
        
        prev_time = loop_start
        loop_start = time.time()
        frame = picam2.capture_array()

        results = model.predict(frame, verbose=False)
        #annotated = frame.copy()

############# FILTER TO KEEP ONLY A SPECIFIC OBJECT ##################
        filtered_boxes = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0].item())
            cls_name = model.names[cls_id].lower()
            if cls_name == "person":
                filtered_boxes.append(box)

        results[0].boxes = filtered_boxes
######################################################################

        annotated = results[0].plot()
        movement_status = "No object"

        if len(results[0].boxes) > 0:

            biggest_box = None
            biggest_area = 0.0
            best_far_box = None
            best_far_area = 0.0

            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].numpy()
                area = max(1.0, (x2-x1)*(y2-y1)) #at least 1 squared pixel to avoid 0

                if prev_box_coords is not None:
                    if boxes_close((x1, y1, x2, y2), prev_box_coords, max_distance=50):
                        if area > biggest_area:
                            biggest_area = area
                            biggest_box = (x1, y1, x2, y2)
                    else:
                        if area > best_far_area:
                            best_far_area = area
                            best_far_box = (x1, y1, x2, y2)
                else:
                    #no previous box -> choose biggest normally
                    if area > biggest_area:
                        biggest_area = area
                        biggest_box = (x1, y1, x2, y2)

            if (biggest_box is None and best_far_box is not None) or (best_far_area > biggest_area):
                biggest_box = best_far_box
                biggest_area = best_far_area
                history.clear()

            if biggest_box is None:
                missing_frames += 1

                if missing_frames > MAX_MISSING:
                    history.clear()
                    prev_box_coords = None
                    movement_status = "Searching"
                else:
                    movement_status = "Lost"

            else:
                #reset missing counter
                missing_frames = 0

                x1, y1, x2, y2 = biggest_box
                cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

                w_px = max(1.0, x2 - x1)
                h_px = max(1.0, y2 - y1)
                area = w_px * h_px

                #compute real distance using known object width and horizontal focal length
                dist = compute_distance_from_width(REAL_OBJECT_W, w_px, f_length_pix)

                history.append({'area': area, 'box': (x1, y1, x2, y2), 'dist': dist, 'w_px': w_px, 'h_px': h_px, 't': loop_start})

                prev_box_coords = (x1, y1, x2, y2)

                #movement detection
                if len(history) >= 3:
                    oldest = history[0]
                    newest = history[-1]
                    A_old = oldest['area']
                    A_new = newest['area']
                    D_old = oldest['dist']
                    D_new = newest['dist']

                    #robot displacement this frame (meters)
                    dz = ROBOT_SPEED * (newest['t'] - oldest['t'])

                    #predicted object distance after camera moves forward by dz
                    D_pred = max(0.05, D_old - dz)

                    #expected area after robot motion : A_pred = A_old * (D_old / D_pred)^2
                    expected_A_new = A_old * ((D_old / D_pred) ** 2)

                    #measured vs expected
                    growth = A_new / max(1e-12, expected_A_new)

                    APPROACH_THRESH = 1.20
                    RECEDE_THRESH = 0.80
                    if growth > APPROACH_THRESH:
                        movement_status = "Approaching"
                    elif growth < RECEDE_THRESH:
                        movement_status = "Receding"
                    else:
                        movement_status = "Stable distance"
                else:
                    movement_status = "Measuring"
        else:
            #no box detected
            missing_frames += 1

            if missing_frames > MAX_MISSING:
                history.clear()
                prev_box_coords = None
                movement_status = "No object"
            else:
                movement_status = "Lost"

        #update timing / fps
        curr_fps = 1.0 / (loop_start - prev_time)

        #check CPU temp every temp_interval seconds
        if time.time() - last_temp_check > temp_interval:
            cpu_temp = get_cpu_temp()
            last_temp_check = time.time()
            #stop program if temp exceeds threshold
            if cpu_temp > MAX_TEMP:
                print(f"CPU Temp too high: {cpu_temp:.1f}°C! Stopping program.")
                picam2.stop()
                sys.exit(1)

        text = f"FPS: {curr_fps:.1f}  Temp: {cpu_temp:.1f}C"
        (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, 0.5, 1)
        cv2.rectangle(annotated, (5, 5), (10 + w, 10 + h + 5), (0, 0, 0), -1)
        (mw, mh), _ = cv2.getTextSize(movement_status, cv2.FONT_HERSHEY_DUPLEX, 0.5, 1)
        cv2.rectangle(annotated, (5, 27), (10 + mw, 27 + mh + 5), (0, 0, 0), -1)
        
        cv2.putText(annotated, text, (10, 22), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(annotated, movement_status, (10, 40), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)

        frame_to_stream = annotated
        time.sleep(0.01)

if __name__ == "__main__":
    # print(model.names)
    #start Flask in a daemon thread
    flask_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, debug=False, threaded=True),
        daemon=True
    )
    flask_thread.start()

    print("\nhttp://<IP adress>:5000/video_feed\n")

    yolo_loop()
