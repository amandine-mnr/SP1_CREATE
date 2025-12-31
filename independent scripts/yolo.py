import cv2
import time
import threading
import sys
from flask import Flask, Response
from ultralytics import YOLO
from picamera2 import Picamera2

# Load YOLO model
model = YOLO("yolov8n.pt")
# model = YOLO("custom_model.pt")
model.fuse()

model.overrides['conf'] = 0.0 # confidence threshold
model.overrides['iou'] = 0.45 # overlap threshold for NMS (Non-Maximum Suppression), removes duplicate overlapping boxes
model.overrides['max_det'] = 50 # max boxes per frame

# Setup Picamera2
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"format": "RGB888", "size": (320, 240)})
picam2.configure(config)
picam2.start()

frame_to_stream = None #global frame for streaming

# Flask app (for streaming)
app = Flask(__name__)

def gen_frames():
    global frame_to_stream
    while True:
        if frame_to_stream is None:
            time.sleep(0.01)
            continue
        ret, buffer = cv2.imencode('.jpg', frame_to_stream)
        if not ret:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# CPU temperature
def get_cpu_temp():
    with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
        temp_str = f.read().strip()
    return int(temp_str) / 1000  #convert millidegree to °C

def yolo_loop():
    global frame_to_stream
    max_temp = 80 #°C
    last_temp_check = 0
    temp_interval = 1 #sec

    while True:

        start_time = time.time()
        frame = picam2.capture_array()
        results = model.predict(frame, verbose=False)
        annotated = results[0].plot() #draw detections

        end_time = time.time()
        curr_fps = 1 / (end_time - start_time)

        # Check CPU temp every temp_interval seconds
        if time.time() - last_temp_check > temp_interval:
            cpu_temp = get_cpu_temp()
            last_temp_check = time.time()
            # Stop program if temp exceeds threshold
            if cpu_temp > max_temp:
                print(f"CPU Temp too high: {cpu_temp:.1f}°C! Stopping program.")
                picam2.stop()
                sys.exit(1)

        text = f"FPS: {curr_fps:.1f}  Temp: {cpu_temp:.1f}C"
        (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, 0.5, 1)
        cv2.rectangle(annotated, (5,5), (10 + w, 10 + h + 5), (0,0,0), -1)
        cv2.putText(annotated, text, (10, 22), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,255,255), 1)

        frame_to_stream = annotated
        time.sleep(0.01)

if __name__ == "__main__":
    # print(model.names)
    # Start Flask in a daemon thread
    flask_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, debug=False, threaded=True),
        daemon=True
    )
    
    flask_thread.start()

    print("\nhttp://<IP adress>:5000/video_feed\n")

    yolo_loop()
