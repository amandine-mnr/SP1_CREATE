# from picamera2 import Picamera2

# picam2 = Picamera2()
# picam2.start()
# picam2.capture_file("test.jpg")
# picam2.stop()

from flask import Flask, Response
from picamera2 import Picamera2
import cv2
import threading

# Flask app
app = Flask(__name__)
frame_to_stream = None

# Initialize Picamera2
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(config)

# Auto white balance
try:
    from libcamera import controls
    picam2.set_controls({"AwbMode": controls.AwbModeEnum.Auto})
except ImportError:
    pass

picam2.start()

def capture_frames():
    global frame_to_stream
    while True:
        frame = picam2.capture_array()
        # Convert to RGB for correct colors
        frame_to_stream = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)

def gen_frames():
    global frame_to_stream
    while True:
        if frame_to_stream is None:
            continue
        ret, buffer = cv2.imencode('.jpg', frame_to_stream)
        if not ret:
            continue
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    threading.Thread(target=capture_frames, daemon=True).start()
    print("Streaming at http://<raspberry_pi_ip>:5000/video_feed")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)
