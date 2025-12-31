from flask import Flask, Response
from picamera2 import Picamera2
import threading
import time
import cv2
from PIL import Image

app = Flask(__name__)

GRAY = False

# Initialize Picamera2
picam2 = Picamera2()
preview_config = picam2.create_video_configuration(main={"format": "RGB888", "size": (320, 240)})
picam2.configure(preview_config)
picam2.start()

def gen_frames():
    while True:
        frame = picam2.capture_array()
        if GRAY :
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return """
    <html>
    <head><title>Pi Camera Stream</title></head>
    <body>
        <h1>Live Feed</h1>
        <img src="/video_feed">
        <p>Press 'p' in the terminal to capture an image.</p>
        <p>Press 'q' + Enter to quit.</p>
    </body>
    </html>
    """

def keyboard_listener():
    while True:
        cmd = input()
        if cmd.lower() == 'p':
            filename = f"capture_{int(time.time())}.png"
            frame = picam2.capture_array() 
            if GRAY :
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            else :
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            # Save as PNG
            Image.fromarray(frame).save(filename)
            print(f"Captured {filename}")
        elif cmd.lower() == 'q':
            print("Quitting...")
            break

if __name__ == "__main__":
    # Start keyboard listener in a separate thread
    listener_thread = threading.Thread(target=keyboard_listener, daemon=True)
    listener_thread.start()

    # Start Flask server
    app.run(host='0.0.0.0', port=5000)
