import time
import serial
from picamera2 import Picamera2
from ultralytics import YOLO
import cv2

# ==========================
# Serial setup for K64F
# ==========================
ser = serial.Serial("/dev/ttyACM0", 9600, timeout=0.1)
time.sleep(2)  # Wait for K64F / OpenSDA to settle

print("Serial port /dev/ttyACM0 opened.")

COOLDOWN = 1.0  # seconds
last_sent_time = {"cat": 0.0, "dog": 0.0}

def send_to_k64(msg: str):
    now = time.time()
    msg_lower = msg.lower()
    if msg_lower in last_sent_time:
        if now - last_sent_time[msg_lower] < COOLDOWN:
            return  # still in cooldown
        last_sent_time[msg_lower] = now
    line = (msg_lower + "\r\n").encode("utf-8")
    ser.write(line)
    print(f"Sent to K64F: {msg_lower}")

def read_from_k64():
    """Non-blocking read of any response lines from K64F."""
    while True:
        data = ser.readline()
        if not data:
            break
        print("K64F:", data.decode(errors="ignore").strip())


# ==========================
# Camera + YOLO setup
# ==========================

picam2 = Picamera2()
video_config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(video_config)
picam2.start()

model = YOLO("yolov8n.pt")  # will download on first run

print("YOLO + Picamera2 started. Press Ctrl+C in the terminal to quit.")

try:
    while True:
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        results = model(frame_bgr)
        r = results[0]

        detected_cat = False
        detected_dog = False

        if r.boxes is not None and r.boxes.cls is not None:
            class_ids = r.boxes.cls.tolist()
            names = r.names  # id -> label

            for cid in class_ids:
                cid_int = int(cid)
                label = names[cid_int] if cid_int in names else str(cid_int)
                label_lower = label.lower()

                if label_lower == "cat":
                    detected_cat = True
                elif label_lower == "dog":
                    detected_dog = True

        if detected_cat:
            send_to_k64("cat")
        if detected_dog:
            send_to_k64("dog")

        # Read and print any responses (e.g. "cat confirmed", "dog confirmed")
        read_from_k64()

except KeyboardInterrupt:
    print("Stopping on user request (Ctrl+C)...")

finally:
    print("Shutting down...")
    picam2.stop()
    picam2.close()
    ser.close()
