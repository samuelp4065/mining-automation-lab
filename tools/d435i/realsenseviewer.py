import time

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

WIDTH, HEIGHT, FPS = 640, 480, 30
MODEL_PATH = "/home/sw/TestD435i/runs/detect/runs/excavator_bucket-4/weights/best.pt"
CONF = 0.5  # minimum detection confidence

model = YOLO(MODEL_PATH)

# Configure color + depth streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
profile = pipeline.start(config)

device = profile.get_device()
print("Connected:", device.get_info(rs.camera_info.name),
      "| Serial:", device.get_info(rs.camera_info.serial_number),
      "| USB:", device.get_info(rs.camera_info.usb_type_descriptor))

depth_scale = device.first_depth_sensor().get_depth_scale()  # raw units -> meters

# Align depth to the color image so pixels match between the two views
align = rs.align(rs.stream.color)

# Filters that clean up depth noise and holes
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()
hole_filling = rs.hole_filling_filter()
colorizer = rs.colorizer()  # turns depth into a color map

mouse = [WIDTH // 2, HEIGHT // 2]

def on_mouse(event, x, y, flags, param):
    mouse[0], mouse[1] = x % WIDTH, y  # works on both halves of the window

def box_distance(depth_raw, x1, y1, x2, y2):
    """Median depth (m) of the box's central half; edges often hit the background."""
    w, h = x2 - x1, y2 - y1
    roi = depth_raw[y1 + h // 4:y2 - h // 4, x1 + w // 4:x2 - w // 4]
    valid = roi[roi > 0]
    return float(np.median(valid)) * depth_scale if valid.size else 0.0

cv2.namedWindow("RealSense", cv2.WINDOW_AUTOSIZE)
cv2.setMouseCallback("RealSense", on_mouse)

use_filters = True
use_detection = True
prev = time.time()

try:
    while True:
        frames = align.process(pipeline.wait_for_frames())
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        if use_filters:
            depth_frame = spatial.process(depth_frame)
            depth_frame = temporal.process(depth_frame)
            depth_frame = hole_filling.process(depth_frame).as_depth_frame()

        color = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data())
        depth_color = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        # Detect excavator buckets on the clean color image (before anything is drawn on it)
        if use_detection:
            result = model.predict(color, conf=CONF, device=0, verbose=False)[0]
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                name = model.names[int(box.cls[0])]
                d = box_distance(depth_raw, x1, y1, x2, y2)
                text = f"{name} {conf:.2f}  {d:.2f} m" if d > 0 else f"{name} {conf:.2f}  no depth"
                for img in (color, depth_color):
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 165, 255), 2)
                    cv2.putText(img, text, (x1, max(y1 - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 165, 255), 2)

        # Distance at the mouse position
        x, y = mouse
        dist = depth_raw[y, x] * depth_scale
        label = f"{dist:.3f} m" if dist > 0 else "no depth"
        for img in (color, depth_color):
            cv2.drawMarker(img, (x, y), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img, label, (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        now = time.time()
        fps = 1.0 / (now - prev)
        prev = now
        cv2.putText(color, f"FPS: {fps:.1f}  filters: {'on' if use_filters else 'off'}"
                           f"  detect: {'on' if use_detection else 'off'}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        view = np.hstack((color, depth_color))
        cv2.imshow("RealSense", view)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):          # q or Esc: quit
            break
        elif key == ord("f"):              # f: toggle depth filters
            use_filters = not use_filters
        elif key == ord("d"):              # d: toggle detection
            use_detection = not use_detection
        elif key == ord("s"):              # s: save snapshot
            stamp = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(f"color_{stamp}.png", np.asanyarray(color_frame.get_data()))
            np.save(f"depth_{stamp}.npy", depth_raw)  # raw depth, multiply by depth_scale for meters
            print("Saved", stamp)
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
