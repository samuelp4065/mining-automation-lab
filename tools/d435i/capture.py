"""Live RealSense preview. Space = save a picture, q/Esc = quit.

Saves color images to captures/ (ready to upload to Roboflow). Set SAVE_DEPTH = True
to also save the aligned raw depth as .npy (multiply by depth_scale for meters).
"""
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

WIDTH, HEIGHT, FPS = 1280, 720, 30
SAVE_DIR = Path(__file__).parent / "captures"
SAVE_DEPTH = False

SAVE_DIR.mkdir(exist_ok=True)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
if SAVE_DEPTH:
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
profile = pipeline.start(config)
align = rs.align(rs.stream.color) if SAVE_DEPTH else None

count = len(list(SAVE_DIR.glob("*.png")))
flash_until = 0.0

try:
    while True:
        frames = pipeline.wait_for_frames()
        if align:
            frames = align.process(frames)
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        color = np.asanyarray(color_frame.get_data())

        # Draw overlays on a copy so saved pictures stay clean
        preview = color.copy()
        if time.time() < flash_until:
            cv2.rectangle(preview, (0, 0), (WIDTH - 1, HEIGHT - 1), (255, 255, 255), 12)
        cv2.putText(preview, f"Saved: {count}   [space] capture   [q] quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("Capture", preview)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(" "):
            stamp = time.strftime("%Y%m%d_%H%M%S") + f"_{int(time.time() * 1000) % 1000:03d}"
            cv2.imwrite(str(SAVE_DIR / f"img_{stamp}.png"), color)
            if SAVE_DEPTH:
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    np.save(SAVE_DIR / f"depth_{stamp}.npy", np.asanyarray(depth_frame.get_data()))
            count += 1
            flash_until = time.time() + 0.15
            print(f"Saved img_{stamp}.png ({count})")
        elif key in (ord("q"), 27):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
