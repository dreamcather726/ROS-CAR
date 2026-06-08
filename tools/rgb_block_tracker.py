import time

import cv2
import numpy as np


WINDOW = "USB Camera"

# Frequently changed settings.
DEFAULT_CAMERA = "/dev/video20"
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_MIN_AREA = 300
DEFAULT_CLOSE_SIZE = 3
LOWER_RGB = np.array([140, 0, 0], dtype=np.uint8)
UPPER_RGB = np.array([255, 90, 90], dtype=np.uint8)


def find_target(frame):
    mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), LOWER_RGB, UPPER_RGB)
    kernel = np.ones((DEFAULT_CLOSE_SIZE, DEFAULT_CLOSE_SIZE), np.uint8)
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < DEFAULT_MIN_AREA:
        return None

    x, y, w, h = cv2.boundingRect(contour)
    moments = cv2.moments(contour)
    cx = x + w // 2 if moments["m00"] == 0 else int(moments["m10"] / moments["m00"])
    cy = y + h // 2 if moments["m00"] == 0 else int(moments["m01"] / moments["m00"])
    return x, y, w, h, cx, cy, area


def draw_status(frame, target, fps):
    h, w = frame.shape[:2]
    center = (w // 2, h // 2)
    cv2.line(frame, (center[0], 0), (center[0], h), (80, 80, 80), 1)
    cv2.line(frame, (0, center[1]), (w, center[1]), (80, 80, 80), 1)

    if target:
        x, y, bw, bh, cx, cy, area = target
        err_x = cx - center[0]
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
        text = f"target cx={cx} cy={cy} err_x={err_x} area={area:.0f}"
        text_color = (0, 255, 0)
    else:
        text = "target lost"
        text_color = (0, 0, 255)

    cv2.putText(frame, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, text_color, 2)
    cv2.putText(frame, f"fps {fps:.1f}", (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def main():
    cap = cv2.VideoCapture(DEFAULT_CAMERA, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {DEFAULT_CAMERA}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, DEFAULT_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, DEFAULT_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    fps = 0.0
    last_time = time.monotonic()

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        now = time.monotonic()
        current_fps = 1.0 / max(now - last_time, 1e-6)
        fps = 0.9 * fps + 0.1 * current_fps if fps else current_fps
        last_time = now

        draw_status(frame, find_target(frame), fps)
        cv2.imshow(WINDOW, frame)
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
