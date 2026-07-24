"""
EchoPath - Main Pipeline (Phase 1 — CV standalone, no ESP32 yet)
=================================================================
Fuses local YOLOv8 object detection with serial distance data to
compute LEFT/CENTER/RIGHT navigation alerts.

Buzzer logic:
  - Direction commands (LEFT / CENTER / RIGHT) are sent to the ESP32
    whenever an alert is active AND the state changes.
  - Additionally, while alert_active is True (object inside ultrasonic
    threshold), a BEEP is pulsed every BEEP_INTERVAL seconds so the
    buzzer keeps sounding even if the direction doesn't change.
  - Sending NONE turns the buzzer off immediately.
"""

import os
import time
import cv2
import numpy as np
import supervision as sv
from dotenv import load_dotenv

from serial_comm import get_distance, send_command, is_connected, shutdown
from cv_detection import get_detections

load_dotenv()

# ─── CONFIG ───────────────────────────────────────────────────────────────────
CAMERA_INDEX       = 1      # 1 = external USB webcam, 0 = built-in laptop cam
CAM_WIDTH          = 1280
CAM_HEIGHT         = 720
ENTER_THRESHOLD_CM = 50     # trigger alert when object is closer than this
EXIT_THRESHOLD_CM  = 60     # must move further than this to clear (hysteresis)
BEEP_INTERVAL      = 0.6    # seconds between repeated buzzer pulses while alert active

# ─── Camera ───────────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
if not cap.isOpened():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError("No camera found at index 1 or 0.")
    print("[Camera] USB camera not found — using built-in camera.")
else:
    print(f"[Camera] Using USB camera (index {CAMERA_INDEX}).")

cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)

# ─── Supervision annotators ───────────────────────────────────────────────────
COLORS = sv.ColorPalette.from_hex([
    "#FF4757",  # red
    "#2ED573",  # green
    "#1E90FF",  # blue
    "#FFA502",  # orange
    "#A29BFE",  # purple
    "#00CEC9",  # teal
    "#FDCB6E",  # yellow
    "#E17055",  # salmon
])

box_annotator = sv.BoxAnnotator(
    thickness=3,
    color=COLORS,
    color_lookup=sv.ColorLookup.INDEX,
)
label_annotator = sv.LabelAnnotator(
    text_scale=0.65,
    text_thickness=2,
    text_padding=8,
    color=COLORS,
    color_lookup=sv.ColorLookup.INDEX,
    text_position=sv.Position.TOP_LEFT,
)

# ─── Alert / buzzer state ─────────────────────────────────────────────────────
last_command      = "NONE"
alert_active      = False
last_beep_time    = 0.0     # tracks when we last sent a buzzer pulse

# ─── Alert color map for HUD text ─────────────────────────────────────────────
ALERT_COLOR = {
    "LEFT":   (0, 165, 255),   # orange
    "RIGHT":  (0, 165, 255),   # orange
    "CENTER": (0,   0, 255),   # red  — directly ahead = most urgent
    "NONE":   (0, 255,   0),   # green — safe
}

# ─── HUD helper ───────────────────────────────────────────────────────────────
def draw_hud(frame, distance, command, obj_class, connected):
    h, w = frame.shape[:2]

    # ── semi-transparent dark panel ──────────────────────────────────────────
    panel_w, panel_h = 420, 155
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (10 + panel_w, 10 + panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    # ── Distance row ─────────────────────────────────────────────────────────
    dist_text = f"{distance} cm" if distance != -1 else "-- cm"
    dist_col  = (0, 255, 255)   # cyan
    cv2.putText(frame, "DIST :", (20, 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (180, 180, 180), 1)
    cv2.putText(frame, dist_text, (125, 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.90, dist_col, 2)

    # ── Alert row ────────────────────────────────────────────────────────────
    alert_col = ALERT_COLOR.get(command, (0, 255, 0))
    cv2.putText(frame, "ALERT:", (20, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (180, 180, 180), 1)
    cv2.putText(frame, command, (125, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.90, alert_col, 2)

    # ── Object row ───────────────────────────────────────────────────────────
    obj_text = obj_class if obj_class else "none"
    cv2.putText(frame, "OBJ  :", (20, 130),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (180, 180, 180), 1)
    cv2.putText(frame, obj_text, (125, 130),
                cv2.FONT_HERSHEY_SIMPLEX, 0.90, (255, 255, 255), 2)

    # ── ESP32 / buzzer status badge ──────────────────────────────────────────
    badge_col  = (0, 200, 80) if connected else (60, 60, 60)
    badge_text = "ESP32 OK" if connected else "ESP32 --"
    cv2.rectangle(frame, (w - 150, 10), (w - 10, 42), badge_col, -1)
    cv2.putText(frame, badge_text, (w - 143, 33),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    # ── Buzzer pulsing indicator (flashes red when beeping) ─────────────────
    if alert_active and command != "NONE":
        buzz_col = (0, 0, 220) if (int(time.time() * 2) % 2 == 0) else (0, 0, 100)
        cv2.circle(frame, (w - 180, 26), 10, buzz_col, -1)
        cv2.putText(frame, "BUZZ", (w - 225, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, buzz_col, 2)

    return frame

# ─── Main loop ────────────────────────────────────────────────────────────────
print("[EchoPath] CV pipeline running. Press Esc or Q to quit.")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[EchoPath] Frame read failed — camera disconnected?")
            break

        # ── Vision detection ──────────────────────────────────────────────────
        direction, best_info, all_dets = get_detections(frame)
        obj_class = best_info["class"] if best_info else None

        # ── Ultrasonic distance + hysteresis ──────────────────────────────────
        distance = get_distance()
        if distance != -1 and distance < ENTER_THRESHOLD_CM:
            alert_active = True
        elif distance == -1 or distance > EXIT_THRESHOLD_CM:
            alert_active = False
        # buffer zone (ENTER < d < EXIT): keep previous state

        # ── Alert command ─────────────────────────────────────────────────────
        # Alert fires only when vision AND ultrasonic both agree there's an object
        command = direction if (alert_active and direction != "NONE") else "NONE"

        # ── Direction change → send to ESP32 immediately ─────────────────────
        if command != last_command:
            send_command(command)
            last_command   = command
            last_beep_time = time.monotonic()

        # ── Periodic BEEP pulse while alert is active ─────────────────────────
        # This keeps the buzzer sounding even when direction doesn't change.
        elif alert_active and command != "NONE":
            now = time.monotonic()
            if now - last_beep_time >= BEEP_INTERVAL:
                send_command(command)   # re-send same direction to re-trigger buzzer
                last_beep_time = now

        # ── Draw ALL detected objects with supervision ─────────────────────────
        if all_dets:
            xyxy      = []
            labels    = []
            class_ids = []

            for i, det in enumerate(all_dets):
                x1, y1, x2, y2 = det["box"]
                xyxy.append([float(x1), float(y1), float(x2), float(y2)])
                labels.append(f"  {det['class']}  {det['confidence']:.0%}  ")
                class_ids.append(i)

            detections = sv.Detections(
                xyxy=np.array(xyxy, dtype=float),
                class_id=np.array(class_ids, dtype=int),
            )
            frame = box_annotator.annotate(frame, detections)
            frame = label_annotator.annotate(frame, detections, labels=labels)

        # ── HUD ───────────────────────────────────────────────────────────────
        frame = draw_hud(frame, distance, command, obj_class, is_connected())

        cv2.imshow("EchoPath  |  Q / Esc = quit", frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        if cv2.getWindowProperty("EchoPath  |  Q / Esc = quit",
                                  cv2.WND_PROP_VISIBLE) < 1:
            break

finally:
    send_command("NONE")        # silence buzzer on exit
    cap.release()
    cv2.destroyAllWindows()
    shutdown()
    print("[EchoPath] Stopped cleanly.")
