"""
EchoPath - Computer Vision Detection Module
Wraps the YOLOv8 model and turns raw detections into a simple directional
signal (LEFT / CENTER / RIGHT / NONE) based on where the closest/largest
relevant object sits in the camera frame.
"""

from ultralytics import YOLO

# ---- CONFIG ----
MODEL_PATH = "yolov8n.pt"
CONFIDENCE_THRESHOLD = 0.4

# All COCO classes relevant to navigation for the visually impaired.
# The direction alert is triggered by the largest of these in frame.
RELEVANT_CLASSES = {
    # People & animals
    "person", "cat", "dog", "horse", "cow",
    # Vehicles (outdoor hazards)
    "bicycle", "car", "motorcycle", "bus", "truck",
    # Indoor obstacles
    "chair", "couch", "bed", "dining table", "toilet", "door",
    # Carried / small objects
    "backpack", "handbag", "suitcase", "umbrella",
    "bottle", "cup", "laptop", "cell phone", "book",
    # Structural
    "bench", "potted plant", "tv",
}

DEAD_ZONE_PX = 100   # +/- pixels around center still counts as "CENTER"

# ---- MODEL (loaded once at import time) ----
_model = YOLO(MODEL_PATH)


def get_detections(frame):
    """
    Runs YOLOv8 on a frame and returns:
      direction    : 'LEFT' | 'CENTER' | 'RIGHT' | 'NONE'
      best_info    : dict with 'class', 'box' for the primary (largest) object,
                     or None if nothing detected
      all_detections: list of dicts [{class, box, confidence}] for ALL detected
                      relevant objects — used for drawing all boxes on screen
    """
    results = _model.predict(frame, verbose=False, conf=CONFIDENCE_THRESHOLD)
    frame_center_x = frame.shape[1] / 2

    all_detections = []
    best_box = None
    best_area = 0

    for r in results:
        for box in r.boxes:
            cls_name = _model.names[int(box.cls)]
            if cls_name not in RELEVANT_CLASSES:
                continue

            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            conf = float(box.conf[0])
            area = (x2 - x1) * (y2 - y1)

            all_detections.append({
                "class": cls_name,
                "box": (int(x1), int(y1), int(x2), int(y2)),
                "confidence": conf,
            })

            # Alert direction uses the largest (closest) relevant object
            if area > best_area:
                best_area = area
                best_box = (x1, y1, x2, y2, cls_name)

    if best_box is None:
        return "NONE", None, all_detections

    x1, y1, x2, y2, cls_name = best_box
    obj_center_x = (x1 + x2) / 2

    if obj_center_x < frame_center_x - DEAD_ZONE_PX:
        direction = "LEFT"
    elif obj_center_x > frame_center_x + DEAD_ZONE_PX:
        direction = "RIGHT"
    else:
        direction = "CENTER"

    best_info = {
        "class": cls_name,
        "box": (int(x1), int(y1), int(x2), int(y2)),
    }
    return direction, best_info, all_detections
