# import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time
import enum
from ultralytics import YOLO
from collections import Counter
from geojson_reader import EdgeDirection
from ament_index_python.packages import get_package_share_directory

class DetectedSign(enum.Enum):
    NONE = {
        'id': 0, 'closed_directions': []
    }
    ALLOWED_FORWARD = {
        'id': 1, 'closed_directions': [EdgeDirection.LEFT, EdgeDirection.RIGHT]
    }
    ALLOWED_RIGHT = {
        'id': 2, 'closed_directions': [EdgeDirection.LEFT, EdgeDirection.FORWARD]
    }
    ALLOWED_LEFT = {
        'id': 3, 'closed_directions': [EdgeDirection.FORWARD, EdgeDirection.RIGHT]
    }
    FORBIDDEN_RIGHT = {
        'id': 4, 'closed_directions': [EdgeDirection.RIGHT]
    }
    FORBIDDEN_LEFT = {
        'id': 5, 'closed_directions': [EdgeDirection.LEFT]
    }

    def __init__(self, vals):
        self.id = vals['id']
        self.closed_directions = vals['closed_directions']
    # BUS_STOP
    # PARKING
    # DANGEROUS_OBJECT


YOLO_CLASS_TO_ENUM = {
    "St_true": DetectedSign.ALLOWED_FORWARD,
    "right_true": DetectedSign.ALLOWED_RIGHT,
    "left_true": DetectedSign.ALLOWED_LEFT,
    "right_false": DetectedSign.FORBIDDEN_RIGHT,
    "Left_false": DetectedSign.FORBIDDEN_LEFT,
}

weights_path = os.path.join(get_package_share_directory('ramson_bot'), 'weights', 'best.pt')

model = YOLO(weights_path)

def detect_sign() -> DetectedSign:
    cap = cv2.VideoCapture(2, cv2.CAP_V4L2)

    # pipeline = rs.pipeline()
    # config = rs.config()
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # pipeline.start(config)

    # def get_frame():
    #     frames = pipeline.wait_for_frames()
    #     color_frame = frames.get_color_frame()
    #     if not color_frame:
    #         return None
    #     return np.asanyarray(color_frame.get_data())

    # for _ in range(5):
    #     pipeline.wait_for_frames()

    top_detections = []

    # try:
    for _ in range(5):
        # frame = get_frame()
        # if frame is None:
        #     continue
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame, conf=0.6)[0]
        max_area = 0
        chosen_class = None

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            area = (x2 - x1) * (y2 - y1)
            cls_name = model.names[int(box.cls)]
            if area > max_area and cls_name in YOLO_CLASS_TO_ENUM:
                max_area = area
                chosen_class = YOLO_CLASS_TO_ENUM[cls_name]

        if chosen_class:
            top_detections.append(chosen_class)

        time.sleep(0.2)

    # finally:
    #     # pipeline.stop()
    #     pass

    if top_detections:
        most_common = Counter(top_detections).most_common(1)[0]
        return most_common[0]
    else:
        return DetectedSign.NONE
