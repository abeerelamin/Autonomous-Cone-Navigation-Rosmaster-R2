import cv2
import numpy as np


def get_color_range(color: str):
    """
    Return lower and upper HSV bounds for a given color name.
    """
    color = color.lower()

    if color == "blue":
        return np.array([100, 150, 0]), np.array([140, 255, 255])
    elif color == "red":
        return np.array([0, 120, 70]), np.array([10, 255, 255])
    elif color == "orange":
        return np.array([10, 100, 20]), np.array([25, 255, 255])
    elif color == "green":
        return np.array([40, 70, 70]), np.array([80, 255, 255])
    elif color == "yellow":
        return np.array([20, 100, 100]), np.array([30, 255, 255])
    else:
        return None, None


def find_largest_colored_object(frame, lower_color, upper_color):
    """
    Given a BGR frame and HSV bounds, return the bounding box (x, y, w, h)
    of the largest detected colored region. If none found, return None.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    return x, y, w, h


def draw_bounding_box(frame, bbox, color=(0, 255, 0), thickness=2):
    """
    Draw a rectangle on the frame given a bounding box (x, y, w, h).
    """
    x, y, w, h = bbox
    cv2.rectangle(frame, (x, y), (x + w, y + h), color, thickness)


def bbox_area(bbox) -> int:
    """
    Return the area (w * h) of the bounding box.
    """
    _, _, w, h = bbox
    return w * h


def bbox_center_x(bbox) -> float:
    """
    Return the x-coordinate of the center of the bounding box.
    """
    x, _, w, _ = bbox
    return x + w / 2.0


def servo_angle_from_center(frame_width: int, center_x: float) -> int:
    """
    Map the x position of the object in the frame to a servo angle [0, 180].
    This keeps the cone centered in the camera view.
    """
    if frame_width <= 0:
        return 90  # fallback to center

    normalized_value = int((center_x * 180) / frame_width)
    # clamp to [0, 180]
    normalized_value = max(0, min(180, normalized_value))
    return normalized_value
