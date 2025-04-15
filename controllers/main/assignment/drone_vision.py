import numpy as np
import time
import cv2

def purple_mask(frame):
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for purple color in HSV
    lower_purple = np.array([140, 60, 60])  # Shifted hue slightly higher
    upper_purple = np.array([165, 255, 255])  # Extended range toward pink


    # Create a mask for purple color
    mask = cv2.inRange(hsv, lower_purple, upper_purple)

    return mask

def detect_rectangle(frame):
    mask = purple_mask(frame)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize variables to store the largest contour and its bounding rectangle
    largest_contour = None
    largest_area = 0
    bounding_rect = None

    # Loop through each contour and find the largest one
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > largest_area:
            largest_area = area
            largest_contour = contour
            bounding_rect = cv2.boundingRect(contour)

    return bounding_rect

def purple_color_detected(frame, pixel_threshold=500):
    """
    Checks if the purple-pinkish color is present in the frame
    based on a pixel count threshold.
    
    Parameters:
    - frame: BGR image (np.ndarray)
    - pixel_threshold: Minimum number of mask pixels to count as detected
    
    Returns:
    - bool: True if color detected, False otherwise
    """
    mask = purple_mask(frame)
    pixel_count = cv2.countNonZero(mask)
    return pixel_count > pixel_threshold


def detect_parallelogram(frame):
    mask = purple_mask(frame)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize variables to store the largest contour and its approximated parallelogram
    largest_contour = None
    largest_area = 0
    parallelogram_points = None

    # Loop through each contour and find the largest one that approximates a parallelogram
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > largest_area:
            # Approximate the contour to a polygon
            epsilon = 0.05 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Check if the approximated polygon has 4 sides (parallelogram)
            if len(approx) == 4:
                largest_area = area
                largest_contour = contour
                parallelogram_points = approx

    return parallelogram_points


def draw_rectangle(frame, bounding_rect):
    if bounding_rect is not None:
        x, y, w, h = bounding_rect
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw a green rectangle
    return frame

def draw_parallelogram(frame, parallelogram_points):
    if parallelogram_points is not None:
        # Draw the parallelogram using the points
        cv2.polylines(frame, [parallelogram_points], isClosed=True, color=(255, 0, 0), thickness=2)  # Draw a blue parallelogram
    return frame