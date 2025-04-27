import numpy as np
import cv2
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import time
import exercises.ex3_motion_planner as mp

# The available ground truth state measurements can be accessed by calling sensor_data[item]. All values of "item" are provided as defined in main.py within the function read_sensors. 
# The "item" values that you may later retrieve for the hardware project are:
# "x_global": Global X position
# "y_global": Global Y position
# "z_global": Global Z position
# 'v_x": Global X velocity
# "v_y": Global Y velocity
# "v_z": Global Z velocity
# "ax_global": Global X acceleration
# "ay_global": Global Y acceleration
# "az_global": Global Z acceleration (With gravtiational acceleration subtracted)
# "roll": Roll angle (rad)
# "pitch": Pitch angle (rad)
# "yaw": Yaw angle (rad)
# "q_x": X Quaternion value
# "q_y": Y Quaternion value
# "q_z": Z Quaternion value
# "q_w": W Quaternion value

# A link to further information on how to access the sensor data on the Crazyflie hardware for the hardware practical can be found here: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/#stateestimate

camera_params = {
        'W': 300,               # Image width in pixels
        'H': 300,               # Image height in pixels
        'FOV': 1.5,             # Field of view in radians
        'camera_to_body_offset': [0.03, 0, 0.01]  # Camera position in body frame
    }

S_WIDTH = 300
S_HEIGHT = 300
control_command = [0, 0, 0, 0]  # Initialize control command
f_pixel = 161.01
takeoff = False
slope_threshold = 0.3
inside_door = False 
door_pos = [[0,0,0]]
turn_counter = 0
intial_pos = [0, 0, 0, 0]
offset_correct = [[0,0,0]]
path = [[0,0,0]]
stupid_path = False
grid_size = 0.1
bounds = [0, 9, 0, 9, 0, 4]
plan_path = False
setpoints = None
timepoints = None
reco_timer = 0

def get_command(sensor_data, camera_data, dt):

    global control_command, inside_door, door_pos, turn_counter, initial_pos, offset_correct, stupid_path, path, bounds, grid_size, takeoff, plan_path, setpoints, timepoints, reco_timer
    # NOTE: Displaying the camera image with cv2.imshow() will throw an error because GUI operations should be performed in the main thread.
    # If you want to display the camera image you can call it main.py.
    # Take off example
    if sensor_data['z_global'] < 1 and not takeoff:
        initial_pos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']]
        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.1, sensor_data['yaw']]
        return control_command

    # ---- YOUR CODE HERE ----
    takeoff = True
    # controller = pid.quadrotor_controller(exp_num=3)  # Or 2 or 3 depending on your setup

    if not plan_path:
        reco_timer += dt

    if path[0] == [0,0,0]:
        path[0] = initial_pos
    elif distance(sensor_data, path[-1]) > 0.8:
        path.append([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']])

    if distance(sensor_data, door_pos[-1]) > 0.3 and len(door_pos) <6 and door_pos[0] != [0,0,0]:
        if len(door_pos)+2 > len(offset_correct):
            offset_correct.append([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']])

    if door_pos is not None:
        if len(door_pos) == 5:
            if distance(sensor_data, door_pos[-1]) > 0.3:
                door_pos.append([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']])


    if door_pos is not None:
        if len(door_pos) == 6:
            if offset_correct[0] == [0,0,0] or distance_points(initial_pos, offset_correct[0]) < 0.5:
                offset_correct.pop(0)
            waypoints = [door_pos[0], offset_correct[0], offset_correct[1], door_pos[1], offset_correct[2], door_pos[2], offset_correct[3], door_pos[3], offset_correct[4], door_pos[4], door_pos[5]]
            waypoints = add_pre_door_waypoints_pp(waypoints, distance_before=0.5)
            waypoints.append([initial_pos[0], initial_pos[1], initial_pos[2]])
            waypoints += waypoints
            # waypoints.pop(-1)
            command = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']]
            if not plan_path:
                planner = MotionPlanner3D(waypoints, [], reco_timer)
                setpoints = planner.trajectory_setpoints
                timepoints = planner.time_setpoints
                # print("Time setpoints: ", time_points)
                # print("Trajectory setpoints: ", set_points)
                plan_path = True
            elif plan_path:
                command = trajectory_tracking(sensor_data, dt, timepoints, setpoints, tol=0.2)
            # waypoints_4D = [wp if len(wp) == 4 else list(wp) + [0.0] for wp in waypoints]
            # time_step = 0.9  # or whatever you prefer
            # timepoints = [i * time_step for i in range(len(waypoints_4D))]
            # command = trajectory_tracking(sensor_data, dt, timepoints, np.array(waypoints_4D), tol=0.2)
            # command = path_planning(sensor_data, dt, waypoints_4D, tol=0.1)
            return command
    
    if stupid_path:
        waypoints_4D = [wp if len(wp) == 4 else list(wp) + [0.0] for wp in path]
        for wp in waypoints_4D:
                if wp[3] < 0:
                    wp[3] = 0.0
        command = path_planning(sensor_data, dt, waypoints_4D, tol=0.2)
        return command


    # Define the drawing frame
    drawing_frame = camera_data.copy()

    parallelogram = detect_parallelogram(camera_data)

    if turn_counter > 300 :
        # Reset the flags if no parallelogram is detected after 20 turns
        if len(door_pos) < 5 :
            stupid_path = True
        else: return initial_pos

    if not(purple_color_detected(drawing_frame, pixel_threshold= 200)):
        # No Purle detected, reset the flags
        turn_counter += 1
        return rotate(sensor_data, 15)  # Rotate to search for the parallelogram  
    elif purple_color_detected(drawing_frame, pixel_threshold= 40000) and not inside_door:
        inside_door = True
        if distance(sensor_data, door_pos[-1]) > 0.9:
            door_pos.append([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']])
            return move_towards_camera_point(sensor_data, [S_WIDTH / 2, S_HEIGHT / 2], camera_data, speed=2)
        else :   
            door_pos[-1] = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']]
    elif purple_color_detected(drawing_frame, pixel_threshold= 25000) and not inside_door:
        inside_door = True
        if door_pos[0] == [0,0,0]:
            door_pos[0] = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']]
        elif distance(sensor_data, door_pos[-1]) > 0.9:
            door_pos.append([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']]) 
            return move_towards_camera_point(sensor_data, [S_WIDTH / 2, S_HEIGHT / 2], camera_data, speed=2)  # Move towards the center of the camera frame

    if parallelogram is None and inside_door:
        # No parallelogram detected, reset the flags
        inside_door = False
        return rotate(sensor_data, 15)

    if parallelogram is None and purple_color_detected(drawing_frame, pixel_threshold= 200):
        # No parallelogram detected, reset the flags
        return move_right(sensor_data, distance= 2)  # Move right to search for the parallelogram
    
    inside_door = False


    if parallelogram is not None:
        turn_counter = 0
        # Calculate the centroid of the parallelogram in pixel coordinates
        points = parallelogram.reshape(-1, 2)
        centroid = np.mean(points, axis=0)
        cv2.circle(drawing_frame, (int(centroid[0]), int(centroid[1])), 5, (255, 0, 0), -1) 

        if is_center_aligned(centroid, camera_data):
            return move_towards_camera_point(sensor_data, centroid, camera_data, 0.7)

        if centroid is not None:
            # comm = align_center(centroid, sensor_data, camera_data)
            # if abs(sensor_data['z_global'] - comm[2]) > 0.3:
            #     return move_towards_camera_point(sensor_data, centroid, camera_data, 0.3)
            return align_center(centroid, sensor_data, camera_data)           

    current_pose = ([sensor_data["x_global"], sensor_data["y_global"], sensor_data["z_global"]], 
                    [sensor_data["q_x"], sensor_data["q_y"], sensor_data["q_z"], sensor_data["q_w"]])


    return [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']]  # No movement command

def align_center(center, sensor_data, camera_data):
    err = np.array([center[0] - camera_data.shape[1] / 2, center[1] - camera_data.shape[0] / 2])
    Kp_yaw = 0.5
    yaw_err = err[0] / f_pixel 
    yaw = sensor_data['yaw'] - Kp_yaw * yaw_err
    yaw = np.clip(yaw, -np.pi, np.pi)
    Kp_z = 0.1
    z_err = err[1]
    z = sensor_data['z_global'] - Kp_z * z_err
    z = np.clip(z, 0.2, 2.0)
    control_command = [sensor_data['x_global'], sensor_data['y_global'], z, yaw]
    return control_command


def is_center_aligned(center, camera_data, threshold_px=100):
    err_x = center[0] - camera_data.shape[1] / 2
    err_y = center[1] - camera_data.shape[0] / 2
    return np.abs(err_x) < threshold_px and np.abs(err_y) < threshold_px



def move_right(sensor_data, distance = 0.5):

    x = sensor_data['x_global']
    y = sensor_data['y_global']
    yaw = sensor_data['yaw']

    tar_x = x + distance * np.sin(yaw)
    tar_y = y - distance * np.cos(yaw)

    control_command = [tar_x, tar_y, sensor_data['z_global'], sensor_data['yaw']]

    return control_command


def move_left(sensor_data, distance = 0.5):

    x = sensor_data['x_global']
    y = sensor_data['y_global']
    yaw = sensor_data['yaw']

    tar_x = x - distance * np.sin(yaw)
    tar_y = y + distance * np.cos(yaw)

    control_command = [tar_x, tar_y, sensor_data['z_global'], sensor_data['yaw']]

    return control_command

def move_towards_camera_point(sensor_data, camera_point, camera_data, speed=0.5):
    """
    Moves the drone forward towards the center point of the camera.

    Args:
        sensor_data: Dictionary containing the current sensor data.
        camera_point: Tuple containing the target [x_pixel, y_pixel] coordinates in the camera frame.
        camera_data: The current camera frame data.
        speed: Speed factor for movement (default is 0.5).

    Returns:
        control_command: List containing the control command [x, y, z, yaw].
    """
    # Proportional control for yaw and z
    Kp_yaw = 0.6
    Kp_z = 0.01

    # Calculate the error in pixel coordinates
    err = np.array([camera_point[0] - camera_data.shape[1] / 2, camera_point[1] - camera_data.shape[0] / 2])

    # Adjust yaw to align with the center
    yaw_err = err[0] / f_pixel
    yaw = sensor_data['yaw'] - Kp_yaw * yaw_err
    yaw = np.clip(yaw, -np.pi, np.pi)

    # Adjust z to maintain height
    z_err = err[1]
    z = sensor_data['z_global'] - Kp_z * z_err
    z = np.clip(z, 0.2, 2.0)

    # Move forward in the direction the drone is facing
    x = sensor_data['x_global'] + speed * np.cos(sensor_data['yaw'])
    y = sensor_data['y_global'] + speed * np.sin(sensor_data['yaw'])

    # Generate control command
    control_command = [x, y, z, yaw]

    return control_command

def rotate(sensor_data, angle):
    """
    Rotates the drone by a specified angle.

    Args:
        sensor_data: Dictionary containing the current sensor data.
        angle: Angle in degrees to rotate.

    Returns:
        control_command: List containing the control command [x, y, z, yaw].
    """

    alpha = np.deg2rad(angle) 

    x = sensor_data['x_global']
    y = sensor_data['y_global']
    z = sensor_data['z_global']
    yaw = sensor_data['yaw'] + alpha

    # Ensure yaw is within [-pi, pi]
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi

    control_command = [x, y, z, yaw]

    return control_command

def move_right_and_rotate(sensor_data, distance=0.5, angle=45):
    """
    Moves the drone right and rotates it by a specified angle.

    Args:
        sensor_data: Dictionary containing the current sensor data.
        distance: Distance to move right (default is 0.5).
        angle: Angle in degrees to rotate (default is 45).

    Returns:
        control_command: List containing the control command [x, y, z, yaw].
    """
    # Move right
    x = sensor_data['x_global'] + distance * np.sin(sensor_data['yaw'])
    y = sensor_data['y_global'] - distance * np.cos(sensor_data['yaw'])
    z = sensor_data['z_global']
    
    # Rotate
    alpha = np.deg2rad(angle)
    yaw = sensor_data['yaw'] + alpha

    # Ensure yaw is within [-pi, pi]
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi

    control_command = [x, y, z, yaw]

    return control_command

def paral_slope(parallelogram_points):
    """
    Calculates the slope of the upper side of a given parallelogram.

    Args:
        parallelogram_points: A numpy array of shape (4, 1, 2) containing the coordinates of the parallelogram's vertices.

    Returns:
        slope: The slope of the upper side of the parallelogram.
    """
    if parallelogram_points is None or len(parallelogram_points) != 4:
        raise ValueError("Invalid parallelogram. It must have exactly 4 vertices.")

    # Flatten the points to (4, 2) and sort the points by their y-coordinates (ascending)
    parallelogram_points = parallelogram_points.reshape(4, 2)
    sorted_points = parallelogram_points[np.argsort(parallelogram_points[:, 1])]

    # The upper side is formed by the two points with the highest y-coordinates
    upper_points = sorted_points[2:]

    # Calculate the slope of the line connecting the two upper points
    x1, y1 = upper_points[0]
    x2, y2 = upper_points[1]

    if x2 - x1 == 0:
        return float('inf')  # Vertical line

    slope = (y2 - y1) / (x2 - x1)
    return slope

def distance(sensor_data, previous_sensor_data):
    """
    Calculate the distance between the current and previous sensor data.

    Args:
        sensor_data: Dictionary containing the current sensor data.
        previous_sensor_data: Dictionary containing the previous sensor data.

    Returns:
        distance: Distance between the two sensor data points.
    """
    x_current = sensor_data['x_global']
    y_current = sensor_data['y_global']
    z_current = sensor_data['z_global']

    x_previous = previous_sensor_data[0]
    y_previous = previous_sensor_data[1]
    z_previous = previous_sensor_data[2]

    distance = np.sqrt((x_current - x_previous) ** 2 + (y_current - y_previous) ** 2 + (z_current - z_previous) ** 2)
    
    return distance

def distance_points(point1, point2):
    """
    Calculate the distance between two points in 3D space.

    Args:
        point1: list containing the coordinates of the first point [x1, y1, z1].
        point2: list containing the coordinates of the second point [x2, y2, z2].
    
    Returns:
        distance: Distance between the two points.
    """
    x1, y1, z1 = point1[0], point1[1], point1[2]
    x2, y2, z2 = point2[0], point2[1], point2[2]

    distance = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
    
    return distance

def purple_mask(frame):
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for purple color in HSV
    lower_purple = np.array([140, 60, 60])  # Shifted hue slightly higher
    upper_purple = np.array([165, 255, 255])  # Extended range toward pink


    # Create a mask for purple color
    mask = cv2.inRange(hsv, lower_purple, upper_purple)

    return mask

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

    rightmost_center_x = -np.inf
    parallelogram_points = None

    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.05 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the approximated polygon has 4 sides (parallelogram)
        if len(approx) == 4:
            # Compute the center X of the polygon
            approx_flat = approx.reshape(4, 2)
            center_x = np.mean(approx_flat[:, 0])  # Mean of x-coordinates

            if center_x > rightmost_center_x:
                rightmost_center_x = center_x
                parallelogram_points = approx

    return parallelogram_points

def detect_parallelograms(frame):
    mask = purple_mask(frame)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    parallelograms = []

    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.05 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the approximated polygon has 4 sides (potential parallelogram)
        if len(approx) == 4:
            parallelograms.append(approx)

    return parallelograms

def get_parallelogram_centers(frame):
    """
    Detects parallelograms in the frame and returns their pixel-space centers.

    Args:
        frame: Image in which to detect parallelograms.

    Returns:
        List of (x, y) centroids of detected parallelograms in pixel coordinates.
    """
    parallelograms = detect_parallelograms(frame)
    centers = []

    for points in parallelograms:
        # Reshape to (4, 2) and compute centroid
        pts = points.reshape(-1, 2)
        centroid = np.mean(pts, axis=0)
        centers.append(tuple(centroid))

    return centers

on_ground = True
height_desired = 0.5
timer = None
startpos = None
timer_done = None
index_current_setpoint = 0

def path_planning(sensor_data, dt, setpoints, tol):
    global on_ground, height_desired, index_current_setpoint, timer, timer_done, startpos

    # Take off
    if startpos is None:
        startpos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']]
        # print(startpos)    
    if on_ground and sensor_data['z_global'] < 0.49:
        #current_setpoint = [0.97,0.84,height_desired,0]
        current_setpoint = [startpos[0], startpos[1], height_desired, 0.0]
        # print(current_setpoint)
        return current_setpoint
    else:
        on_ground = False

    # Start timer
    if (index_current_setpoint == 1) & (timer is None):
        timer = 0
        print("Time recording started")
    if timer is not None:
        timer += dt
    # Hover at the final setpoint
    if index_current_setpoint == len(setpoints):
        control_command = [0.0, 0.0, height_desired, 0.0] #[startpos[0], startpos[1], startpos[2]-0.05, 0.0]

        if timer_done is None:
            timer_done = True
            print("Path planning took " + str(np.round(timer,1)) + " [s]")
        return control_command

    # Get the goal position and drone position
    current_setpoint = setpoints[index_current_setpoint]
    x_drone, y_drone, z_drone, yaw_drone = sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']
    distance_drone_to_goal = np.linalg.norm([current_setpoint[0] - x_drone, current_setpoint[1] - y_drone, current_setpoint[2] - z_drone, current_setpoint[3] - yaw_drone%(2*np.pi)])

    # When the drone reaches the goal setpoint, e.g., distance < 0.1m
    if distance_drone_to_goal < tol:
        # Select the next setpoint as the goal position
        index_current_setpoint += 1
        # Hover at the final setpoint
        if index_current_setpoint == len(setpoints):
            index_current_setpoint = 0
            current_setpoint = setpoints[0]
            return current_setpoint

    return current_setpoint

def trajectory_tracking(sensor_data, dt, timepoints, setpoints, tol, repeat = True):
    global on_ground, index_current_setpoint, timer, timer_done

    start_point = setpoints[0]
    end_point = setpoints[-1]

    # Take off 
    if on_ground and sensor_data['z_global'] < 0.2:
        current_setpoint = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'] + 0.5, sensor_data['yaw']]
        return current_setpoint
    else:
        on_ground = False
        if timer is None:
            # Begin timer and start trajectory
            timer = 0
            print("Trajectory tracking started")
            index_current_setpoint = 1
        else:
            timer += dt

    # Determine the current setpoint based on the time
    if not on_ground and timer is not None:
        if index_current_setpoint < len(timepoints) - 1:
            # Update new setpoint
            current_setpoint = setpoints[index_current_setpoint,:]
            if timer >= timepoints[index_current_setpoint] or np.linalg.norm([sensor_data['x_global'] - current_setpoint[0], sensor_data['y_global'] - current_setpoint[1], sensor_data['z_global'] - current_setpoint[2]]) < tol:

                index_current_setpoint += 1
            current_setpoint = setpoints[index_current_setpoint,:]
        else:
            # Hover at the final setpoint
            current_setpoint = end_point
            if timer_done is None and np.linalg.norm([sensor_data['x_global'] - end_point[0], sensor_data['y_global'] - end_point[1], sensor_data['z_global'] - end_point[2]]) < tol:
                timer_done = True
                print("Trajectory took " + str(np.round(timer,1)) + " [s]")
                if repeat:
                    timer_done = None
                    timer = None
                
    return current_setpoint

def add_pre_door_waypoints(waypoints, distance_before=0.3):
    """
    Adds a point before each door following the vector door -> offset.
    
    Args:
        waypoints: List of waypoints [door0, offset0, offset1, door1, offset2, door2, ...]
        distance_before: Distance before each door to add the new point.
        
    Returns:
        new_waypoints: New list with pre-door points added.
    """
    new_waypoints = []
    i = 0
    while i < len(waypoints) - 1:
        door = waypoints[i]
        offset = waypoints[i+1]
        
        # Compute vector from door to offset
        direction = np.array(offset[:3]) - np.array(door[:3])
        direction_norm = np.linalg.norm(direction)
        
        if direction_norm != 0:
            direction_unit = direction / direction_norm
        else:
            direction_unit = np.zeros_like(direction)
        
        # Create the pre-door point
        pre_door = np.array(door[:3]) - distance_before * direction_unit
        
        # Add pre-door point, door, and offset to the new list
        new_waypoints.append(list(pre_door) + [0.0])  # Add yaw 0.0 if needed
        new_waypoints.append(door if len(door) == 4 else list(door) + [0.0])
        new_waypoints.append(offset if len(offset) == 4 else list(offset) + [0.0])
        
        i += 2  # Move to next door-offset pair

    # Add remaining points if any
    if i < len(waypoints):
        for wp in waypoints[i:]:
            new_waypoints.append(wp if len(wp) == 4 else list(wp) + [0.0])
    
    return new_waypoints

def add_pre_door_waypoints_pp(waypoints, distance_before=0.3):
    """
    Adds a point before each door following the vector door -> offset, without using numpy.
    No yaw is added. 
    After building, swaps the second, third, and fourth elements.
    
    Args:
        waypoints: List of waypoints [door0, offset0, offset1, door1, offset2, door2, ...]
        distance_before: Distance before each door to add the new point.

    Returns:
        new_waypoints: New list with pre-door points added and elements swapped.
    """
    new_waypoints = []
    i = 0
    while i < len(waypoints) - 1:
        door = waypoints[i]
        offset = waypoints[i+1]
        
        # Compute direction vector (offset - door)
        dir_x = offset[0] - door[0]
        dir_y = offset[1] - door[1]
        dir_z = offset[2] - door[2]

        # Compute the norm (length) of the direction vector
        norm = (dir_x**2 + dir_y**2 + dir_z**2) ** 0.5
        
        if norm != 0:
            # Normalize direction vector
            dir_x /= norm
            dir_y /= norm
            dir_z /= norm
        else:
            dir_x, dir_y, dir_z = 0.0, 0.0, 0.0
        
        # Compute the pre-door point (pure 3D)
        pre_door = [
            door[0] - distance_before * dir_x,
            door[1] - distance_before * dir_y,
            door[2] - distance_before * dir_z
        ]

        # Add pre-door, door, and offset (without yaw)
        new_waypoints.append(pre_door)
        new_waypoints.append(door[:3])    # Only x, y, z
        new_waypoints.append(offset[:3])  # Only x, y, z
        
        i += 2

    # Add any remaining points
    if i < len(waypoints):
        for wp in waypoints[i:]:
            new_waypoints.append(wp[:3])  # Only x, y, z

    # Swap second, third, and fourth elements
    if len(new_waypoints) >= 4:
        new_waypoints[1], new_waypoints[2], new_waypoints[3] = new_waypoints[3], new_waypoints[1], new_waypoints[2]

    return new_waypoints


def add_pre_door_waypoints_nonp(waypoints, distance_before=0.3):
    """
    Adds a point before each door following the vector door -> offset, without using numpy.
    After building, swaps the first and second elements of the new list.
    
    Args:
        waypoints: List of waypoints [door0, offset0, offset1, door1, offset2, door2, ...]
        distance_before: Distance before each door to add the new point.
        
    Returns:
        new_waypoints: New list with pre-door points added and first two elements swapped.
    """
    new_waypoints = []
    i = 0
    while i < len(waypoints) - 1:
        door = waypoints[i]
        offset = waypoints[i+1]
        
        # Compute direction vector (offset - door)
        dir_x = offset[0] - door[0]
        dir_y = offset[1] - door[1]
        dir_z = offset[2] - door[2]

        # Compute the norm (length) of the direction vector
        norm = (dir_x**2 + dir_y**2 + dir_z**2) ** 0.5
        
        if norm != 0:
            # Normalize direction vector
            dir_x /= norm
            dir_y /= norm
            dir_z /= norm
        else:
            dir_x, dir_y, dir_z = 0.0, 0.0, 0.0
        
        # Compute the pre-door point
        pre_door_x = door[0] - distance_before * dir_x
        pre_door_y = door[1] - distance_before * dir_y
        pre_door_z = door[2] - distance_before * dir_z
        
        pre_door = [pre_door_x, pre_door_y, pre_door_z, 0.0]  # Add yaw 0.0

        # Add to new waypoints
        new_waypoints.append(pre_door)
        new_waypoints.append(door if len(door) == 4 else door + [0.0])
        new_waypoints.append(offset if len(offset) == 4 else offset + [0.0])
        
        i += 2

    # Add any remaining points
    if i < len(waypoints):
        for wp in waypoints[i:]:
            new_waypoints.append(wp if len(wp) == 4 else wp + [0.0])

    # Swap first and second elements
    if len(new_waypoints) >= 2:
        new_waypoints[1], new_waypoints[2], new_waypoints[3] = new_waypoints[3], new_waypoints[1], new_waypoints[2]

    return new_waypoints

def add_pre_door_waypoints_with_midpoint(waypoints, distance_before=0.3):
    """
    Adds a point before each door following the vector door -> offset, without using numpy.
    Then adds a midpoint between door and offset.
    After that, moves the first element to the third position.
    
    Args:
        waypoints: List of waypoints [door0, offset0, offset1, door1, offset2, door2, ...]
        distance_before: Distance before each door to add the new point.
        
    Returns:
        new_waypoints: New list with pre-door, door, midpoint, offset points added and first element shifted.
    """
    new_waypoints = []
    i = 0
    while i < len(waypoints) - 1:
        door = waypoints[i]
        offset = waypoints[i+1]
        
        # Compute direction vector (offset - door)
        dir_x = offset[0] - door[0]
        dir_y = offset[1] - door[1]
        dir_z = offset[2] - door[2]

        # Compute the norm (length) of the direction vector
        norm = (dir_x**2 + dir_y**2 + dir_z**2) ** 0.5
        
        if norm != 0:
            # Normalize direction vector
            dir_x /= norm
            dir_y /= norm
            dir_z /= norm
        else:
            dir_x, dir_y, dir_z = 0.0, 0.0, 0.0
        
        # Compute the pre-door point
        pre_door_x = door[0] - distance_before * dir_x
        pre_door_y = door[1] - distance_before * dir_y
        pre_door_z = door[2] - distance_before * dir_z
        
        pre_door = [pre_door_x, pre_door_y, pre_door_z, 0.0]  # Add yaw 0.0

        # Compute midpoint between door and offset
        mid_x = (door[0] + offset[0]) / 2
        mid_y = (door[1] + offset[1]) / 2
        mid_z = (door[2] + offset[2]) / 2
        midpoint = [mid_x, mid_y, mid_z, 0.0]  # Midpoint with yaw 0.0

        # Add in order: pre-door, door, midpoint, offset
        new_waypoints.append(pre_door)
        new_waypoints.append(door if len(door) == 4 else door + [0.0])
        new_waypoints.append(midpoint)
        new_waypoints.append(offset if len(offset) == 4 else offset + [0.0])
        
        i += 2  # Move to next door-offset pair

    # Add any remaining points (rare, for odd cases)
    if i < len(waypoints):
        for wp in waypoints[i:]:
            new_waypoints.append(wp if len(wp) == 4 else wp + [0.0])

    if len(new_waypoints) >= 5:
        new_waypoints[0], new_waypoints[1], new_waypoints[2], new_waypoints[3], new_waypoints[4] = new_waypoints[0], new_waypoints[4], new_waypoints[1], new_waypoints[2], new_waypoints[3]

    new_waypoints[-1][0] = new_waypoints[-1][0] - 1
    
    return new_waypoints

class MotionPlanner3D():
    
    def __init__(self, waypoints, obstacles, reco_timer):
        # Inputs:
        # - waypoints: The sequence of input path waypoints provided by the path-planner, including the start and final goal position: Vector of m waypoints, consisting of an array of 3 elements corresponding to the x, y, z position of the drone 
        # - obstacles: 2D array with obstacle locations and obstacle widths [x, y, z, dx, dy, dz]*n_obs   
        ## DO NOT MODIFY --------------------------------------------------------------------------------------- ##
        self.path = waypoints

        self.trajectory_setpoints = None

        self.init_params(self.path, reco_timer)

        self.run_planner(obstacles, self.path)

        # ---------------------------------------------------------------------------------------------------- ##

    def run_planner(self, obs, path_waypoints):
        # Run the subsequent functions to compute the polynomial coefficients and extract and visualize the trajectory setpoints
        ## DO NOT MODIFY --------------------------------------------------------------------------------------- ##
    
        poly_coeffs = self.compute_poly_coefficients(path_waypoints)
        self.trajectory_setpoints, self.time_setpoints = self.poly_setpoint_extraction(poly_coeffs, obs, path_waypoints)

        ## ---------------------------------------------------------------------------------------------------- ##

    def init_params(self, path_waypoints, reco_timer):

        # Inputs:
        # - path_waypoints: The sequence of input path waypoints provided by the path-planner, including the start and final goal position: Vector of m waypoints, consisting of a tuple with three reference positions each as provided by AStar

        # TUNE THE FOLLOWING PARAMETERS (PART 2) ----------------------------------------------------------------- ##
        self.disc_steps = 18    # Integer number steps to divide every path segment into to provide the reference positions for PID control # IDEAL: Between 10 and 20
        self.vel_lim = 7.0        # Velocity limit of the drone (m/s)
        self.acc_lim = 50.0       # Acceleration limit of the drone (m/s²)
        t_f = 36  #2*reco_timer - 7 #               # Final time at the end of the path (s)

        # Determine the number of segments of the path
        self.times = np.linspace(0, t_f, len(path_waypoints)) # The time vector at each path waypoint to traverse (Vector of size m) (must be 0 at start)

    def compute_poly_matrix(self, t):
        # Inputs:
        # - t: The time of evaluation of the A matrix (t=0 at the start of a path segment, else t >= 0) [Scalar]
        # Outputs: 
        # - The constraint matrix "A_m(t)" [5 x 6]
        # The "A_m" matrix is used to represent the system of equations [x, \dot{x}, \ddot{x}, \dddot{x}, \ddddot{x}]^T  = A_m(t) * poly_coeffs (where poly_coeffs = [c_0, c_1, c_2, c_3, c_4, c_5]^T and represents the unknown polynomial coefficients for one segment)
        A_m = np.zeros((5,6))
        
        # TASK: Fill in the constraint factor matrix values where each row corresponds to the positions, velocities, accelerations, snap and jerk here
        # SOLUTION ---------------------------------------------------------------------------------- ## 
        
        A_m = np.array([
            [t**5, t**4, t**3, t**2, t, 1], #pos
            [5*(t**4), 4*(t**3), 3*(t**2), 2*t, 1, 0], #vel
            [20*(t**3), 12*(t**2), 6*t, 2, 0, 0], #acc  
            [60*(t**2), 24*t, 6, 0, 0, 0], #jerk
            [120*t, 24, 0, 0, 0, 0] #snap
        ])

        return A_m

    def compute_poly_coefficients(self, path_waypoints):
        
        # Computes a minimum jerk trajectory given time and position waypoints.
        # Inputs:
        # - path_waypoints: The sequence of input path waypoints provided by the path-planner, including the start and final goal position: Vector of m waypoints, consisting of a tuple with three reference positions each as provided by AStar
        # Outputs:
        # - poly_coeffs: The polynomial coefficients for each segment of the path [6(m-1) x 3]

        # Use the following variables and the class function self.compute_poly_matrix(t) to solve for the polynomial coefficients
        
        seg_times = np.diff(self.times) #The time taken to complete each path segment
        m = len(path_waypoints) #Number of path waypoints (including start and end)
        poly_coeffs = np.zeros((6*(m-1),3))

        # YOUR SOLUTION HERE ---------------------------------------------------------------------------------- ## 

        # 1. Fill the entries of the constraint matrix A and equality vector b for x,y and z dimensions in the system A * poly_coeffs = b. Consider the constraints according to the lecture: We should have a total of 6*(m-1) constraints for each dimension.
        # 2. Solve for poly_coeffs given the defined system

        for dim in range(3):  # Compute for x, y, and z separately
            A = np.zeros((6*(m-1), 6*(m-1)))
            b = np.zeros(6*(m-1))
            pos = np.array([p[dim] for p in path_waypoints])
            A_0 = self.compute_poly_matrix(0) # A_0 gives the constraint factor matrix A_m for any segment at t=0, this is valid for the starting conditions at every path segment

            # SOLUTION
            row = 0
            for i in range(m-1):
                pos_0 = pos[i] #Starting position of the segment
                pos_f = pos[i+1] #Final position of the segment
                # The prescribed zero velocity (v) and acceleration (a) values at the start and goal position of the entire path
                v_0, a_0 = 0, 0
                v_f, a_f = 0, 0
                A_f = self.compute_poly_matrix(seg_times[i]) # A_f gives the constraint factor matrix A_m for a segment i at its relative end time t=seg_times[i]
                if i == 0: # First path segment
                #     # 1. Implement the initial constraints here for the first segment using A_0
                #     # 2. Implement the final position and the continuity constraints for velocity, acceleration, snap and jerk at the end of the first segment here using A_0 and A_f (check hints in the exercise description)
                    A[row, i*6:(i+1)*6] = A_0[0] #Initial position constraint
                    b[row] = pos_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[0] #Final position constraint
                    b[row] = pos_f
                    row += 1
                    A[row, i*6:(i+1)*6] = A_0[1] #Initial velocity constraint
                    b[row] = v_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_0[2] #Initial acceleration constraint
                    b[row] = a_0
                    row += 1
                    #Continuity of velocity, acceleration, jerk, snap
                    A[row:row+4, i*6:(i+1)*6] = A_f[1:]
                    A[row:row+4, (i+1)*6:(i+2)*6] = -A_0[1:]
                    b[row:row+4] = np.zeros(4)
                    row += 4
                elif i < m-2: # Intermediate path segments
                #     # 1. Similarly, implement the initial and final position constraints here for each intermediate path segment
                #     # 2. Similarly, implement the end of the continuity constraints for velocity, acceleration, snap and jerk at the end of each intermediate segment here using A_0 and A_f
                    A[row, i*6:(i+1)*6] = A_0[0] #Initial position constraint
                    b[row] = pos_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[0] #Final position constraint
                    b[row] = pos_f
                    row += 1
                    #Continuity of velocity, acceleration, jerk and snap
                    A[row:row+4, i*6:(i+1)*6] = A_f[1:]
                    A[row:row+4, (i+1)*6:(i+2)*6] = -A_0[1:]
                    b[row:row+4] = np.zeros(4)
                    row += 4
                elif i == m-2: #Final path segment
                #     # 1. Implement the initial and final position, velocity and accelerations constraints here for the final path segment using A_0 and A_f
                    A[row, i*6:(i+1)*6] = A_0[0] #Initial position constraint
                    b[row] = pos_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[0] #Final position constraint
                    b[row] = pos_f
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[1] #Final velocity constraint
                    b[row] = v_f
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[2] #Final acceleration constraint
                    b[row] = a_f
                    row += 1
            # Solve for the polynomial coefficients for the dimension dim

            poly_coeffs[:,dim] = np.linalg.solve(A, b)   

        return poly_coeffs

    def poly_setpoint_extraction(self, poly_coeffs, obs, path_waypoints):

        # DO NOT MODIFY --------------------------------------------------------------------------------------- ##

        # Uses the class features: self.disc_steps, self.times, self.poly_coeffs, self.vel_lim, self.acc_lim
        x_vals, y_vals, z_vals = np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1))
        v_x_vals, v_y_vals, v_z_vals = np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1))
        a_x_vals, a_y_vals, a_z_vals = np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1))

        # Define the time reference in self.disc_steps number of segements
        time_setpoints = np.linspace(self.times[0], self.times[-1], self.disc_steps*len(self.times))  # Fine time intervals

        # Extract the x,y and z direction polynomial coefficient vectors
        coeff_x = poly_coeffs[:,0]
        coeff_y = poly_coeffs[:,1]
        coeff_z = poly_coeffs[:,2]

        for i,t in enumerate(time_setpoints):
            seg_idx = min(max(np.searchsorted(self.times, t)-1,0), len(coeff_x) - 1)
            # Determine the x,y and z position reference points at every refernce time
            x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_x[seg_idx*6:(seg_idx+1)*6])
            y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_y[seg_idx*6:(seg_idx+1)*6])
            z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_z[seg_idx*6:(seg_idx+1)*6])
            # Determine the x,y and z velocities at every reference time
            v_x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_x[seg_idx*6:(seg_idx+1)*6])
            v_y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_y[seg_idx*6:(seg_idx+1)*6])
            v_z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_z[seg_idx*6:(seg_idx+1)*6])
            # Determine the x,y and z accelerations at every reference time
            a_x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_x[seg_idx*6:(seg_idx+1)*6])
            a_y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_y[seg_idx*6:(seg_idx+1)*6])
            a_z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_z[seg_idx*6:(seg_idx+1)*6])

        yaw_vals = np.zeros((self.disc_steps*len(self.times),1))
        trajectory_setpoints = np.hstack((x_vals, y_vals, z_vals, yaw_vals))
            
        # Find the maximum absolute velocity during the segment
        vel_max = np.max(np.sqrt(v_x_vals**2 + v_y_vals**2 + v_z_vals**2))
        vel_mean = np.mean(np.sqrt(v_x_vals**2 + v_y_vals**2 + v_z_vals**2))
        acc_max = np.max(np.sqrt(a_x_vals**2 + a_y_vals**2 + a_z_vals**2))
        acc_mean = np.mean(np.sqrt(a_x_vals**2 + a_y_vals**2 + a_z_vals**2))

        print("Maximum flight speed: " + str(vel_max))
        print("Average flight speed: " + str(vel_mean))
        print("Average flight acceleration: " + str(acc_mean))
        print("Maximum flight acceleration: " + str(acc_max))
        
        # Check that it is less than an upper limit velocity v_lim
        assert vel_max <= self.vel_lim, "The drone velocity exceeds the limit velocity : " + str(vel_max) + " m/s"
        assert acc_max <= self.acc_lim, "The drone acceleration exceeds the limit acceleration : " + str(acc_max) + " m/s²"

        # ---------------------------------------------------------------------------------------------------- ##

        return trajectory_setpoints, time_setpoints
