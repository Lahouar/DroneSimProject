import numpy as np
import time
import cv2
import sys
import os
# import exercises.ex3_motion_planner as mp
# import exercises.ex1_pid_control as pid
# import exercises.ex0_rotations as rot

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
frames = []
poses = []
nb_frames = 2
control_command = [0, 0, 0, 0]  # Initialize control command
f_pixel = 161.01
right_scan = False
left_scan = False
center_aligned = False
go_forward = False
takeoff = False
slope_threshold = 0.3
inside_door = False 
door_pos = [[0,0,0]]
turn_counter = 0
intial_pos = [0, 0, 0, 0]
grid_size = 0.25
bounds = (0, 5, 0, 3, 0, 1.5)
step = 0
offset_correct = [[0,0,0]]
fast_laps = False

def get_command(sensor_data, camera_data, dt):

    global frames, poses, control_command, right_scan, left_scan, center_aligned, go_forward, inside_door, door_pos, turn_counter, initial_pos, grid_size, bounds, step, offset_correct, fast_laps
    # NOTE: Displaying the camera image with cv2.imshow() will throw an error because GUI operations should be performed in the main thread.
    # If you want to display the camera image you can call it main.py.
    global takeoff
    # Take off example
    if sensor_data['z_global'] < 1 and not takeoff:
        initial_pos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']]
        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.1, sensor_data['yaw']]
        return control_command

    # ---- YOUR CODE HERE ----
    takeoff = True
    # controller = pid.quadrotor_controller(exp_num=3)  # Or 2 or 3 depending on your setup

    if distance(sensor_data, door_pos[-1]) > 0.8 and len(door_pos) <6 and door_pos[0] != [0,0,0]:
        if len(door_pos)+2 > len(offset_correct):
            offset_correct.append([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']])

    # Position (x, y, z)
    position = [sensor_data["x_global"], sensor_data["y_global"], sensor_data["z_global"]]

    euler_angles = [sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']]
        
    # Quaternion (q_x, q_y, q_z, q_w)
    quaternion = [sensor_data["q_x"], sensor_data["q_y"], sensor_data["q_z"], sensor_data["q_w"]]

    # Always store the last 3 frames and their corresponding poses
    if len(frames) == nb_frames:
        frames = []
        poses = []

    if door_pos is not None:
        if len(door_pos) == 5:
            if distance(sensor_data, door_pos[-1]) > 0.8:
                door_pos.append([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']])

 
    if door_pos is not None:
        if len(door_pos) == 6:
            if offset_correct[0] == [0,0,0] or distance_points(initial_pos, offset_correct[0]) < 0.5:
                offset_correct.pop(0)
            waypoints = [door_pos[0], offset_correct[0], offset_correct[1], door_pos[1], offset_correct[2], door_pos[2], offset_correct[3], door_pos[3], offset_correct[4], door_pos[4], door_pos[5]]
            fast_laps = True
            waypoints_4D = [wp if len(wp) == 4 else list(wp) + [0.0] for wp in waypoints]
            command = path_planning(sensor_data, dt, waypoints_4D, tol=0.05)
            return command
            print("Waypoints: ", waypoints)
            print("Door positions: ", door_pos)
            print("Offset positions: ", offset_correct)
            # door_pos_tuples = [tuple(pos) for pos in door_pos]
            # print("Door positions tuple: ", door_pos_tuples)
            # planner = mp.MotionPlanner3D(door_pos_tuples, [[0,0,0,0,0,0]], bounds, grid_size, (0, 0, 0))
            # trajectory = planner.trajectory_setpoints
            # times = planner.time_setpoints

            # if step == 0:
            #     if distance(sensor_data, initial_pos) < 0.3:
            #         step = step + 1
            #     return initial_pos
            
            # setpoint = [waypoints[step-1][0], waypoints[step-1][1], waypoints[step-1][2], sensor_data['yaw']]
            # # pwm = controller.setpoint_to_pwm(dt, setpoint, sensor_data)
            # if distance(sensor_data, waypoints[step-1]) < 0.2:
            #     step = step + 1
            #     if step == len(waypoints)+1:
            #         step = 0
            # print("Step: ", step)
            # # pwm = rot.rot_body2inertial(pwm, euler_angles, quaternion)
            # return setpoint

    # Define the drawing frame
    drawing_frame = camera_data.copy()
    
    # Draw the detected parallelogram on the drawing frame
    # drawing_frame = draw_parallelogram(drawing_frame, detect_parallelogram(camera_data))
    parallelogram = detect_parallelogram(camera_data)

    if turn_counter > 300 :
        # Reset the flags if no parallelogram is detected after 20 turns
        right_scan = False
        left_scan = False
        center_aligned = False
        go_forward = False
        print("Door positions: ", door_pos)
        return initial_pos

    if not(purple_color_detected(drawing_frame, pixel_threshold= 200)):
        # No Purle detected, reset the flags
        right_scan = False
        left_scan = False
        center_aligned = False
        go_forward = False
        turn_counter += 1
        return rotate(sensor_data, 15)  # Rotate to search for the parallelogram
    elif purple_color_detected(drawing_frame, pixel_threshold= 25000) and not inside_door:
        inside_door = True
        if door_pos[0] == [0,0,0]:
            door_pos[0] = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']]
        elif distance(sensor_data, door_pos[-1]) > 1.5:
            door_pos.append([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']]) 
            return move_towards_camera_point(sensor_data, [S_WIDTH / 2, S_HEIGHT / 2], camera_data, speed=2)  # Move towards the center of the camera frame

    if parallelogram is None and inside_door:
        # No parallelogram detected, reset the flags
        right_scan = False
        left_scan = False
        center_aligned = False
        go_forward = False
        inside_door = False
        return rotate(sensor_data, 15)

    if parallelogram is None and purple_color_detected(drawing_frame, pixel_threshold= 200):
        # No parallelogram detected, reset the flags
        right_scan = False
        left_scan = False
        center_aligned = False
        go_forward = False
        #return rotate(sensor_data, 15)  # Rotate to search for the parallelogram
        return move_right(sensor_data, distance= 2)  # Move right to search for the parallelogram
    
    inside_door = False


    if parallelogram is not None:
        turn_counter = 0
        # Calculate the centroid of the parallelogram in pixel coordinates
        points = parallelogram.reshape(-1, 2)
        centroid = np.mean(points, axis=0)
        cv2.circle(drawing_frame, (int(centroid[0]), int(centroid[1])), 5, (255, 0, 0), -1) 

        # centroid = choose_target(sensor_data, camera_data)
        # cv2.circle(drawing_frame, (int(centroid[0]), int(centroid[1])), 5, (255, 0, 0), -1) 
        # print("centroid = ", centroid)

        if is_center_aligned(centroid, camera_data):
            return move_towards_camera_point(sensor_data, centroid, camera_data, 0.7)

        if centroid is not None:
            #center_aligned = True
            return align_center(centroid, sensor_data, camera_data)
        

        
        
        # slope = paral_slope(parallelogram)  
        # if center_aligned and slope < -0.5:
        #     center_aligned = False
        #     go_forward = True
        #     slope_coeff = 2 * abs(slope)
        #     #return move_right(sensor_data, slope_coeff)
        #     return move_right_PID(sensor_data, distance= slope_coeff)
        # elif center_aligned and slope > 0.5:
        #     center_aligned = False
        #     go_forward = True
        #     slope_coeff = 2 * abs(slope)
        #     # return rotate_on_circle(sensor_data, centroid, 3, 0.1, dt)
        #     return move_left_PID(sensor_data, distance= slope_coeff)
        # if centroid is not None and not center_aligned:
        #     #print("distance = ", abs(centroid[0] - sensor_data['z_global']))
        #     center_aligned = True
        #     # return align_center(centroid, sensor_data, camera_data)
        # else :
        #     # center_aligned = True
        #     go_forward = True
        #     return move_towards_camera_point(sensor_data, centroid, camera_data, 0.5)

        # if centroid is not None and not center_aligned:
        #     #print("distance = ", abs(centroid[0] - sensor_data['z_global']))
        #     center_aligned = True
        #     return align_center(centroid, sensor_data, camera_data)
        

        # if go_forward and center_aligned :
        #     return move_towards_camera_point(sensor_data, centroid, camera_data, 0.5)
        

        # print("Slope: ", slope)
        
        
            

    current_pose = ([sensor_data["x_global"], sensor_data["y_global"], sensor_data["z_global"]], 
                    [sensor_data["q_x"], sensor_data["q_y"], sensor_data["q_z"], sensor_data["q_w"]])


    return [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']]  # No movement command

def align_center(center, sensor_data, camera_data):
    err = np.array([center[0] - camera_data.shape[1] / 2, center[1] - camera_data.shape[0] / 2])
    Kp_yaw = 0.5
    yaw_err = err[0] / f_pixel 
    yaw = sensor_data['yaw'] - Kp_yaw * yaw_err
    yaw = np.clip(yaw, -np.pi, np.pi)
    Kp_z = 0.2
    z_err = err[1]
    z = sensor_data['z_global'] - Kp_z * z_err
    z = np.clip(z, 0.2, 2.0)
    control_command = [sensor_data['x_global'], sensor_data['y_global'], z, yaw]
    return control_command

import numpy as np

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

    #control_command = rot.rot_body2inertial(control_command, [sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']], [sensor_data['q_x'], sensor_data['q_y'], sensor_data['q_z'], sensor_data['q_w']])

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
    Kp_yaw = 0.5
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

def PID_command(center, angle, sensor_data):
    """
    Generates a PID control command based on the detected center of the parallelogram and the drone's current state.

    Args:
        center: Tuple containing the x and y coordinates of the detected center.
        angle: Angle of the door.
        sensor_data: Dictionary containing the current sensor data.
    """

    err_x = center[0] - S_WIDTH / 2
    err_y = -(center[1] - S_HEIGHT / 2)
    err_angle = angle
    print(err_angle)
    # Proportional gains
    kp_y = 0.005
    kp_z = 0.3
    kp_yaw = 0.005
    orientation = np.array([sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']])
    R = rot.euler2rotmat(orientation)
    loc = R.T @ np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']])
    x_local, y_local, z_local = loc[0], loc[1], loc[2]
    x_pid = x_local + 0.5
    y_pid = y_local - kp_y * err_angle
    z_pid = z_local + kp_z * err_y
    glob = R @ np.array([x_pid, y_pid, z_pid])
    command_x, command_y, command_z = glob[0], glob[1], glob[2]
    command_yaw = sensor_data['yaw'] - kp_yaw * err_x

    return [command_x, command_y, command_z, command_yaw]



def move_right_PID(sensor_data, distance=0.5, kp=1.0):
    """
    Move right by applying a control in the drone's local frame, 
    then transform it to the global frame using orientation.
    """
    # Orientation and position
    orientation = np.array([sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']])
    R = rot.euler2rotmat(orientation)  # Rotation matrix from drone body to world

    # Target movement in local frame: +Y (right in body frame)
    target_local = np.array([0.0, kp * distance, 0.0])  # no change in z, just sidestep right
    current_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']])

    # Transform to global frame
    target_global_offset = R @ target_local
    target_pos = current_pos + target_global_offset
    ang = np.deg2rad(15)

    # Return target position with current yaw (no rotation change)
    return [target_pos[0], target_pos[1], target_pos[2], sensor_data['yaw']+ang]

def move_left_PID(sensor_data, distance=0.5, kp=1.0):
    """
    Move left by applying a control in the drone's local frame, 
    then transform it to the global frame using orientation.
    """
    orientation = np.array([sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']])
    R = rot.euler2rotmat(orientation)

    # Target movement in local frame: -Y (left in body frame)
    target_local = np.array([0.0, -kp * distance, 0.0])
    current_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']])

    target_global_offset = R @ target_local
    target_pos = current_pos + target_global_offset
    ang = np.deg2rad(-15)

    return [target_pos[0], target_pos[1], target_pos[2], sensor_data['yaw']+ang]

def rotate_on_circle(sensor_data, center, radius, angular_speed, dt):
    """
    Makes the drone rotate on a circular path of a given radius.

    Args:
        sensor_data: Dictionary containing the current sensor data.
        center: Tuple (x, y) representing the center of the circle.
        radius: Radius of the circular path (in meters).
        angular_speed: Angular speed of rotation (in radians per second).
        dt: Time step for the control loop (in seconds).

    Returns:
        control_command: List containing the control command [x, y, z, yaw].
    """
    # Current position of the drone
    x_current = sensor_data["x_global"]
    y_current = sensor_data["y_global"]
    z_current = sensor_data["z_global"]
    yaw_current = sensor_data["yaw"]

    # Calculate the angle of the drone relative to the center
    angle = np.arctan2(y_current - center[1], x_current - center[0])

    # Update the angle to move along the circle
    angle += angular_speed * dt  # Positive angular speed for counterclockwise, negative for clockwise

    # Calculate the new position on the circular path
    x_new = center[0] + radius * np.cos(angle)
    y_new = center[1] + radius * np.sin(angle)
    z_new = z_current  # Maintain the same altitude

    # Calculate the yaw to face the center of the circle
    yaw_new = np.arctan2(center[1] - y_new, center[0] - x_new)

    # Generate the control command
    control_command = [x_new, y_new, z_new, yaw_new]

    return control_command

def distance_to_target(sensor_data, target):
    """
    Calculate the distance from the drone to a target point.

    Args:
        sensor_data: Dictionary containing the current sensor data.
        target: Tuple (x, y) representing the target point.

    Returns:
        distance: Distance from the drone to the target point.
    """
    x_drone = sensor_data['x_global']
    y_drone = sensor_data['y_global']

    distance = np.sqrt((x_drone - target[0]) ** 2 + (y_drone - target[1]) ** 2)
    return distance

def choose_target(sensor_data, camera_data):
    """
    Choose the closest target from a list of targets.

    Args:
        sensor_data: Dictionary containing the current sensor data.
        targets: List of target points [(x1, y1), (x2, y2), ...].

    Returns:
        closest_target: The closest target point to the drone.
    """

    centers = get_parallelogram_centers(camera_data)

    distances = [distance_to_target(sensor_data, center) for center in centers]
    closest_target_index = np.argmin(distances)
    closest_target = centers[closest_target_index]
    
    return closest_target

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

def distance_points(point1, point2 ):
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


import numpy as np
import matplotlib.pyplot as plt
import time

on_ground = True
height_desired = 0.5
timer = None
startpos = None
timer_done = None
index_current_setpoint = 0

def path_planning(sensor_data, dt, setpoints, tol):
    global on_ground, height_desired, index_current_setpoint, timer, timer_done, startpos

    dt = dt-0.5
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