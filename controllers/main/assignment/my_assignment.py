import numpy as np
import time
import cv2
import assignment.drone_vision as dv
import assignment.triangulation as tr
import exercises.ex0_rotations as rot
from assignment.target_manager import target_point

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

frames = []
poses = []
nb_frames = 2
control_command = [0, 0, 0, 0]  # Initialize control command
f_pixel = 161.01
right_scan = False
left_scan = False
center_aligned = False
go_forward = False

def get_command(sensor_data, camera_data, dt):

    global frames, poses, control_command, right_scan, left_scan, center_aligned, go_forward
    # NOTE: Displaying the camera image with cv2.imshow() will throw an error because GUI operations should be performed in the main thread.
    # If you want to display the camera image you can call it main.py.

    # Take off example
    if sensor_data['z_global'] < 1:
        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.1, sensor_data['yaw']]
        return control_command

    # ---- YOUR CODE HERE ----

    # Position (x, y, z)
    position = [sensor_data["x_global"], sensor_data["y_global"], sensor_data["z_global"]]

    euler_angles = [sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']]
        
    # Quaternion (q_x, q_y, q_z, q_w)
    quaternion = [sensor_data["q_x"], sensor_data["q_y"], sensor_data["q_z"], sensor_data["q_w"]]

    # Always store the last 3 frames and their corresponding poses
    if len(frames) == nb_frames:
        frames = []
        poses = []

    # Define the drawing frame
    drawing_frame = camera_data.copy()
    
    # Draw the detected parallelogram on the drawing frame
    drawing_frame = dv.draw_parallelogram(drawing_frame, dv.detect_parallelogram(camera_data))
    parallelogram = dv.detect_parallelogram(camera_data)

    if parallelogram is not None:
        # Calculate the centroid of the parallelogram in pixel coordinates
        points = parallelogram.reshape(-1, 2)
        centroid = np.mean(points, axis=0)
        cv2.circle(drawing_frame, (int(centroid[0]), int(centroid[1])), 5, (255, 0, 0), -1) 
        

        if centroid is not None and not center_aligned:
            print("distance = ", abs(centroid[0] - sensor_data['z_global']))
            center_aligned = True
            return align_center(centroid, sensor_data, camera_data)
        

        if go_forward and center_aligned:
            return move_towards_camera_point(sensor_data, centroid, camera_data, 0.5)
        
        if center_aligned and paral_slope(parallelogram) < 0:
            center_aligned = False
            go_forward = True
            return move_right(sensor_data, 2)
        elif center_aligned and paral_slope(parallelogram) > 0:
            center_aligned = False
            return move_left(sensor_data, 0.5)
            

        # if centroid is not None and abs(centroid[0] - sensor_data['z_global']) < 130:
        #     print("distance = ", abs(centroid[0] - sensor_data['z_global']))
        #     return align_center(centroid, sensor_data, camera_data)
        # elif not right_scan and not left_scan:
        #     right_scan = True
        #     return move_right(sensor_data, 5)
        # elif right_scan and not left_scan:
        #     frames.append(camera_data.copy())
        #     poses.append((position, quaternion))
        #     left_scan = True
        #     return move_left(sensor_data, 5)
        # elif left_scan and right_scan :
        #     frames.append(camera_data.copy())
        #     poses.append((position, quaternion))
        #     right_scan = False
        #     left_scan = False
        
    # Store the pose as a tuple of position and quaternion
    #poses.append((position, quaternion))

    current_pose = ([sensor_data["x_global"], sensor_data["y_global"], sensor_data["z_global"]], 
                    [sensor_data["q_x"], sensor_data["q_y"], sensor_data["q_z"], sensor_data["q_w"]])

    # if len(frames) == nb_frames:
    #     center_3d = tr.triangulate_parallelogram_center(frames, poses, camera_params)

    #     if center_3d is not None:
    #         # Draw the center point in the camera frame
    #         # After calculating center_3d, project it back to the current camera view
    #         # pixel_coords = tr.project_3d_to_pixel(center_3d, current_pose, camera_params)
    #         # cv2.circle(drawing_frame, (int(pixel_coords[0]), int(pixel_coords[1])), 5, (0, 255, 0), -1)

    #         # Store the next target for the drone to fly to
    #         if target_point[0] is None:
    #             target_point[0] = float(center_3d[0])
    #             target_point[1] = float(center_3d[1])
    #             target_point[2] = float(center_3d[2])
    #             yaw_cmd = np.arctan2(target_point[1], target_point[0])
    #             print("Target point set:", target_point)
    #             control_command = [target_point[0], target_point[1], target_point[2], sensor_data['yaw']]
    #             control_command = rot.rot_body2inertial(control_command, euler_angles, quaternion) 
    #             control_command = [float(i) for i in control_command]  # Convert to float
    #             print("Control command:", control_command) 
    #         else:
    #         #     # If the target point is already set, check if it needs to be updated
    #         #     #calculate the difference between center and target point
    #             diff = np.array(center_3d) - np.array(target_point)
    #             dist_squared = np.dot(diff, diff)  # Squared distance
    #         #     #calculate the distance between the drone and the target point
    #             drone_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']])
    #             target_pos = np.array(target_point)
    #             drone_target_diff = drone_pos - target_pos
    #             dist_drone = np.linalg.norm(drone_target_diff)  # Euclidean distance
    #             if dist_squared > 55 : #and dist_drone < 0.1 :
    #                 target_point[0] = float(center_3d[0])
    #                 target_point[1] = float(center_3d[1])
    #                 target_point[2] = float(center_3d[2])
    #                 yaw_cmd = np.arctan2(target_point[1], target_point[0])
    #                 print("Target point updated:", target_point)
    #                 control_command = [target_point[0], target_point[1], target_point[2], sensor_data['yaw']]
    #                 control_command = rot.rot_body2inertial(control_command, euler_angles, quaternion)
    #                 control_command = [float(i) for i in control_command]  # Convert to float
    #                 print("Control command:", control_command) 
                                 

    return [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']]  # No movement command

def align_center(center, sensor_data, camera_data):
    err = np.array([center[0] - camera_data.shape[1] / 2, center[1] - camera_data.shape[0] / 2])
    Kp_yaw = 0.5
    yaw_err = err[0] / f_pixel 
    yaw = sensor_data['yaw'] - Kp_yaw * yaw_err
    yaw = np.clip(yaw, -np.pi, np.pi)
    Kp_z = 0.01
    z_err = err[1]
    z = sensor_data['z_global'] - Kp_z * z_err
    z = np.clip(z, 0.2, 2.0)
    control_command = [sensor_data['x_global'], sensor_data['y_global'], z, yaw]
    return control_command


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


