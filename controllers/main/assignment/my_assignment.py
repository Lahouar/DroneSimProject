import numpy as np
import time
import cv2
import assignment.drone_vision as dv
import assignment.triangulation as tr

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
nb_frames = 3

def get_command(sensor_data, camera_data, dt):

    # NOTE: Displaying the camera image with cv2.imshow() will throw an error because GUI operations should be performed in the main thread.
    # If you want to display the camera image you can call it main.py.

    # Take off example
    if sensor_data['z_global'] < 0.49:
        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
        return control_command

    # ---- YOUR CODE HERE ----
    control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]

    drawing_frame = camera_data.copy()

    #drawing_frame = dv.purple_mask(drawing_frame)
    #drawing_frame = dv.draw_rectangle(drawing_frame, dv.detect_rectangle(camera_data))
    drawing_frame = dv.draw_parallelogram(drawing_frame, dv.detect_parallelogram(camera_data))

    if not np.array_equal(drawing_frame, camera_data):

        global frames, poses

        # Always store the last 3 frames and their corresponding poses
        if len(frames) >= nb_frames:
            frames.pop(0)  # Remove the oldest frame
            poses.pop(0)   # Remove the oldest pose

        # Add the new frame and pose
        frames.append(camera_data.copy())

        # Position (x, y, z)
        position = [sensor_data["x_global"], sensor_data["y_global"], sensor_data["z_global"]]
        
        # Quaternion (q_x, q_y, q_z, q_w)
        quaternion = [sensor_data["q_x"], sensor_data["q_y"], sensor_data["q_z"], sensor_data["q_w"]]
        
        # Store the pose as a tuple of position and quaternion
        poses.append((position, quaternion))

        if len(frames) == nb_frames:
            center_3d = tr.triangulate_parallelogram_center(frames, poses, camera_params)

            # After calculating center_3d, project it back to the current camera view
            current_pose = ([sensor_data["x_global"], sensor_data["y_global"], sensor_data["z_global"]], 
                            [sensor_data["q_x"], sensor_data["q_y"], sensor_data["q_z"], sensor_data["q_w"]])
            
            pixel_coords = tr.project_3d_to_pixel(center_3d, current_pose, camera_params)

            cv2.circle(drawing_frame, (int(pixel_coords[0]), int(pixel_coords[1])), 5, (0, 255, 0), -1)

    cv2.imshow("Camera", drawing_frame)
    cv2.waitKey(1)
    
    
    return control_command # Ordered as array with: [pos_x_cmd, pos_y_cmd, pos_z_cmd, yaw_cmd] in meters and radians