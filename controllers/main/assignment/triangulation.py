import numpy as np
import assignment.drone_vision as dv
from exercises.ex0_rotations import quaternion2rotmat


def triangulate_2lines(P, Q, v1, v2):
    """
    Triangulates a 3D point from two camera views.
    
    Parameters:
    P  -- Position of first camera (numpy array of shape (3,))
    Q  -- Position of second camera (numpy array of shape (3,))
    v1 -- Viewing vector from first camera (numpy array of shape (3,))
    v2 -- Viewing vector from second camera (numpy array of shape (3,))
    
    Returns:
    H  -- Estimated 3D point (numpy array of shape (3,))
    """
    # Construct matrix A and vector b for solving lambda and mu
    A = np.column_stack((v1, -v2))  # Shape (3,2)
    b = Q - P  # Shape (3,)
    
    # Solve for lambda and mu using least squares (pseudoinverse)
    lamb_mu = np.linalg.lstsq(A, b, rcond=None)[0]  # [lambda, mu]
    lambda_, mu = lamb_mu
    
    # Compute F and G
    F = P + lambda_ * v1
    G = Q + mu * v2
    
    # Compute midpoint H
    H = (F + G) / 2
    
    return H

def triangulate_parallelogram_center(frames, camera_poses, camera_params):
    """
    Triangulate the 3D center of a parallelogram detected in multiple camera frames
    
    Args:
        frames: List of frames (images) containing the parallelogram
        camera_poses: List of (position, orientation) tuples for each camera
                     position: (x, y, z)
                     orientation: quaternion (x, y, z, w)
        camera_params: Dictionary with camera parameters:
                     - W: image width in pixels
                     - H: image height in pixels
                     - FOV: field of view in radians
                     - camera_to_body_offset: (x, y, z) offset from body to camera
    Returns:
        3D position of the parallelogram's center in world coordinates
    """
    
    # First detect the parallelogram in each frame and find its centroid
    parallelogram_centers_px = []
    
    for frame in frames:
        # Detect parallelogram
        points = dv.detect_parallelogram(frame)
        
        if points is not None and len(points) == 4:
            # Calculate centroid of the parallelogram in pixel coordinates
            # Reshape points to Nx2 array
            points = points.reshape(-1, 2)
            centroid = np.mean(points, axis=0)
            parallelogram_centers_px.append(centroid)
        else:
            # If no parallelogram detected, use image center as fallback
            centroid = (camera_params['W']/2, camera_params['H']/2)
            parallelogram_centers_px.append(centroid)
    
    # Now triangulate using all pairs of observations
    num_views = len(frames)
    triangulated_points = []
    
    # Triangulate between all unique pairs of views
    for i in range(num_views):
        for j in range(i+1, num_views):
            point_3d = triangulate_point_2(
                parallelogram_centers_px[i],
                parallelogram_centers_px[j],
                camera_poses[i],
                camera_poses[j],
                camera_params
            )
            triangulated_points.append(point_3d)
    
    # Return the mean of all triangulated points
    return np.mean(triangulated_points, axis=0)

def triangulate_point_2(pixel_coords1, pixel_coords2, camera_pose1, camera_pose2, camera_params):
    """Triangulate a 3D point from two pixel coordinates and corresponding camera poses."""
    # Extract camera parameters
    W = camera_params['W']
    H = camera_params['H']
    FOV = camera_params['FOV']
    cam_offset = np.array(camera_params['camera_to_body_offset'])

    # Focal length in pixels
    f_pixels = W / (2 * np.tan(FOV / 2))

    def pixel_to_camera_vector(u, v):
        u_centered = u - W / 2
        v_centered = -(v - H / 2)  # y-axis correction
        return np.array([u_centered, v_centered, f_pixels])

    # Pixel vectors in camera frame
    v1 = pixel_to_camera_vector(*pixel_coords1)
    v2 = pixel_to_camera_vector(*pixel_coords2)

    # Camera to body frame rotation (fixed)
    R_cam_to_body = np.array([
        [0, -1, 0],   # x_cam = -y_body
        [0, 0, -1],   # y_cam = -z_body
        [1, 0, 0]     # z_cam = x_body
    ])

    # Get rotation matrices from body to world using quaternion2rotmat
    R_body_to_world1 = quaternion2rotmat(camera_pose1[1])
    R_body_to_world2 = quaternion2rotmat(camera_pose2[1])

    # Combined camera to world rotation
    R_c1_to_w = R_body_to_world1 @ R_cam_to_body
    R_c2_to_w = R_body_to_world2 @ R_cam_to_body

    # Direction vectors in world frame
    r = R_c1_to_w @ v1
    s = R_c2_to_w @ v2

    # Camera positions in world frame
    body_pos1 = np.array(camera_pose1[0])
    body_pos2 = np.array(camera_pose2[0])

    P = body_pos1 + R_body_to_world1 @ cam_offset
    Q = body_pos2 + R_body_to_world2 @ cam_offset

    # Solve for scalars λ and μ using least squares
    A = np.column_stack((-r, s))
    b = Q - P
    try:
        lambdas = np.linalg.lstsq(A, b, rcond=None)[0]
        lambda_, mu = lambdas[0], lambdas[1]
    except np.linalg.LinAlgError:
        lambda_, mu = 1.0, 1.0  # Fallback if singular

    F = P + lambda_ * r
    G = Q + mu * s

    return (F + G) / 2


def triangulate_point(pixel_coords1, pixel_coords2, camera_pose1, camera_pose2, camera_params):
    """ (Same implementation as before) """
    # Extract camera parameters
    W = camera_params['W']
    H = camera_params['H']
    FOV = camera_params['FOV']
    cam_offset = np.array(camera_params['camera_to_body_offset'])
    
    # Calculate focal length in pixels
    f_pixels = W / (2 * np.tan(FOV / 2))
    
    # Convert pixel coordinates to camera vectors
    def pixel_to_camera_vector(u, v):
        # Convert to centered coordinates
        u_centered = u - W/2
        v_centered = -(v - H/2)  # Negative because image y-axis points down
        return np.array([u_centered, v_centered, f_pixels])
    
    v1 = pixel_to_camera_vector(*pixel_coords1)
    v2 = pixel_to_camera_vector(*pixel_coords2)
    
    # Get camera-to-world rotation matrices
    def get_rotation_matrix(orientation):
        # Convert quaternion to rotation matrix
        x, y, z, w = orientation
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])
    
    # Camera to body rotation (due to frame convention difference)
    R_cam_to_body = np.array([
        [0, -1, 0],  # x_cam = -y_body
        [0, 0, -1],   # y_cam = -z_body
        [1, 0, 0]     # z_cam = x_body
    ])
    
    # Body to world rotation
    R_body_to_world1 = get_rotation_matrix(camera_pose1[1])
    R_body_to_world2 = get_rotation_matrix(camera_pose2[1])
    
    # Combined camera to world rotation
    R_c1_to_w = R_body_to_world1 @ R_cam_to_body
    R_c2_to_w = R_body_to_world2 @ R_cam_to_body
    
    # Transform vectors to world frame
    r = R_c1_to_w @ v1
    s = R_c2_to_w @ v2
    
    # Get camera positions in world frame (body position + offset transformed to world)
    body_pos1 = np.array(camera_pose1[0])
    body_pos2 = np.array(camera_pose2[0])
    
    P = body_pos1 + R_body_to_world1 @ cam_offset
    Q = body_pos2 + R_body_to_world2 @ cam_offset
    
    # Solve for lambda and mu using least squares
    A = np.column_stack((-r, s))
    b = Q - P
    try:
        lambdas = np.linalg.lstsq(A, b, rcond=None)[0]
        lambda_ = lambdas[0]
        mu = lambdas[1]
    except np.linalg.LinAlgError:
        # Fallback to midpoint if lines are nearly parallel
        lambda_ = 1.0
        mu = 1.0
    
    # Calculate points on both lines
    F = P + lambda_ * r
    G = Q + mu * s
    
    # Return midpoint as triangulated point
    return (F + G) / 2


def project_3d_to_pixel(point_3d, camera_pose, camera_params):
    """
    Project a 3D point in world coordinates to 2D pixel coordinates
    
    Args:
        point_3d: 3D point in world coordinates (x, y, z)
        camera_pose: (position, orientation) of camera in world frame
        camera_params: camera parameters dictionary
    Returns:
        Pixel coordinates (u, v)
    """
    # Extract parameters
    W = camera_params['W']
    H = camera_params['H']
    FOV = camera_params['FOV']
    cam_offset = np.array(camera_params['camera_to_body_offset'])
    f_pixels = W / (2 * np.tan(FOV / 2))
    
    # Get rotation matrix from quaternion
    def get_rotation_matrix(orientation):
        x, y, z, w = orientation
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])
    
    # Camera to body rotation
    R_cam_to_body = np.array([
        [0, -1, 0],
        [0, 0, -1],
        [1, 0, 0]
    ])
    
    # Body to world rotation
    R_body_to_world = get_rotation_matrix(camera_pose[1])
    
    # World to camera rotation
    R_world_to_cam = (R_body_to_world @ R_cam_to_body).T
    
    # Camera position in world frame
    cam_pos_world = np.array(camera_pose[0]) + R_body_to_world @ cam_offset
    
    # Transform point to camera coordinates
    point_cam = R_world_to_cam @ (np.array(point_3d) - cam_pos_world)
    
    # Project to image plane
    u = (point_cam[0] / point_cam[2]) * f_pixels + W/2
    v = -(point_cam[1] / point_cam[2]) * f_pixels + H/2  # Negative because image y-axis points down
    
    return (int(u), int(v))