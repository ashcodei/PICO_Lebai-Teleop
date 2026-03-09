"""
Coordinate Frame Utilities for PICO VR Teleoperation

Math utilities for VR-to-robot coordinate transforms.
"""

import numpy as np


# Default VR-to-Robot rotation matrix
# PICO VR frame: X=right, Y=up, Z=back (OpenXR convention)
# Lebai robot frame: X=forward, Y=left, Z=up (common robot convention)
# Assumes operator faces the robot.
R_VR_TO_ROBOT_DEFAULT = np.array([
    [ 0,  0, -1],   # Robot X = -VR Z (forward)
    [-1,  0,  0],   # Robot Y = -VR X (left)
    [ 0,  1,  0],   # Robot Z =  VR Y (up)
], dtype=np.float64)


def quat_to_rotation_matrix(q_wxyz):
    """Convert quaternion (w, x, y, z) to 3x3 rotation matrix."""
    w, x, y, z = q_wxyz
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def rotation_matrix_to_quat(R):
    """Convert 3x3 rotation matrix to quaternion (w, x, y, z)."""
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def rotate_quaternion(q_wxyz, R):
    """
    Rotate a quaternion by a 3x3 rotation matrix using conjugation.

    Applies: q' = R_quat * q * R_quat_inv
    This matches the XRoboToolkit reference implementation.
    """
    R_quat = rotation_matrix_to_quat(R)
    R_quat_inv = np.array([R_quat[0], -R_quat[1], -R_quat[2], -R_quat[3]])
    return quat_multiply(quat_multiply(R_quat, q_wxyz), R_quat_inv)


def quat_diff_to_rotvec(q1_wxyz, q2_wxyz):
    """
    Compute rotation vector from q1 to q2.

    Returns [rx, ry, rz] (axis-angle representation).
    """
    # q_diff = q2 * q1_inv
    # q1_inv for unit quaternion: (w, -x, -y, -z)
    q1_inv = np.array([q1_wxyz[0], -q1_wxyz[1], -q1_wxyz[2], -q1_wxyz[3]])
    q_diff = quat_multiply(q2_wxyz, q1_inv)

    # Ensure w >= 0 for shortest path
    if q_diff[0] < 0:
        q_diff = -q_diff

    # Convert to axis-angle (matching XRoboToolkit reference)
    w = q_diff[0]
    vec = q_diff[1:]

    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    if angle < 1e-6:
        return np.zeros(3)

    sin_half = np.sin(angle / 2.0)
    if sin_half < 1e-6:
        return np.zeros(3)

    axis = vec / sin_half
    return axis * angle


def quat_multiply(q1, q2):
    """Multiply two quaternions (w, x, y, z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_nlerp(q1_wxyz, q2_wxyz, alpha):
    """
    Normalized linear interpolation between two quaternions.

    Args:
        q1_wxyz: Start quaternion (w, x, y, z)
        q2_wxyz: End quaternion (w, x, y, z)
        alpha: Interpolation factor (0=q1, 1=q2)
    """
    q1 = np.asarray(q1_wxyz, dtype=np.float64)
    q2 = np.asarray(q2_wxyz, dtype=np.float64)
    # Take shortest path
    if np.dot(q1, q2) < 0:
        q2 = -q2
    q = (1.0 - alpha) * q1 + alpha * q2
    return q / np.linalg.norm(q)


def rotvec_to_quat_wxyz(rotvec):
    """Convert rotation vector [rx, ry, rz] to quaternion (w, x, y, z)."""
    angle = np.linalg.norm(rotvec)
    if angle < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rotvec / angle
    half = angle / 2.0
    w = np.cos(half)
    xyz = axis * np.sin(half)
    return np.array([w, xyz[0], xyz[1], xyz[2]])


def clamp_tcp_to_workspace(tcp, limits):
    """
    Clamp XYZ of TCP pose to workspace bounding box.

    Args:
        tcp: [x, y, z, rx, ry, rz] in meters/radians
        limits: dict with x_min, x_max, y_min, y_max, z_min, z_max

    Returns:
        Clamped TCP list
    """
    clamped = list(tcp)
    clamped[0] = np.clip(clamped[0], limits['x_min'], limits['x_max'])
    clamped[1] = np.clip(clamped[1], limits['y_min'], limits['y_max'])
    clamped[2] = np.clip(clamped[2], limits['z_min'], limits['z_max'])
    return clamped


def clamp_speed(prev_tcp, target_tcp, dt, max_linear, max_angular):
    """
    Limit per-step movement to enforce max speed.

    Args:
        prev_tcp: Previous TCP [x, y, z, rx, ry, rz]
        target_tcp: Target TCP [x, y, z, rx, ry, rz]
        dt: Time step in seconds
        max_linear: Max linear speed in m/s
        max_angular: Max angular speed in rad/s

    Returns:
        Speed-clamped target TCP
    """
    if dt <= 0:
        return list(target_tcp)

    result = list(target_tcp)

    # Linear speed clamping
    delta_pos = np.array(target_tcp[:3]) - np.array(prev_tcp[:3])
    linear_dist = np.linalg.norm(delta_pos)
    max_linear_dist = max_linear * dt

    if linear_dist > max_linear_dist and linear_dist > 1e-8:
        scale = max_linear_dist / linear_dist
        for i in range(3):
            result[i] = prev_tcp[i] + delta_pos[i] * scale

    # Angular speed clamping
    delta_rot = np.array(target_tcp[3:6]) - np.array(prev_tcp[3:6])
    angular_dist = np.linalg.norm(delta_rot)
    max_angular_dist = max_angular * dt

    if angular_dist > max_angular_dist and angular_dist > 1e-8:
        scale = max_angular_dist / angular_dist
        for i in range(3):
            result[i + 3] = prev_tcp[i + 3] + delta_rot[i] * scale

    return result


def compute_calibration_matrix(origin_vr, x_vr, z_vr):
    """
    Compute R_VR_TO_ROBOT from 3-point calibration.

    Args:
        origin_vr: VR position at origin [x, y, z]
        x_vr: VR position along desired Robot +X [x, y, z]
        z_vr: VR position along desired Robot +Z [x, y, z]

    Returns:
        3x3 rotation matrix R_VR_TO_ROBOT
    """
    origin = np.array(origin_vr)
    x_point = np.array(x_vr)
    z_point = np.array(z_vr)

    # Gram-Schmidt orthogonalization
    x_hat = x_point - origin
    x_hat = x_hat / np.linalg.norm(x_hat)

    z_raw = z_point - origin
    z_hat = z_raw - np.dot(z_raw, x_hat) * x_hat
    z_hat = z_hat / np.linalg.norm(z_hat)

    y_hat = np.cross(z_hat, x_hat)

    R = np.column_stack([x_hat, y_hat, z_hat]).T
    return R
