"""
PICO VR Teleoperation Controller for Lebai LM3
PICO VR 遥操作控制器（乐白LM3）

Bridges XRoboToolkit SDK tracking data to Lebai robot arm motion.
将XRoboToolkit SDK跟踪数据桥接到乐白机械臂运动。
"""

import threading
import time
import logging
import numpy as np

from frame_utils import (
    R_VR_TO_ROBOT_DEFAULT,
    rotate_quaternion,
    quat_diff_to_rotvec,
    quat_multiply,
    quat_nlerp,
    rotvec_to_quat_wxyz,
    clamp_tcp_to_workspace,
    clamp_speed,
)

logger = logging.getLogger(__name__)

# Try to import xrobotoolkit_sdk
_xrt_available = False
try:
    import xrobotoolkit_sdk as xrt
    _xrt_available = True
except ImportError:
    logger.info("xrobotoolkit_sdk not installed — VR features unavailable")

# Try to import lebai_sdk
_lebai_available = False
try:
    import lebai_sdk
    _lebai_available = True
except ImportError:
    logger.info("lebai_sdk not installed — robot features unavailable")


class PicoTeleopController:
    """
    PICO VR teleoperation controller for Lebai LM3.
    PICO VR 遥操作控制器（乐白LM3）。

    Reads VR controller pose + buttons, computes delta TCP,
    and streams movel commands to the Lebai robot.
    """

    # States
    STATE_DISCONNECTED = "disconnected"
    STATE_SDK_CONNECTED = "sdk_connected"
    STATE_RUNNING = "running"
    STATE_PAUSED = "paused"
    STATE_ERROR = "error"
    STATE_EMERGENCY_STOP = "emergency_stop"

    def __init__(
        self,
        lebai_ip="10.20.17.1",
        scale_factor=0.5,
        control_hz=50,
        max_linear_speed=0.5,
        max_angular_speed=1.0,
        acceleration=0.8,
        deadzone=0.05,
        gripper_force=50.0,
        gripper_deadband=2.0,
        workspace_limits=None,
        frame_rotation=None,
    ):
        # Config
        self._lebai_ip = lebai_ip
        self._scale_factor = scale_factor
        self._control_hz = control_hz
        self._max_linear_speed = max_linear_speed
        self._max_angular_speed = max_angular_speed
        self._acceleration = acceleration
        self._deadzone = deadzone
        self._gripper_force = gripper_force
        self._gripper_deadband = gripper_deadband

        self._workspace_limits = workspace_limits or {
            'x_min': -0.5, 'x_max': 0.5,
            'y_min': -0.5, 'y_max': 0.5,
            'z_min': 0.05, 'z_max': 0.6,
        }

        if frame_rotation is not None:
            self._R_vr_to_robot = np.array(frame_rotation, dtype=np.float64).reshape(3, 3)
        else:
            self._R_vr_to_robot = R_VR_TO_ROBOT_DEFAULT.copy()

        # State
        self._state = self.STATE_DISCONNECTED
        self._state_lock = threading.Lock()
        self._error_message = ""

        # Lebai robot instance
        self._robot = None
        self._lebai_connected = False

        # VR SDK state
        self._xrt_connected = False
        self._pico_streaming = False  # True only when actually receiving VR data
        self._pico_data_timeout = 2.0  # seconds without data before considered disconnected
        self._last_vr_data_time = 0.0

        # Control thread
        self._control_thread = None
        self._stop_event = threading.Event()
        self._paused = False

        # Teleop state
        self._is_controlling = False
        self._init_controller_xyz = None
        self._init_controller_quat = None
        self._prev_joint_angles = None
        self._init_tcp_pose = None
        self._last_target_tcp = [0.0] * 6
        self._last_vr_pose = [0.0] * 7
        self._last_grip_val = 0.0
        self._last_trigger_val = 0.0
        self._last_left_trigger_val = 0.0
        self._last_left_grip_val = 0.0

        # EMA smoothing state for VR input filtering
        self._smooth_xyz = None
        self._smooth_quat = None
        self._ema_alpha = 0.4  # EMA factor (lower = more smoothing)

        # Orientation tracking (quaternion composition, not rotvec addition)
        self._init_vr_quat = None
        self._init_robot_quat = None

        # IK reference joint support (auto-detected)
        self._ik_supports_ref = True

        # Gripper state
        self._last_gripper_amplitude = 100.0  # Start open
        self._gripper_target = 100.0  # Tracks desired amplitude (accumulates small changes)
        self._last_gripper_cmd_time = 0.0  # Rate-limit gripper commands
        self._gripper_dt = 0.02  # Will be updated with actual_dt each tick
        self._suction_on = False
        self._prev_a_button = False
        self._prev_y_button = False

        # Cached robot data
        self._cached_joint_positions = [0.0] * 6
        self._cached_tcp_position = [0.0] * 6

        # VR preview mode (runs without Lebai, computes simulated target)
        self._preview_thread = None
        self._preview_stop = threading.Event()
        self._preview_active = False
        self._simulated_tcp = [0.0] * 6  # What the robot TCP *would* be
        self._vr_raw_quat = [1.0, 0.0, 0.0, 0.0]  # w,x,y,z raw from VR
        self._vr_delta_xyz = [0.0, 0.0, 0.0]  # Current delta in robot frame
        self._vr_delta_rot = [0.0, 0.0, 0.0]  # Current delta rotation
        self._a_button = False
        self._b_button = False
        self._x_button = False
        self._y_button = False

        # Callbacks for UI updates
        self._state_callbacks = []

    # ======================================================================
    # Properties
    # ======================================================================

    @property
    def state(self):
        with self._state_lock:
            return self._state

    @state.setter
    def state(self, new_state):
        with self._state_lock:
            old = self._state
            self._state = new_state
        for cb in self._state_callbacks:
            try:
                cb(old, new_state)
            except Exception:
                pass

    @property
    def error_message(self):
        return self._error_message

    @property
    def is_controlling(self):
        return self._is_controlling

    @property
    def xrt_available(self):
        return _xrt_available

    @property
    def lebai_available(self):
        return _lebai_available

    @property
    def xrt_connected(self):
        return self._xrt_connected

    @property
    def lebai_connected(self):
        return self._lebai_connected

    @property
    def last_target_tcp(self):
        return list(self._last_target_tcp)

    @property
    def last_vr_pose(self):
        return list(self._last_vr_pose)

    @property
    def last_grip_val(self):
        return self._last_grip_val

    @property
    def last_trigger_val(self):
        return self._last_trigger_val

    @property
    def last_gripper_amplitude(self):
        return self._last_gripper_amplitude

    @property
    def suction_on(self):
        return self._suction_on

    @property
    def cached_joint_positions(self):
        return list(self._cached_joint_positions)

    @property
    def cached_tcp_position(self):
        return list(self._cached_tcp_position)

    @property
    def scale_factor(self):
        return self._scale_factor

    @property
    def max_linear_speed(self):
        return self._max_linear_speed

    @property
    def control_hz(self):
        return self._control_hz

    def add_state_callback(self, callback):
        self._state_callbacks.append(callback)

    # ======================================================================
    # Connection
    # ======================================================================

    def connect_xrt(self) -> bool:
        """Connect to XRoboToolkit SDK / 连接 XRoboToolkit SDK"""
        if not _xrt_available:
            self._error_message = "xrobotoolkit_sdk not installed"
            logger.warning(self._error_message)
            return False

        # Set up ADB reverse ports so PICO can reach PC Service via localhost
        try:
            import subprocess
            subprocess.run(["adb", "reverse", "--remove-all"], capture_output=True, timeout=5)
            subprocess.run(["adb", "reverse", "tcp:60061", "tcp:60061"], capture_output=True, timeout=5)
            subprocess.run(["adb", "reverse", "tcp:63901", "tcp:63901"], capture_output=True, timeout=5)
            logger.info("ADB reverse ports set up (60061, 63901)")
        except Exception as e:
            logger.warning(f"ADB reverse setup failed (PICO may not be USB-connected): {e}")

        try:
            xrt.init()
            self._xrt_connected = True
            logger.info("XRoboToolkit SDK connected")

            # Auto-start VR preview (reads VR data without needing Lebai)
            self._start_preview()

            if self._lebai_connected:
                self.state = self.STATE_SDK_CONNECTED
            else:
                self.state = self.STATE_SDK_CONNECTED
            return True
        except Exception as e:
            self._error_message = f"XRT init failed: {e}"
            logger.error(self._error_message)
            return False

    def disconnect_xrt(self) -> bool:
        """Disconnect XRoboToolkit SDK / 断开 XRoboToolkit SDK"""
        self._stop_preview()

        if self._state == self.STATE_RUNNING:
            self.stop_teleop()

        if _xrt_available and self._xrt_connected:
            try:
                xrt.close()
            except Exception as e:
                logger.warning(f"XRT close error: {e}")

        self._xrt_connected = False
        self._pico_streaming = False
        if self._lebai_connected:
            self.state = self.STATE_SDK_CONNECTED
        else:
            self.state = self.STATE_DISCONNECTED
        logger.info("XRoboToolkit SDK disconnected")
        return True

    def connect_lebai(self) -> bool:
        """Connect to Lebai robot / 连接乐白机器人"""
        if not _lebai_available:
            self._error_message = "lebai_sdk not installed"
            logger.warning(self._error_message)
            return False

        try:
            self._robot = lebai_sdk.connect(self._lebai_ip, False)
            time.sleep(1.0)

            if self._robot:
                self._lebai_connected = True
                self._robot.start_sys()
                time.sleep(0.5)
                self._update_robot_cache()
                logger.info(f"Lebai connected to {self._lebai_ip}")
                self.state = self.STATE_SDK_CONNECTED
                return True
            else:
                raise Exception("Failed to create robot instance")
        except Exception as e:
            self._error_message = f"Lebai connect failed: {e}"
            logger.error(self._error_message)
            return False

    def disconnect_lebai(self) -> bool:
        """Disconnect Lebai robot / 断开乐白机器人"""
        if self._state == self.STATE_RUNNING:
            self.stop_teleop()

        if self._robot:
            try:
                self._robot.stop_move()
            except Exception:
                pass
            self._robot = None

        self._lebai_connected = False
        if self._xrt_connected:
            self.state = self.STATE_SDK_CONNECTED
        else:
            self.state = self.STATE_DISCONNECTED
        logger.info("Lebai disconnected")
        return True

    # ======================================================================
    # Teleop Control
    # ======================================================================

    def start_teleop(self) -> bool:
        """Start teleoperation loop / 启动遥操作循环"""
        if not self._xrt_connected:
            self._error_message = "XRT SDK not connected"
            return False
        if not self._lebai_connected:
            self._error_message = "Lebai not connected"
            return False

        self._stop_event.clear()
        self._paused = False
        self._is_controlling = False

        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name="PicoTeleopLoop"
        )
        self._control_thread.start()
        self.state = self.STATE_RUNNING
        logger.info("Teleop started")
        return True

    def stop_teleop(self) -> bool:
        """Stop teleoperation loop / 停止遥操作循环"""
        self._stop_event.set()
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)
        self._control_thread = None
        self._is_controlling = False

        # Stop robot motion
        if self._robot:
            try:
                self._robot.stop_move()
            except Exception:
                pass

        if self._lebai_connected or self._xrt_connected:
            self.state = self.STATE_SDK_CONNECTED
        else:
            self.state = self.STATE_DISCONNECTED
        logger.info("Teleop stopped")
        return True

    def pause_teleop(self):
        """Pause teleoperation / 暂停遥操作"""
        self._paused = True
        if self._robot:
            try:
                self._robot.stop_move()
            except Exception:
                pass
        self.state = self.STATE_PAUSED

    def resume_teleop(self):
        """Resume teleoperation / 恢复遥操作"""
        self._paused = False
        self._is_controlling = False  # Re-capture references on resume
        self.state = self.STATE_RUNNING

    def emergency_stop(self):
        """Emergency stop — halt everything immediately / 紧急停止"""
        self._stop_event.set()
        self._is_controlling = False

        if self._robot:
            try:
                self._robot.stop_move()
            except Exception:
                pass
            try:
                self._robot.estop()
            except Exception:
                pass

        self.state = self.STATE_EMERGENCY_STOP
        logger.warning("EMERGENCY STOP triggered")

    def reset_from_emergency(self) -> bool:
        """Reset after emergency stop / 紧急停止后复位"""
        if self._robot:
            try:
                self._robot.start_sys()
                time.sleep(0.5)
                self.state = self.STATE_SDK_CONNECTED
                logger.info("Reset from emergency stop")
                return True
            except Exception as e:
                self._error_message = f"Reset failed: {e}"
                logger.error(self._error_message)
                return False
        return False

    # ======================================================================
    # Runtime Parameter Setters
    # ======================================================================

    def set_scale_factor(self, f):
        self._scale_factor = float(f)

    def set_max_linear_speed(self, v):
        self._max_linear_speed = float(v)

    def set_control_hz(self, hz):
        self._control_hz = int(hz)

    def set_workspace_limits(self, limits):
        self._workspace_limits.update(limits)

    def set_frame_rotation(self, R):
        self._R_vr_to_robot = np.array(R, dtype=np.float64).reshape(3, 3)

    # ======================================================================
    # VR Preview Mode (reads VR, computes simulated deltas, no robot needed)
    # ======================================================================

    def _start_preview(self):
        """Start VR preview polling thread / 启动VR预览轮询线程"""
        if self._preview_active:
            return
        self._preview_stop.clear()
        self._preview_thread = threading.Thread(
            target=self._preview_loop, daemon=True, name="VRPreview"
        )
        self._preview_thread.start()
        self._preview_active = True
        logger.info("VR preview started")

    def _stop_preview(self):
        """Stop VR preview polling thread / 停止VR预览轮询线程"""
        self._preview_stop.set()
        if self._preview_thread and self._preview_thread.is_alive():
            self._preview_thread.join(timeout=1.0)
        self._preview_thread = None
        self._preview_active = False

    def _preview_loop(self):
        """
        VR preview loop — reads VR data at 20Hz, computes what the robot
        target TCP would be. Runs independently of the teleop control loop.
        VR预览循环 — 20Hz读取VR数据，计算模拟的机器人目标TCP。
        """
        period = 1.0 / 20.0  # 20Hz for preview
        sim_init_xyz = None
        sim_init_quat = None
        # Use a fake "home" TCP as the simulated robot starting pose
        sim_init_tcp = [0.0, -0.3, 0.4, 0.0, 0.0, 0.0]  # Typical Lebai home-ish pose

        while not self._preview_stop.is_set():
            # Skip if the real control loop is running (it handles everything)
            if self._state == self.STATE_RUNNING:
                self._preview_stop.wait(0.1)
                continue

            t0 = time.perf_counter()
            try:
                # Read VR inputs
                grip_val = xrt.get_right_grip()
                trigger_val = xrt.get_right_trigger()
                left_trigger_val = xrt.get_left_trigger()
                left_grip_val = xrt.get_left_grip()
                vr_pose = xrt.get_right_controller_pose()

                self._last_grip_val = grip_val
                self._last_trigger_val = trigger_val
                self._last_left_trigger_val = left_trigger_val
                self._last_left_grip_val = left_grip_val
                self._last_vr_pose = list(vr_pose) if vr_pose is not None else [0]*7

                # Track whether we're actually receiving valid VR data
                if vr_pose is not None and any(v != 0.0 for v in vr_pose):
                    self._last_vr_data_time = time.time()
                    self._pico_streaming = True
                elif time.time() - self._last_vr_data_time > self._pico_data_timeout:
                    self._pico_streaming = False

                # Read buttons
                try:
                    self._a_button = xrt.get_A_button()
                    self._b_button = xrt.get_B_button()
                    self._x_button = xrt.get_X_button()
                    self._y_button = xrt.get_Y_button()
                except Exception:
                    pass

                # Simulate gripper amplitude (incremental: trigger=close, grip=open)
                gripper_speed_per_sec = 150.0
                if left_trigger_val > 0.1:
                    self._last_gripper_amplitude = max(0.0, self._last_gripper_amplitude - left_trigger_val * gripper_speed_per_sec * period)
                elif left_grip_val > 0.1:
                    self._last_gripper_amplitude = min(100.0, self._last_gripper_amplitude + left_grip_val * gripper_speed_per_sec * period)

                # Simulated suction toggle
                if self._a_button and not self._prev_a_button:
                    self._suction_on = not self._suction_on
                self._prev_a_button = self._a_button

                if vr_pose is not None:
                    # Store raw quaternion for display
                    self._vr_raw_quat = [vr_pose[6], vr_pose[3], vr_pose[4], vr_pose[5]]

                    # Transform to robot frame
                    vr_xyz = self._R_vr_to_robot @ np.array(vr_pose[:3])
                    vr_quat_wxyz = np.array([vr_pose[6], vr_pose[3], vr_pose[4], vr_pose[5]])
                    vr_quat_wxyz = rotate_quaternion(vr_quat_wxyz, self._R_vr_to_robot)

                    active = grip_val > (1.0 - self._deadzone)

                    if active:
                        if sim_init_xyz is None:
                            sim_init_xyz = vr_xyz.copy()
                            sim_init_quat = vr_quat_wxyz.copy()
                            self._is_controlling = True
                        else:
                            delta_xyz = (vr_xyz - sim_init_xyz) * self._scale_factor
                            delta_rot = quat_diff_to_rotvec(sim_init_quat, vr_quat_wxyz)

                            self._vr_delta_xyz = delta_xyz.tolist()
                            self._vr_delta_rot = delta_rot.tolist()

                            self._simulated_tcp = [
                                sim_init_tcp[0] + delta_xyz[0],
                                sim_init_tcp[1] + delta_xyz[1],
                                sim_init_tcp[2] + delta_xyz[2],
                                sim_init_tcp[3] + delta_rot[0],
                                sim_init_tcp[4] + delta_rot[1],
                                sim_init_tcp[5] + delta_rot[2],
                            ]
                            self._last_target_tcp = list(self._simulated_tcp)
                    else:
                        if sim_init_xyz is not None:
                            sim_init_xyz = None
                            sim_init_quat = None
                            self._is_controlling = False
                            self._vr_delta_xyz = [0.0, 0.0, 0.0]
                            self._vr_delta_rot = [0.0, 0.0, 0.0]

            except Exception as e:
                logger.debug(f"Preview loop error: {e}")

            elapsed = time.perf_counter() - t0
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ======================================================================
    # Control Loop
    # ======================================================================

    def _control_loop(self):
        """Main teleoperation control loop / 主遥操作控制循环"""
        prev_tcp = list(self._cached_tcp_position)
        prev_loop_time = time.perf_counter()

        while not self._stop_event.is_set():
            period = 1.0 / self._control_hz  # Re-read each tick so Hz slider takes effect live
            t0 = time.perf_counter()
            # Use actual elapsed time for accurate motion timing
            actual_dt = max(0.002, min(t0 - prev_loop_time, 0.1))
            prev_loop_time = t0

            if self._paused:
                time.sleep(period)
                prev_loop_time = time.perf_counter()
                continue

            try:
                # Read VR inputs
                grip_val = xrt.get_right_grip()
                trigger_val = xrt.get_right_trigger()
                left_trigger_val = xrt.get_left_trigger()
                left_grip_val = xrt.get_left_grip()
                vr_pose = xrt.get_right_controller_pose()
                # vr_pose = [x, y, z, qx, qy, qz, qw]

                self._last_grip_val = grip_val
                self._last_trigger_val = trigger_val
                self._last_left_trigger_val = left_trigger_val
                self._last_left_grip_val = left_grip_val
                self._last_vr_pose = list(vr_pose) if vr_pose is not None else [0]*7

                # Gripper control (left: trigger=close, grip=open)
                self._gripper_dt = actual_dt
                self._update_gripper(left_trigger_val, left_grip_val)

                # Suction toggle (A button)
                self._update_suction()

                # Emergency stop check (Y button)
                self._check_emergency_button()
                if self._stop_event.is_set():
                    break

                # Arm control (right trigger, top button) — hysteresis to prevent flicker
                if self._is_controlling:
                    active = trigger_val > 0.3  # release threshold (low)
                else:
                    active = trigger_val > 0.7  # activate threshold (high)

                if active and vr_pose is not None:
                    # Transform VR pose to robot coordinate frame
                    vr_xyz = self._R_vr_to_robot @ np.array(vr_pose[:3])
                    vr_quat_wxyz = np.array([vr_pose[6], vr_pose[3], vr_pose[4], vr_pose[5]])
                    vr_quat_wxyz = rotate_quaternion(vr_quat_wxyz, self._R_vr_to_robot)

                    if not self._is_controlling:
                        # First frame: compute offset between controller and robot TCP
                        # so that controller position maps directly to robot tip
                        self._update_robot_cache()
                        robot_xyz = np.array(self._cached_tcp_position[:3])
                        robot_rot = np.array(self._cached_tcp_position[3:6])
                        # offset = robot_tcp - controller_pos (scaled)
                        self._offset_xyz = robot_xyz - vr_xyz * self._scale_factor
                        # Save initial quaternions for proper orientation composition
                        self._init_vr_quat = vr_quat_wxyz.copy()
                        self._init_robot_quat = rotvec_to_quat_wxyz(robot_rot)
                        # Initialize EMA smoothing state
                        self._smooth_xyz = vr_xyz.copy()
                        self._smooth_quat = vr_quat_wxyz.copy()
                        prev_tcp = list(self._cached_tcp_position)
                        self._prev_joint_angles = None
                        self._is_controlling = True
                        logger.info("Teleop engaged — controller synced to robot tip")
                    else:
                        # Apply EMA smoothing to filter VR tracking noise
                        alpha = self._ema_alpha
                        self._smooth_xyz = (1.0 - alpha) * self._smooth_xyz + alpha * vr_xyz
                        self._smooth_quat = quat_nlerp(self._smooth_quat, vr_quat_wxyz, alpha)
                        vr_xyz = self._smooth_xyz
                        vr_quat_wxyz = self._smooth_quat

                        # Position: direct mapping with scale and offset
                        mapped_xyz = vr_xyz * self._scale_factor + self._offset_xyz

                        # Orientation: proper quaternion composition
                        # delta = current * inverse(initial) → rotation from init to now
                        q_vr_init_inv = np.array([
                            self._init_vr_quat[0],
                            -self._init_vr_quat[1],
                            -self._init_vr_quat[2],
                            -self._init_vr_quat[3],
                        ])
                        q_delta = quat_multiply(vr_quat_wxyz, q_vr_init_inv)
                        q_target = quat_multiply(q_delta, self._init_robot_quat)
                        mapped_rot = quat_diff_to_rotvec(
                            np.array([1, 0, 0, 0]), q_target)

                        target_tcp = [
                            mapped_xyz[0], mapped_xyz[1], mapped_xyz[2],
                            mapped_rot[0], mapped_rot[1], mapped_rot[2],
                        ]

                        # Safety: workspace clamp
                        target_tcp = clamp_tcp_to_workspace(target_tcp, self._workspace_limits)

                        # Safety: speed clamp (use actual elapsed time)
                        target_tcp = clamp_speed(
                            prev_tcp, target_tcp, actual_dt,
                            self._max_linear_speed, self._max_angular_speed
                        )

                        # IK → joint angles, then stream via move_pvt
                        pose_dict = {
                            'x': target_tcp[0], 'y': target_tcp[1], 'z': target_tcp[2],
                            'rx': target_tcp[3], 'ry': target_tcp[4], 'rz': target_tcp[5],
                        }
                        try:
                            # Try IK with reference joints for solution continuity
                            if self._ik_supports_ref and self._prev_joint_angles is not None:
                                try:
                                    joint_angles = self._robot.kinematics_inverse(
                                        pose_dict, self._prev_joint_angles)
                                except TypeError:
                                    # SDK doesn't support reference parameter
                                    self._ik_supports_ref = False
                                    joint_angles = self._robot.kinematics_inverse(pose_dict)
                            else:
                                joint_angles = self._robot.kinematics_inverse(pose_dict)

                            # Joint limit protection: ±720° = ±12.566 rad
                            JOINT_LIMIT = 12.4  # ~710°, leave margin before 720°
                            if self._prev_joint_angles is not None:
                                clamped = list(joint_angles)
                                for i in range(len(clamped)):
                                    prev = self._prev_joint_angles[i]
                                    curr = clamped[i]
                                    if curr > JOINT_LIMIT and curr > prev:
                                        clamped[i] = JOINT_LIMIT
                                    elif curr < -JOINT_LIMIT and curr < prev:
                                        clamped[i] = -JOINT_LIMIT
                                joint_angles = clamped

                            # Compute joint velocities (use actual elapsed time)
                            if self._prev_joint_angles is not None:
                                joint_vel = [
                                    (joint_angles[i] - self._prev_joint_angles[i]) / actual_dt
                                    for i in range(len(joint_angles))
                                ]
                            else:
                                joint_vel = [0.0] * len(joint_angles)
                            self._prev_joint_angles = list(joint_angles)
                        except Exception as e:
                            logger.warning(f"IK failed: {e}")
                            joint_angles = None

                        if joint_angles is not None:
                            try:
                                # Use actual elapsed time so robot interpolation matches reality
                                self._robot.move_pvt(joint_angles, joint_vel, actual_dt)
                            except Exception as e:
                                try:
                                    self._robot.towardj(joint_angles, self._acceleration, self._max_linear_speed)
                                except Exception as e2:
                                    logger.warning(f"motion failed: {e2}")

                        self._last_target_tcp = target_tcp
                        prev_tcp = target_tcp
                else:
                    if self._is_controlling:
                        # Released trigger — stop motion
                        try:
                            self._robot.stop_move()
                        except Exception:
                            pass
                        self._is_controlling = False
                        self._offset_xyz = None
                        self._init_vr_quat = None
                        self._init_robot_quat = None
                        self._smooth_xyz = None
                        self._smooth_quat = None
                        self._prev_joint_angles = None
                        logger.info("Teleop disengaged")

                # Only update robot cache when not actively controlling
                # to avoid blocking the control loop with network calls
                if not self._is_controlling:
                    self._update_robot_cache()

            except Exception as e:
                logger.error(f"Control loop error: {e}")
                # VR tracking lost or SDK error — enter safe state
                if self._is_controlling:
                    try:
                        self._robot.stop_move()
                    except Exception:
                        pass
                    self._is_controlling = False

            # Maintain loop rate
            elapsed = time.perf_counter() - t0
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ======================================================================
    # Gripper / Suction / Emergency
    # ======================================================================

    def _update_gripper(self, left_trigger_val, left_grip_val):
        """Map left controller to Lebai claw / 左手控制夹爪
        Left trigger (top, index finger) = close gripper incrementally
        Left grip (side, squeeze) = open gripper incrementally
        Harder press = faster change
        """
        current = self._gripper_target
        gripper_speed_per_sec = 150.0  # max %/second at full press

        if left_trigger_val > 0.1:
            # Close: decrement proportional to press pressure and actual time
            self._gripper_target = current - left_trigger_val * gripper_speed_per_sec * self._gripper_dt
        elif left_grip_val > 0.1:
            # Open: increment proportional to press pressure and actual time
            self._gripper_target = current + left_grip_val * gripper_speed_per_sec * self._gripper_dt
        else:
            return  # neither pressed, don't change
        self._gripper_target = max(0.0, min(100.0, self._gripper_target))

        # Only send command when accumulated change exceeds deadband
        # AND rate-limit to avoid flooding the robot with commands
        now = time.time()
        if (abs(self._gripper_target - self._last_gripper_amplitude) >= self._gripper_deadband
                and now - self._last_gripper_cmd_time >= 0.05):  # max 20 commands/sec
            try:
                self._robot.set_claw(amplitude=self._gripper_target, force=self._gripper_force)
                self._last_gripper_amplitude = self._gripper_target
                self._last_gripper_cmd_time = now
            except Exception as e:
                logger.warning(f"set_claw failed: {e}")

    def _update_suction(self):
        """Toggle suction on A-button press / A按钮切换吸盘"""
        try:
            a_pressed = xrt.get_A_button()
        except Exception:
            return

        if a_pressed and not self._prev_a_button:
            self._suction_on = not self._suction_on
            try:
                if self._suction_on:
                    self._robot.set_do("FLANGE", 0, 1)
                else:
                    self._robot.set_do("FLANGE", 0, 0)
                logger.info(f"Suction {'ON' if self._suction_on else 'OFF'}")
            except Exception as e:
                logger.warning(f"Suction toggle failed: {e}")

        self._prev_a_button = a_pressed

    def _check_emergency_button(self):
        """Y button triggers emergency stop / Y按钮触发紧急停止"""
        try:
            y_pressed = xrt.get_Y_button()
        except Exception:
            return

        if y_pressed and not self._prev_y_button:
            logger.warning("VR Y-button emergency stop triggered")
            self.emergency_stop()

        self._prev_y_button = y_pressed

    # ======================================================================
    # Robot Cache
    # ======================================================================

    def _update_robot_cache(self):
        """Update cached robot positions from SDK / 从SDK更新缓存的机器人位置"""
        if not self._robot:
            return
        try:
            kin = self._robot.get_kin_data()
            if kin:
                self._cached_joint_positions = list(kin.get('actual_joint_pose', [0]*6))
                tcp = kin.get('actual_tcp_pose', {})
                self._cached_tcp_position = [
                    tcp.get('x', 0), tcp.get('y', 0), tcp.get('z', 0),
                    tcp.get('rx', 0), tcp.get('ry', 0), tcp.get('rz', 0),
                ]
        except Exception:
            pass

    # ======================================================================
    # Status / Recording
    # ======================================================================

    def get_status(self) -> dict:
        """Get full controller status / 获取完整控制器状态"""
        return {
            "state": self.state,
            "error": self._error_message,
            "xrt_connected": self._xrt_connected,
            "pico_streaming": self._pico_streaming,
            "xrt_available": _xrt_available,
            "lebai_connected": self._lebai_connected,
            "lebai_available": _lebai_available,
            "is_controlling": self._is_controlling,
            "preview_active": self._preview_active,
            "vr_pose": self._last_vr_pose,
            "vr_raw_quat": list(self._vr_raw_quat),
            "vr_delta_xyz": list(self._vr_delta_xyz),
            "vr_delta_rot": list(self._vr_delta_rot),
            "grip": self._last_grip_val,
            "trigger": self._last_trigger_val,
            "left_trigger": self._last_left_trigger_val,
            "left_grip": self._last_left_grip_val,
            "a_button": self._a_button,
            "b_button": self._b_button,
            "x_button": self._x_button,
            "y_button": self._y_button,
            "robot_tcp": self._cached_tcp_position,
            "robot_joints": self._cached_joint_positions,
            "target_tcp": self._last_target_tcp,
            "simulated_tcp": list(self._simulated_tcp),
            "gripper_amplitude": self._last_gripper_amplitude,
            "suction_on": self._suction_on,
            "scale_factor": self._scale_factor,
            "max_linear_speed": self._max_linear_speed,
            "control_hz": self._control_hz,
        }

    def get_state_for_recording(self) -> dict:
        """Get state snapshot for data recording / 获取用于数据记录的状态快照"""
        return {
            "timestamp": time.time(),
            "vr_pose": list(self._last_vr_pose),
            "vr_grip": self._last_grip_val,
            "vr_trigger": self._last_trigger_val,
            "robot_joints": list(self._cached_joint_positions),
            "robot_tcp": list(self._cached_tcp_position),
            "target_tcp": list(self._last_target_tcp),
            "gripper_amplitude": self._last_gripper_amplitude,
            "suction_on": self._suction_on,
            "is_controlling": self._is_controlling,
        }
