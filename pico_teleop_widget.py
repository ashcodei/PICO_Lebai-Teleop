"""
PICO VR Teleoperation Widget (Tkinter)

GUI for controlling and monitoring VR-to-robot teleoperation.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import logging
import numpy as np

from pico_teleop_controller import PicoTeleopController
from frame_utils import compute_calibration_matrix

logger = logging.getLogger(__name__)


class PicoTeleopWidget(ttk.Frame):
    """PICO VR Teleoperation Widget."""

    def __init__(self, parent, controller: PicoTeleopController):
        super().__init__(parent, padding=0)
        self._controller = controller
        self._calibration_step = 0
        self._calibration_points = [None, None, None]
        self._cal_prev_a = False  # Track A button state for calibration capture

        self._build_scrollable()
        self._build_ui()
        self._update_display()

    def _build_scrollable(self):
        """Set up a scrollable canvas inside this frame."""
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        self._canvas = tk.Canvas(self, highlightthickness=0)
        self._scrollbar = ttk.Scrollbar(self, orient="vertical", command=self._canvas.yview)
        self._canvas.configure(yscrollcommand=self._scrollbar.set)

        self._scrollbar.grid(row=0, column=1, sticky="ns")
        self._canvas.grid(row=0, column=0, sticky="nsew")

        self._inner = ttk.Frame(self._canvas, padding=10)
        self._canvas_window = self._canvas.create_window((0, 0), window=self._inner, anchor="nw")

        self._inner.bind("<Configure>", self._on_inner_configure)
        self._canvas.bind("<Configure>", self._on_canvas_configure)

        # Mouse wheel scrolling
        self._canvas.bind("<Enter>", lambda e: self._bind_mousewheel())
        self._canvas.bind("<Leave>", lambda e: self._unbind_mousewheel())

    def _on_inner_configure(self, event):
        self._canvas.configure(scrollregion=self._canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        self._canvas.itemconfig(self._canvas_window, width=event.width)

    def _bind_mousewheel(self):
        self._canvas.bind_all("<Button-4>", self._on_mousewheel_up)
        self._canvas.bind_all("<Button-5>", self._on_mousewheel_down)
        self._canvas.bind_all("<MouseWheel>", self._on_mousewheel)

    def _unbind_mousewheel(self):
        self._canvas.unbind_all("<Button-4>")
        self._canvas.unbind_all("<Button-5>")
        self._canvas.unbind_all("<MouseWheel>")

    def _on_mousewheel_up(self, event):
        self._canvas.yview_scroll(-3, "units")

    def _on_mousewheel_down(self, event):
        self._canvas.yview_scroll(3, "units")

    def _on_mousewheel(self, event):
        self._canvas.yview_scroll(-1 * (event.delta // 120), "units")

    def _build_ui(self):
        f = self._inner
        f.columnconfigure(0, weight=1)

        row = 0

        # -- Status Section --
        status_frame = ttk.LabelFrame(f, text="Status", padding=8)
        status_frame.grid(row=row, column=0, sticky="ew", pady=(0, 6))
        status_frame.columnconfigure(1, weight=1)

        ttk.Label(status_frame, text="XRT SDK:").grid(row=0, column=0, sticky="w", padx=(0, 8))
        self._lbl_xrt_status = ttk.Label(status_frame, text="--", foreground="gray")
        self._lbl_xrt_status.grid(row=0, column=1, sticky="w")

        ttk.Label(status_frame, text="Lebai:").grid(row=1, column=0, sticky="w", padx=(0, 8))
        self._lbl_lebai_status = ttk.Label(status_frame, text="--", foreground="gray")
        self._lbl_lebai_status.grid(row=1, column=1, sticky="w")

        ttk.Label(status_frame, text="Teleop:").grid(row=2, column=0, sticky="w", padx=(0, 8))
        self._lbl_teleop_status = ttk.Label(status_frame, text="--", foreground="gray")
        self._lbl_teleop_status.grid(row=2, column=1, sticky="w")

        ttk.Label(status_frame, text="Controlling:").grid(row=3, column=0, sticky="w", padx=(0, 8))
        self._lbl_controlling = ttk.Label(status_frame, text="No", foreground="gray")
        self._lbl_controlling.grid(row=3, column=1, sticky="w")

        row += 1

        # -- Connection Controls --
        conn_frame = ttk.LabelFrame(f, text="Connection", padding=8)
        conn_frame.grid(row=row, column=0, sticky="ew", pady=(0, 6))

        btn_row1 = ttk.Frame(conn_frame)
        btn_row1.pack(fill="x", pady=(0, 4))
        self._btn_connect_xrt = ttk.Button(btn_row1, text="Connect XRT SDK",
                                            command=self._on_connect_xrt)
        self._btn_connect_xrt.pack(side="left", padx=(0, 4))
        self._btn_disconnect_xrt = ttk.Button(btn_row1, text="Disconnect XRT",
                                               command=self._on_disconnect_xrt)
        self._btn_disconnect_xrt.pack(side="left", padx=(0, 4))

        btn_row2 = ttk.Frame(conn_frame)
        btn_row2.pack(fill="x", pady=(0, 4))
        self._btn_connect_lebai = ttk.Button(btn_row2, text="Connect Lebai",
                                              command=self._on_connect_lebai)
        self._btn_connect_lebai.pack(side="left", padx=(0, 4))
        self._btn_disconnect_lebai = ttk.Button(btn_row2, text="Disconnect Lebai",
                                                 command=self._on_disconnect_lebai)
        self._btn_disconnect_lebai.pack(side="left", padx=(0, 4))

        ip_frame = ttk.Frame(conn_frame)
        ip_frame.pack(fill="x", pady=(0, 4))
        ttk.Label(ip_frame, text="Lebai IP:").pack(side="left", padx=(0, 4))
        self._entry_ip = ttk.Entry(ip_frame, width=16)
        self._entry_ip.insert(0, self._controller._lebai_ip)
        self._entry_ip.pack(side="left", padx=(0, 4))

        # Quick IP preset for Lebai WiFi
        self._btn_ip_wifi = ttk.Button(ip_frame, text="Lebai WiFi (10.20.17.1)",
                                        command=lambda: self._set_ip("10.20.17.1"))
        self._btn_ip_wifi.pack(side="left", padx=(0, 4))

        net_frame = ttk.Frame(conn_frame)
        net_frame.pack(fill="x")
        self._lbl_net_info = ttk.Label(net_frame,
            text="Lebai connection: WiFi (10.20.17.1)  |  PICO: USB (ADB)",
            font=("Consolas", 9), foreground="gray")
        self._lbl_net_info.pack(side="left")

        row += 1

        # -- Teleop Controls --
        teleop_frame = ttk.LabelFrame(f, text="Teleop Controls", padding=8)
        teleop_frame.grid(row=row, column=0, sticky="ew", pady=(0, 6))

        btn_row3 = ttk.Frame(teleop_frame)
        btn_row3.pack(fill="x", pady=(0, 4))
        self._btn_start = ttk.Button(btn_row3, text="Start Teleop", command=self._on_start)
        self._btn_start.pack(side="left", padx=(0, 4))
        self._btn_stop = ttk.Button(btn_row3, text="Stop Teleop", command=self._on_stop)
        self._btn_stop.pack(side="left", padx=(0, 4))
        self._btn_pause = ttk.Button(btn_row3, text="Pause", command=self._on_pause)
        self._btn_pause.pack(side="left", padx=(0, 4))
        self._btn_resume = ttk.Button(btn_row3, text="Resume", command=self._on_resume)
        self._btn_resume.pack(side="left", padx=(0, 4))

        btn_row4 = ttk.Frame(teleop_frame)
        btn_row4.pack(fill="x")
        self._btn_estop = ttk.Button(btn_row4, text="EMERGENCY STOP",
                                      command=self._on_emergency_stop,
                                      style="Emergency.TButton")
        self._btn_estop.pack(side="left", padx=(0, 4))
        self._btn_reset = ttk.Button(btn_row4, text="Reset E-Stop",
                                      command=self._on_reset_estop)
        self._btn_reset.pack(side="left")

        row += 1

        # -- Parameters --
        param_frame = ttk.LabelFrame(f, text="Parameters", padding=8)
        param_frame.grid(row=row, column=0, sticky="ew", pady=(0, 6))
        param_frame.columnconfigure(1, weight=1)

        ttk.Label(param_frame, text="Scale Factor:").grid(row=0, column=0, sticky="w")
        self._scale_var = tk.DoubleVar(value=self._controller.scale_factor)
        self._scale_slider = ttk.Scale(param_frame, from_=0.1, to=3.0,
                                        variable=self._scale_var, orient="horizontal",
                                        command=self._on_scale_change)
        self._scale_slider.grid(row=0, column=1, sticky="ew", padx=4)
        self._lbl_scale = ttk.Label(param_frame, text=f"{self._controller.scale_factor:.2f}", width=6)
        self._lbl_scale.grid(row=0, column=2)

        ttk.Label(param_frame, text="Max Speed (m/s):").grid(row=1, column=0, sticky="w")
        self._speed_var = tk.DoubleVar(value=self._controller.max_linear_speed)
        self._speed_slider = ttk.Scale(param_frame, from_=0.05, to=1.0,
                                        variable=self._speed_var, orient="horizontal",
                                        command=self._on_speed_change)
        self._speed_slider.grid(row=1, column=1, sticky="ew", padx=4)
        self._lbl_speed = ttk.Label(param_frame, text=f"{self._controller.max_linear_speed:.2f}", width=6)
        self._lbl_speed.grid(row=1, column=2)

        ttk.Label(param_frame, text="Control Hz:").grid(row=2, column=0, sticky="w")
        self._hz_var = tk.IntVar(value=self._controller.control_hz)
        self._hz_slider = ttk.Scale(param_frame, from_=10, to=100,
                                     variable=self._hz_var, orient="horizontal",
                                     command=self._on_hz_change)
        self._hz_slider.grid(row=2, column=1, sticky="ew", padx=4)
        self._lbl_hz = ttk.Label(param_frame, text=str(self._controller.control_hz), width=6)
        self._lbl_hz.grid(row=2, column=2)

        row += 1

        # -- VR Live Data (raw from headset) --
        vr_frame = ttk.LabelFrame(f, text="VR Right Controller (Raw)", padding=8)
        vr_frame.grid(row=row, column=0, sticky="ew", pady=(0, 6))
        vr_frame.columnconfigure(1, weight=1)

        MONO = ("Consolas", 9)

        ttk.Label(vr_frame, text="Controller:").grid(row=0, column=0, sticky="w")
        self._lbl_controller_info = ttk.Label(vr_frame,
            text="R-Trigger = move arm  |  L-Trigger = close gripper  |  L-Grip = open gripper  |  Y = E-stop",
            font=MONO, foreground="blue")
        self._lbl_controller_info.grid(row=0, column=1, sticky="w", padx=4)

        ttk.Label(vr_frame, text="Position:").grid(row=1, column=0, sticky="w")
        self._lbl_vr_pos = ttk.Label(vr_frame, text="x=--  y=--  z=--", font=MONO)
        self._lbl_vr_pos.grid(row=1, column=1, sticky="w", padx=4)

        ttk.Label(vr_frame, text="Orientation:").grid(row=2, column=0, sticky="w")
        self._lbl_vr_quat = ttk.Label(vr_frame, text="w=--  x=--  y=--  z=--", font=MONO)
        self._lbl_vr_quat.grid(row=2, column=1, sticky="w", padx=4)

        ttk.Label(vr_frame, text="Grip:").grid(row=3, column=0, sticky="w")
        self._grip_bar = ttk.Progressbar(vr_frame, length=120, mode='determinate', maximum=100)
        self._grip_bar.grid(row=3, column=1, sticky="w", padx=4)

        ttk.Label(vr_frame, text="R-Trigger:").grid(row=4, column=0, sticky="w")
        self._trigger_bar = ttk.Progressbar(vr_frame, length=120, mode='determinate', maximum=100)
        self._trigger_bar.grid(row=4, column=1, sticky="w", padx=4)

        ttk.Label(vr_frame, text="L-Trigger:").grid(row=5, column=0, sticky="w")
        self._left_trigger_bar = ttk.Progressbar(vr_frame, length=120, mode='determinate', maximum=100)
        self._left_trigger_bar.grid(row=5, column=1, sticky="w", padx=4)

        ttk.Label(vr_frame, text="L-Grip:").grid(row=6, column=0, sticky="w")
        self._left_grip_bar = ttk.Progressbar(vr_frame, length=120, mode='determinate', maximum=100)
        self._left_grip_bar.grid(row=6, column=1, sticky="w", padx=4)

        ttk.Label(vr_frame, text="Buttons:").grid(row=7, column=0, sticky="w")
        self._lbl_buttons = ttk.Label(vr_frame, text="A:- B:- X:- Y:-", font=MONO)
        self._lbl_buttons.grid(row=7, column=1, sticky="w", padx=4)

        row += 1

        # -- Translated Motion (robot frame) --
        motion_frame = ttk.LabelFrame(f, text="Translated Motion (Robot Frame)", padding=8)
        motion_frame.grid(row=row, column=0, sticky="ew", pady=(0, 6))
        motion_frame.columnconfigure(1, weight=1)

        ttk.Label(motion_frame, text="Mapping:").grid(row=0, column=0, sticky="w")
        self._lbl_mapping_info = ttk.Label(motion_frame,
            text="VR(X,Y,Z) -> Robot(-Z,-X,Y)  |  Scale: 1.00x",
            font=MONO, foreground="blue")
        self._lbl_mapping_info.grid(row=0, column=1, sticky="w", padx=4)

        ttk.Label(motion_frame, text="Delta XYZ:").grid(row=1, column=0, sticky="w")
        self._lbl_delta_xyz = ttk.Label(motion_frame, text="dx=--  dy=--  dz=--", font=MONO)
        self._lbl_delta_xyz.grid(row=1, column=1, sticky="w", padx=4)

        ttk.Label(motion_frame, text="Delta Rot:").grid(row=2, column=0, sticky="w")
        self._lbl_delta_rot = ttk.Label(motion_frame, text="rx=--  ry=--  rz=--", font=MONO)
        self._lbl_delta_rot.grid(row=2, column=1, sticky="w", padx=4)

        ttk.Label(motion_frame, text="Sim. Target TCP:").grid(row=3, column=0, sticky="w")
        self._lbl_sim_tcp = ttk.Label(motion_frame, text="x=--  y=--  z=--  rx=--  ry=--  rz=--", font=MONO)
        self._lbl_sim_tcp.grid(row=3, column=1, sticky="w", padx=4)

        row += 1

        # -- Robot TCP & Joints --
        robot_frame = ttk.LabelFrame(f, text="Robot State (Lebai LM3 - 6 DOF)", padding=8)
        robot_frame.grid(row=row, column=0, sticky="ew", pady=(0, 6))
        robot_frame.columnconfigure(1, weight=1)

        ttk.Label(robot_frame, text="Actual TCP:").grid(row=0, column=0, sticky="w")
        self._lbl_robot_tcp = ttk.Label(robot_frame, text="x=--  y=--  z=--", font=MONO)
        self._lbl_robot_tcp.grid(row=0, column=1, sticky="w", padx=4)

        ttk.Label(robot_frame, text="Target TCP:").grid(row=1, column=0, sticky="w")
        self._lbl_target_tcp = ttk.Label(robot_frame, text="x=--  y=--  z=--  rx=--  ry=--  rz=--", font=MONO)
        self._lbl_target_tcp.grid(row=1, column=1, sticky="w", padx=4)

        # Joint angles for all 6 joints
        joint_names = ["J1 (Base)", "J2 (Shoulder)", "J3 (Elbow)",
                       "J4 (Wrist 1)", "J5 (Wrist 2)", "J6 (Wrist 3)"]
        self._lbl_joints = []
        self._joint_bars = []
        for i, name in enumerate(joint_names):
            r = 2 + i
            ttk.Label(robot_frame, text=f"{name}:").grid(row=r, column=0, sticky="w")
            jf = ttk.Frame(robot_frame)
            jf.grid(row=r, column=1, sticky="ew", padx=4)
            bar = ttk.Progressbar(jf, length=100, mode='determinate', maximum=360)
            bar.pack(side="left")
            lbl = ttk.Label(jf, text="  0.00 deg", font=MONO, width=12)
            lbl.pack(side="left", padx=4)
            self._joint_bars.append(bar)
            self._lbl_joints.append(lbl)

        ttk.Label(robot_frame, text="Source:").grid(row=8, column=0, sticky="w")
        self._lbl_joint_source = ttk.Label(robot_frame,
            text="(Lebai not connected - showing simulated target TCP, joints require IK from robot)",
            font=MONO, foreground="orange")
        self._lbl_joint_source.grid(row=8, column=1, sticky="w", padx=4)

        row += 1

        # -- End Effector State --
        ee_frame = ttk.LabelFrame(f, text="End Effector", padding=8)
        ee_frame.grid(row=row, column=0, sticky="ew", pady=(0, 6))
        ee_frame.columnconfigure(1, weight=1)

        ttk.Label(ee_frame, text="Gripper:").grid(row=0, column=0, sticky="w")
        gripper_bar_frame = ttk.Frame(ee_frame)
        gripper_bar_frame.grid(row=0, column=1, sticky="ew", padx=4)
        self._gripper_bar = ttk.Progressbar(gripper_bar_frame, length=150, mode='determinate', maximum=100)
        self._gripper_bar.pack(side="left")
        self._lbl_gripper = ttk.Label(gripper_bar_frame, text="100% (open)", width=14)
        self._lbl_gripper.pack(side="left", padx=4)
        self._btn_gripper_zero = ttk.Button(gripper_bar_frame, text="Close (0%)",
                                             command=self._on_gripper_zero, width=10)
        self._btn_gripper_zero.pack(side="left", padx=2)

        ttk.Label(ee_frame, text="Suction:").grid(row=1, column=0, sticky="w")
        self._lbl_suction = ttk.Label(ee_frame, text="OFF", foreground="gray", font=MONO)
        self._lbl_suction.grid(row=1, column=1, sticky="w", padx=4)

        row += 1

        # -- Calibration --
        cal_frame = ttk.LabelFrame(f, text="Calibration (VR-to-Robot Axis Mapping)", padding=8)
        cal_frame.grid(row=row, column=0, sticky="ew", pady=(0, 6))

        cal_btn_row = ttk.Frame(cal_frame)
        cal_btn_row.pack(fill="x", pady=(0, 4))
        self._btn_calibrate = ttk.Button(cal_btn_row, text="Start Calibration",
                                          command=self._on_calibrate)
        self._btn_calibrate.pack(side="left", padx=(0, 4))
        self._btn_cal_reset = ttk.Button(cal_btn_row, text="Reset to Default",
                                          command=self._on_calibrate_reset)
        self._btn_cal_reset.pack(side="left", padx=(0, 4))
        self._btn_cal_cancel = ttk.Button(cal_btn_row, text="Cancel",
                                           command=self._on_calibrate_cancel,
                                           state="disabled")
        self._btn_cal_cancel.pack(side="left")

        self._lbl_cal_hint = ttk.Label(cal_frame,
            text="Aligns VR controller axes to robot axes. "
                 "Stand next to the robot so you can see its arm. Press [A] button on VR controller to capture points.",
            wraplength=500, justify="left")
        self._lbl_cal_hint.pack(fill="x")

    # ======================================================================
    # Button Handlers
    # ======================================================================

    def _set_ip(self, ip):
        self._entry_ip.delete(0, tk.END)
        self._entry_ip.insert(0, ip)

    def _on_gripper_zero(self):
        """Set gripper to 0% (fully closed)"""
        self._controller._last_gripper_amplitude = 0.0
        self._controller._gripper_target = 0.0
        try:
            if self._controller._lebai_connected and self._controller._robot:
                self._controller._robot.set_claw(amplitude=0.0, force=self._controller._gripper_force)
        except Exception as e:
            logger.warning(f"set_claw failed: {e}")

    def _on_connect_xrt(self):
        def _do():
            ok = self._controller.connect_xrt()
            if not ok:
                self.after(0, lambda: messagebox.showwarning(
                    "XRT Connection", self._controller.error_message))
        threading.Thread(target=_do, daemon=True).start()

    def _on_disconnect_xrt(self):
        threading.Thread(target=self._controller.disconnect_xrt, daemon=True).start()

    def _on_connect_lebai(self):
        ip = self._entry_ip.get().strip()
        if ip:
            self._controller._lebai_ip = ip
        def _do():
            ok = self._controller.connect_lebai()
            if not ok:
                self.after(0, lambda: messagebox.showwarning(
                    "Lebai Connection", self._controller.error_message))
        threading.Thread(target=_do, daemon=True).start()

    def _on_disconnect_lebai(self):
        threading.Thread(target=self._controller.disconnect_lebai, daemon=True).start()

    def _on_start(self):
        ok = self._controller.start_teleop()
        if not ok:
            messagebox.showwarning("Start Teleop", self._controller.error_message)

    def _on_stop(self):
        self._controller.stop_teleop()

    def _on_pause(self):
        self._controller.pause_teleop()

    def _on_resume(self):
        self._controller.resume_teleop()

    def _on_emergency_stop(self):
        self._controller.emergency_stop()

    def _on_reset_estop(self):
        def _do():
            ok = self._controller.reset_from_emergency()
            if not ok:
                self.after(0, lambda: messagebox.showwarning(
                    "Reset", self._controller.error_message))
        threading.Thread(target=_do, daemon=True).start()

    def _on_scale_change(self, val):
        v = float(val)
        self._controller.set_scale_factor(v)
        self._lbl_scale.config(text=f"{v:.2f}")

    def _on_speed_change(self, val):
        v = float(val)
        self._controller.set_max_linear_speed(v)
        self._lbl_speed.config(text=f"{v:.2f}")

    def _on_hz_change(self, val):
        v = int(float(val))
        self._controller.set_control_hz(v)
        self._lbl_hz.config(text=str(v))

    # -- Calibration --

    def _on_calibrate(self):
        """Start calibration mode — captures happen via [A] button on VR controller."""
        if not self._controller.xrt_connected:
            messagebox.showwarning("Calibration", "XRT SDK not connected")
            return

        if self._calibration_step == 0:
            self._calibration_step = 1
            self._calibration_points = [None, None, None]
            self._cal_prev_a = True  # Prevent immediate capture if A is held
            self._btn_calibrate.config(text="Waiting...", state="disabled")
            self._btn_cal_cancel.config(state="normal")
            self._lbl_cal_hint.config(
                text="Step 1/3: Stand next to the robot. Hold the controller near "
                     "the robot base. Press [A] on the VR controller to capture the origin."
            )

    def _calibration_capture(self, vr_pose):
        """Called from _update_display when [A] is pressed during calibration."""
        if vr_pose is None or all(v == 0 for v in vr_pose[:3]):
            return

        pos = list(vr_pose[:3])

        if self._calibration_step == 1:
            self._calibration_points[0] = pos
            self._calibration_step = 2
            self._lbl_cal_hint.config(
                text="Origin captured! Step 2/3: Move the controller in the direction "
                     "the robot arm reaches out (its forward direction), about 20cm. "
                     "Press [A] to capture."
            )

        elif self._calibration_step == 2:
            dist = np.linalg.norm(
                np.array(pos) - np.array(self._calibration_points[0]))
            if dist < 0.05:
                self._lbl_cal_hint.config(
                    text="Too close to origin! Move at least 10cm in the robot's "
                         "forward direction, then press [A] again."
                )
                return
            self._calibration_points[1] = pos
            self._calibration_step = 3
            self._lbl_cal_hint.config(
                text="Forward captured! Step 3/3: Go back near the origin, then "
                     "move the controller STRAIGHT UP about 20cm. Press [A] to capture."
            )

        elif self._calibration_step == 3:
            dist = np.linalg.norm(
                np.array(pos) - np.array(self._calibration_points[0]))
            if dist < 0.05:
                self._lbl_cal_hint.config(
                    text="Too close to origin! Move at least 10cm straight up, "
                         "then press [A] again."
                )
                return

            self._calibration_points[2] = pos
            try:
                R = compute_calibration_matrix(
                    self._calibration_points[0],
                    self._calibration_points[1],
                    self._calibration_points[2],
                )
                self._controller.set_frame_rotation(R)
                self._lbl_cal_hint.config(
                    text="Calibration complete! Axes aligned to your position next to the robot.")
                logger.info(f"Calibration matrix updated: {R.tolist()}")
            except Exception as e:
                messagebox.showerror("Calibration", f"Calibration failed: {e}")
                self._lbl_cal_hint.config(text="Calibration failed. Try again.")

            self._calibration_step = 0
            self._btn_calibrate.config(text="Start Calibration", state="normal")
            self._btn_cal_cancel.config(state="disabled")

    def _on_calibrate_reset(self):
        """Reset calibration to default VR-to-robot mapping."""
        from frame_utils import R_VR_TO_ROBOT_DEFAULT
        self._controller.set_frame_rotation(R_VR_TO_ROBOT_DEFAULT)
        self._calibration_step = 0
        self._btn_calibrate.config(text="Start Calibration", state="normal")
        self._btn_cal_cancel.config(state="disabled")
        self._lbl_cal_hint.config(text="Reset to default mapping.")
        logger.info("Calibration reset to default")

    def _on_calibrate_cancel(self):
        """Cancel calibration in progress."""
        self._calibration_step = 0
        self._calibration_points = [None, None, None]
        self._btn_calibrate.config(text="Start Calibration", state="normal")
        self._btn_cal_cancel.config(state="disabled")
        self._lbl_cal_hint.config(
            text="Calibration cancelled. Current mapping unchanged.")

    # ======================================================================
    # Display Update Loop (20Hz)
    # ======================================================================

    def _update_display(self):
        try:
            s = self._controller.get_status()

            # -- Calibration: capture on [A] button rising edge --
            if self._calibration_step > 0:
                a_now = s.get("a_button", False)
                if a_now and not self._cal_prev_a:
                    self._calibration_capture(s["vr_pose"])
                self._cal_prev_a = a_now

            # -- Status --
            if not self._controller.xrt_available:
                self._lbl_xrt_status.config(text="SDK not installed", foreground="red")
            elif s["xrt_connected"]:
                if s.get("pico_streaming"):
                    preview = " (Preview active)" if s.get("preview_active") else ""
                    self._lbl_xrt_status.config(text=f"Connected — PICO streaming{preview}", foreground="green")
                else:
                    self._lbl_xrt_status.config(text="SDK ready — waiting for PICO data...", foreground="orange")
            else:
                self._lbl_xrt_status.config(text="Disconnected", foreground="gray")

            if not self._controller.lebai_available:
                self._lbl_lebai_status.config(text="SDK not installed", foreground="red")
            elif s["lebai_connected"]:
                self._lbl_lebai_status.config(text="Connected", foreground="green")
            else:
                self._lbl_lebai_status.config(text="Disconnected", foreground="gray")

            state = s["state"]
            color_map = {
                "disconnected": "gray", "sdk_connected": "blue",
                "running": "green", "paused": "orange",
                "error": "red", "emergency_stop": "red",
            }
            self._lbl_teleop_status.config(
                text=state.replace("_", " ").title(),
                foreground=color_map.get(state, "gray")
            )

            if s["is_controlling"]:
                self._lbl_controlling.config(text="YES (trigger held)", foreground="green")
            else:
                self._lbl_controlling.config(text="No", foreground="gray")

            # -- VR Raw Data --
            vr = s["vr_pose"]
            self._lbl_vr_pos.config(
                text=f"x={vr[0]:+.4f}  y={vr[1]:+.4f}  z={vr[2]:+.4f}"
            )
            q = s["vr_raw_quat"]
            self._lbl_vr_quat.config(
                text=f"w={q[0]:+.3f} x={q[1]:+.3f} y={q[2]:+.3f} z={q[3]:+.3f}"
            )

            self._grip_bar['value'] = s['grip'] * 100
            self._trigger_bar['value'] = s['trigger'] * 100
            self._left_trigger_bar['value'] = s.get('left_trigger', 0) * 100
            self._left_grip_bar['value'] = s.get('left_grip', 0) * 100

            a = "A" if s.get("a_button") else "-"
            b = "B" if s.get("b_button") else "-"
            x = "X" if s.get("x_button") else "-"
            y = "Y" if s.get("y_button") else "-"
            self._lbl_buttons.config(text=f"[{a}] [{b}] [{x}] [{y}]")

            # -- Translated Motion --
            self._lbl_mapping_info.config(
                text=f"VR(X,Y,Z) -> Robot(-Z,-X,Y)  |  Scale: {s['scale_factor']:.2f}x"
            )

            d = s["vr_delta_xyz"]
            self._lbl_delta_xyz.config(
                text=f"dx={d[0]:+.4f}  dy={d[1]:+.4f}  dz={d[2]:+.4f}  (m)"
            )
            dr = s["vr_delta_rot"]
            self._lbl_delta_rot.config(
                text=f"rx={dr[0]:+.4f}  ry={dr[1]:+.4f}  rz={dr[2]:+.4f}  (rad)"
            )

            sim = s["simulated_tcp"]
            self._lbl_sim_tcp.config(
                text=f"x={sim[0]:+.4f} y={sim[1]:+.4f} z={sim[2]:+.4f} "
                     f"rx={sim[3]:+.3f} ry={sim[4]:+.3f} rz={sim[5]:+.3f}"
            )

            # -- Robot State --
            tcp = s["robot_tcp"]
            target = s["target_tcp"]
            joints = s["robot_joints"]

            if s["lebai_connected"]:
                self._lbl_robot_tcp.config(
                    text=f"x={tcp[0]:+.4f} y={tcp[1]:+.4f} z={tcp[2]:+.4f} "
                         f"rx={tcp[3]:+.3f} ry={tcp[4]:+.3f} rz={tcp[5]:+.3f}"
                )
                self._lbl_target_tcp.config(
                    text=f"x={target[0]:+.4f} y={target[1]:+.4f} z={target[2]:+.4f} "
                         f"rx={target[3]:+.3f} ry={target[4]:+.3f} rz={target[5]:+.3f}"
                )
                # Show real joint angles from robot
                for i in range(6):
                    deg = np.degrees(joints[i])
                    self._lbl_joints[i].config(text=f"{deg:+7.2f} deg")
                    self._joint_bars[i]['value'] = (deg + 180) % 360
                self._lbl_joint_source.config(
                    text="Live from Lebai robot", foreground="green")
            else:
                self._lbl_robot_tcp.config(text="(Lebai not connected)")
                # Show simulated target as "target TCP" so user sees what movel would receive
                self._lbl_target_tcp.config(
                    text=f"x={sim[0]:+.4f} y={sim[1]:+.4f} z={sim[2]:+.4f} "
                         f"rx={sim[3]:+.3f} ry={sim[4]:+.3f} rz={sim[5]:+.3f}"
                )
                # Without robot IK, show orientation as approximate joint hints
                # Map simulated TCP rotation components to joint bars for visual feedback
                for i in range(6):
                    if i < 3:
                        # J1-J3: show simulated TCP position mapped to visual range
                        val = sim[i] * 100  # scale for visual
                        self._lbl_joints[i].config(text=f"~{val:+7.2f} (sim)")
                        self._joint_bars[i]['value'] = (val + 180) % 360
                    else:
                        # J4-J6: show simulated TCP rotation in degrees
                        deg = np.degrees(sim[i])
                        self._lbl_joints[i].config(text=f"~{deg:+7.2f} deg (sim)")
                        self._joint_bars[i]['value'] = (deg + 180) % 360
                self._lbl_joint_source.config(
                    text="Simulated (no IK - approximate visualization from target TCP)",
                    foreground="orange")

            # -- End Effector --
            amp = s["gripper_amplitude"]
            self._gripper_bar['value'] = amp
            state_txt = "open" if amp > 50 else "closed"
            self._lbl_gripper.config(text=f"{amp:.0f}% ({state_txt})")

            if s["suction_on"]:
                self._lbl_suction.config(text="ON", foreground="green")
            else:
                self._lbl_suction.config(text="OFF", foreground="gray")

            # -- Button States --
            is_running = state == "running"
            is_connected = state in ("sdk_connected", "running", "paused")
            self._btn_start.config(state="normal" if (is_connected and not is_running) else "disabled")
            self._btn_stop.config(state="normal" if is_running else "disabled")
            self._btn_pause.config(state="normal" if is_running else "disabled")
            self._btn_resume.config(state="normal" if state == "paused" else "disabled")

        except Exception as e:
            logger.debug(f"Display update error: {e}")

        self.after(50, self._update_display)
