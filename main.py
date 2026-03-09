"""
PICO VR Teleoperation for Lebai LM3 — Main Entry Point

Usage:
    python main.py
    python main.py --ip 10.20.17.1
"""

import argparse
import logging
import os
import sys
import tkinter as tk
from tkinter import ttk

# Ensure our directory is on sys.path for local imports
# NOTE: Use run.sh to launch — it sets LD_LIBRARY_PATH for the native libPXREARobotSDK.so
_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)

from pico_teleop_controller import PicoTeleopController
from pico_teleop_widget import PicoTeleopWidget


def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def setup_styles(root):
    """Configure ttk styles."""
    style = ttk.Style()
    try:
        style.theme_use("clam")
    except Exception:
        pass

    # Emergency stop button — red background
    style.configure("Emergency.TButton",
                     foreground="white",
                     background="#cc0000",
                     font=("TkDefaultFont", 10, "bold"))
    style.map("Emergency.TButton",
              background=[("active", "#ff0000")])


def main():
    parser = argparse.ArgumentParser(description="PICO VR Teleop for Lebai LM3")
    parser.add_argument("--ip", default="10.20.17.1", help="Lebai robot IP address")
    parser.add_argument("--scale", type=float, default=1.0, help="Scale factor")
    parser.add_argument("--hz", type=int, default=50, help="Control loop frequency")
    parser.add_argument("--max-speed", type=float, default=0.5, help="Max linear speed (m/s)")
    args = parser.parse_args()

    setup_logging()
    logger = logging.getLogger("main")
    logger.info("Starting PICO VR Teleop for Lebai LM3")

    # Create controller
    controller = PicoTeleopController(
        lebai_ip=args.ip,
        scale_factor=args.scale,
        control_hz=args.hz,
        max_linear_speed=args.max_speed,
    )

    # Create Tkinter window
    root = tk.Tk()
    root.title("PICO VR Teleop — Lebai LM3")
    root.geometry("680x920")
    root.minsize(620, 800)

    setup_styles(root)

    # Main widget
    widget = PicoTeleopWidget(root, controller)
    widget.pack(fill="both", expand=True, padx=8, pady=8)

    # Clean shutdown
    def on_closing():
        logger.info("Shutting down...")
        try:
            controller.stop_teleop()
        except Exception:
            pass
        try:
            controller.disconnect_xrt()
        except Exception:
            pass
        try:
            controller.disconnect_lebai()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    logger.info("GUI ready")
    root.mainloop()


if __name__ == "__main__":
    main()
