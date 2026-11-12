#!/usr/bin/env python3
"""
Web Interface for Automated hBN Wet Exfoliation via Dobot Magician.
Based on test_demo.py — provides controls for dipping speed, arm rotation angle
during dipping, liquid surface height & immersion depth, and multi-sample
pickup/return at configurable positions next to the dipping station.

Run with:
    python exfoliation_ui.py
Then open http://localhost:8080 in your browser.
"""

import argparse
import glob
import json
import math
import os
import struct
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

try:
    import serial
    from serial.tools import list_ports
    import pydobot
    from pydobot.message import Message
    PYDOBOT_AVAILABLE = True
except ImportError:
    serial = None
    list_ports = None
    pydobot = None
    Message = None
    PYDOBOT_AVAILABLE = False


if PYDOBOT_AVAILABLE:
    class RobustDobot(pydobot.Dobot):
        """
        Subclass of pydobot.Dobot that:
        1. Opens the serial port safely on both Linux/macOS and Windows.
        2. Flushes stale serial buffers before initializing.
        3. Retries reading serial responses instead of failing immediately with
           "'NoneType' object has no attribute 'params'" if a reply takes >100 ms.
        4. Always closes the serial port if initialization fails, preventing port leaks.
        """

        def __init__(self, port, verbose=False):
            threading.Thread.__init__(self)
            self._on = True
            self.verbose = verbose
            self.lock = threading.Lock()

            serial_kwargs = dict(
                port=port,
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.5,
            )
            # exclusive=True is supported on POSIX (Linux/macOS); Windows locks COM ports by default
            if os.name == "posix":
                serial_kwargs["exclusive"] = True

            try:
                self.ser = serial.Serial(**serial_kwargs)
            except Exception as e:
                err_msg = str(e)
                if "Resource temporarily unavailable" in err_msg or "exclusively" in err_msg or "Access is denied" in err_msg:
                    raise RuntimeError(
                        f"Port {port} is already in use by another process "
                        f"(for example, an active Jupyter kernel in test_commands.ipynb or DobotStudio). "
                        f"Please close/disconnect the other program first."
                    ) from e
                raise RuntimeError(f"Could not open serial port {port}: {e}") from e

            try:
                time.sleep(0.15)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()

                self._set_queued_cmd_start_exec()
                self._set_queued_cmd_clear()
                self._set_ptp_joint_params(200, 200, 200, 200, 200, 200, 200, 200)
                self._set_ptp_coordinate_params(velocity=200, acceleration=200)
                self._set_ptp_jump_params(10, 200)
                self._set_ptp_common_params(velocity=100, acceleration=100)
                self._get_pose()
            except Exception as e:
                try:
                    self.ser.close()
                except Exception:
                    pass
                if isinstance(e, AttributeError) and "params" in str(e):
                    raise RuntimeError(
                        f"Serial port {port} opened, but the Dobot did not respond (received no data). "
                        f"Please check that:\n"
                        f"1. The Dobot's physical power switch is turned ON (and finished booting).\n"
                        f"2. No other notebook/script (e.g. test_commands.ipynb) is connected to {port}.\n"
                        f"3. {port} is the correct USB/COM port (or enable 'Simulation Mode' to test without hardware)."
                    ) from e
                raise

        def _read_message(self):
            buf = bytearray()
            deadline = time.time() + 0.6
            while time.time() < deadline:
                time.sleep(0.08)
                chunk = self.ser.read_all()
                if chunk:
                    buf.extend(chunk)
                    idx = buf.find(b"\xaa\xaa")
                    if idx != -1 and len(buf) >= idx + 3:
                        pkt_len = buf[idx + 2]
                        total_needed = idx + 4 + pkt_len
                        if len(buf) >= total_needed:
                            frame = bytes(buf[idx:total_needed])
                            msg = Message(frame)
                            if self.verbose:
                                print("pydobot: <<", msg)
                            return msg
            if len(buf) > 0:
                return Message(bytes(buf))
            return None

        def _send_command(self, msg, wait=False):
            self.lock.acquire()
            response = None
            try:
                for _ in range(3):
                    self._send_message(msg)
                    response = self._read_message()
                    if response is not None and getattr(response, "params", None) is not None:
                        break
                    time.sleep(0.1)
            finally:
                self.lock.release()

            if response is None:
                raise RuntimeError(
                    "No response received from Dobot (serial timeout). "
                    "Ensure the Dobot power switch is ON and no other process is using the port."
                )

            if not wait:
                return response

            if len(response.params) < 4:
                return response

            expected_idx = struct.unpack_from("L", response.params, 0)[0]
            if self.verbose:
                print("pydobot: waiting for command", expected_idx)

            while True:
                try:
                    current_idx = self._get_queued_cmd_current_index()
                except Exception:
                    time.sleep(0.1)
                    continue

                if current_idx != expected_idx:
                    time.sleep(0.1)
                    continue

                if self.verbose:
                    print("pydobot: command %d executed" % current_idx)
                break

            return response


class ExfoliationController:
    """Thread-safe controller managing Dobot connection and wet exfoliation routine."""

    def __init__(self):
        self.device = None
        self.connected = False
        self.simulation = False
        self.port = "COM3" if os.name == "nt" else "/dev/ttyUSB0"
        self.lock = threading.Lock()

        # Execution state
        self.running = False
        self.abort_requested = False
        self.current_step = "Idle"
        self.step_index = 0
        self.total_steps = 0
        self.gripper_closed = False
        self.active_sample_index = -1

        # Current tracked pose: x, y, z, r (Cartesian R sent to Dobot) and j4 (physical gripper servo angle)
        self.pose = {"x": 200.0, "y": 0.0, "z": 50.0, "r": 0.0, "j4": 0.0}
        self.gripper_angle = 0.0  # Desired physical servo bracket angle (j4)
        self.logs = []

        # Default parameters based on test_demo.py
        self.default_params = {
            # Dipping parameters
            "dip_speed": 5.0,          # slow_speed in test_demo.py
            "dip_acc": 40.0,           # slow_acc in test_demo.py
            "dip_angle": 45.0,         # target_r offset (+45.0 deg in test_demo.py)
            "surface_z": 15.0,         # Liquid surface height Z (start_z - 35 = 15.0 in test_demo.py)
            "dip_depth": 15.0,         # Immersion depth below surface (reaches Z = 0.0, i.e. start_z - 50)
            "hold_time": 0.0,          # Dwell time at bottom of dip (seconds)
            "dip_cycles": 1,           # Number of slow dip cycles

            # Dipping / Home coordinates
            "home_x": 200.0,
            "home_y": 0.0,
            "safe_z": 50.0,            # home_z / start_z in test_demo.py
            "home_r": 0.0,             # Neutral physical gripper servo angle (0 deg)
            "compensate_j1": True,     # Compensates R = gripper_angle + atan2(y, x) so servo j4 never tilts when Y != 0

            # Multi-sample pickup list (each item: {"x": float, "y": float, "z": float, "enabled": bool})
            "samples": [
                {"x": 200.0, "y": 50.0, "z": 0.0, "enabled": True},
            ],

            # Transit speeds from test_demo.py
            "fast_speed": 70.0,
            "fast_acc": 70.0,
            "norm_speed": 50.0,
            "norm_acc": 50.0,
            "grip_wait": 2.0,
        }
        self.log("Ready. Connect to Dobot or enable Simulation Mode to begin.")

    def log(self, message: str):
        timestamp = time.strftime("%H:%M:%S")
        entry = f"[{timestamp}] {message}"
        print(entry, flush=True)
        self.logs.append(entry)
        if len(self.logs) > 250:
            self.logs = self.logs[-250:]

    @staticmethod
    def list_serial_ports():
        """Returns a list of port dicts [{'device': ..., 'label': ...}] across Windows, Linux, and macOS."""
        detected = []
        seen = set()

        # 1. Query pyserial's cross-platform comports() (works on Windows COMx, Linux ttyUSB/ttyACM, macOS cu.*)
        if list_ports is not None:
            try:
                for p in list_ports.comports():
                    dev = p.device
                    desc = p.description or ""
                    # Skip non-USB motherboard ttyS* clutter on Linux unless it has hardware info
                    if dev.startswith("/dev/ttyS") and (not desc or desc == "n/a"):
                        continue
                    label = f"{dev} ({desc})" if desc and desc != "n/a" and desc != dev else dev
                    detected.append({"device": dev, "label": label})
                    seen.add(dev)
            except Exception:
                pass

        # 2. Glob common Linux/macOS device nodes in case list_ports missed them
        for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/cu.usbserial*", "/dev/cu.SLAB_USBtoUART*"):
            for dev in sorted(glob.glob(pattern)):
                if dev not in seen:
                    detected.append({"device": dev, "label": dev})
                    seen.add(dev)

        # 3. Always include standard presets for Linux & Windows so user can pick or type any port
        presets = (
            ["COM3", "COM4", "COM5", "COM6", "/dev/ttyUSB0", "/dev/ttyUSB1"]
            if os.name == "nt"
            else ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "COM3", "COM4", "COM5"]
        )
        for dev in presets:
            if dev not in seen:
                detected.append({"device": dev, "label": dev})
                seen.add(dev)

        return detected

    @staticmethod
    def compute_cartesian_r(x: float, y: float, servo_angle: float, compensate_j1: bool = True) -> float:
        """
        In Dobot Magician firmware, Cartesian R is defined as:
            R = J1 + J4   where J1 = degrees(atan2(Y, X)) is the base angle
                          and J4 is the physical 4th-axis gripper servo angle.
        If R is kept at 0 while moving to Y != 0 (e.g. Y = 50 mm), the Dobot firmware drives
        the gripper servo to J4 = -J1 (tilting the gripper by -14 deg!).
        By setting R = servo_angle + J1, the physical servo angle J4 = R - J1 stays locked at
        `servo_angle` regardless of X and Y.
        """
        if not compensate_j1:
            return float(servo_angle)
        if abs(x) < 1e-6 and abs(y) < 1e-6:
            return float(servo_angle)
        j1_deg = math.degrees(math.atan2(float(y), float(x)))
        return float(servo_angle) + j1_deg

    @staticmethod
    def extract_servo_j4(x: float, y: float, cartesian_r: float) -> float:
        if abs(x) < 1e-6 and abs(y) < 1e-6:
            return float(cartesian_r)
        j1_deg = math.degrees(math.atan2(float(y), float(x)))
        return float(cartesian_r) - j1_deg

    def connect(self, port: str, simulation: bool = False):
        port = (port or "").strip()
        if not port:
            raise RuntimeError("Please specify a valid serial port (e.g. /dev/ttyUSB0 or COM3).")

        with self.lock:
            if self.running:
                raise RuntimeError("Cannot change connection while a sequence is running.")
            self._close_unsafe()

            self.port = port
            self.simulation = bool(simulation)

            if self.simulation:
                self.connected = True
                self.log("Connected in SIMULATION mode (no hardware commands sent).")
                return

            if not PYDOBOT_AVAILABLE:
                raise RuntimeError("pydobot is not installed in the current Python environment.")

            self.log(f"Connecting to Dobot on {port}...")
            try:
                self.device = RobustDobot(port=port, verbose=False)
                self.connected = True
                x = float(self.device.x)
                y = float(self.device.y)
                z = float(self.device.z)
                r = float(self.device.r)
                j4 = float(getattr(self.device, "j4", self.extract_servo_j4(x, y, r)))
                self.gripper_angle = round(j4, 2)
                self.pose = {
                    "x": round(x, 2),
                    "y": round(y, 2),
                    "z": round(z, 2),
                    "r": round(r, 2),
                    "j4": round(j4, 2),
                }
                self.log(
                    f"Dobot connected on {port}. Pose -> "
                    f"x:{self.pose['x']:.1f} y:{self.pose['y']:.1f} "
                    f"z:{self.pose['z']:.1f} r:{self.pose['r']:.1f} (Gripper J4:{self.pose['j4']:.1f}°)"
                )
            except Exception as e:
                self.device = None
                self.connected = False
                self.log(f"Error. Dobot not connected: {e}")
                raise RuntimeError(str(e))

    def disconnect(self):
        with self.lock:
            if self.running:
                self.abort_requested = True
            self._close_unsafe()
            self.log("Connection closed. It is now safe to turn off the physical power switch.")

    def _close_unsafe(self):
        if self.device is not None:
            try:
                self.device.close()
            except Exception as e:
                self.log(f"Warning while closing device: {e}")
        self.device = None
        self.connected = False
        self.simulation = False

    def _refresh_pose_unsafe(self):
        if self.device is not None and not self.simulation:
            try:
                x, y, z, r, j1, j2, j3, j4 = self.device.pose()
                self.gripper_angle = round(float(j4), 2)
                self.pose = {
                    "x": round(float(x), 2),
                    "y": round(float(y), 2),
                    "z": round(float(z), 2),
                    "r": round(float(r), 2),
                    "j4": round(float(j4), 2),
                }
            except Exception as e:
                self.log(f"Warning reading pose: {e}")

    def get_pose(self):
        with self.lock:
            if not self.connected:
                raise RuntimeError("Dobot is not connected.")
            self._refresh_pose_unsafe()
            self.log(
                f"Current position -> x:{self.pose['x']:.1f} y:{self.pose['y']:.1f} "
                f"z:{self.pose['z']:.1f} r:{self.pose['r']:.1f} (Gripper J4:{self.pose['j4']:.1f}°)"
            )
            return dict(self.pose)

    def _set_speed(self, speed: float, acc: float):
        s = max(1.0, float(speed))
        a = max(1.0, float(acc))
        if not self.simulation and self.device is not None:
            self.device.speed(s, a)

    def _execute_raw_move(self, x: float, y: float, z: float, cartesian_r: float, servo_j4: float, speed: float = None, acc: float = None):
        """Low-level execution of a single Dobot move command."""
        if self.abort_requested:
            raise InterruptedError("Sequence aborted by user.")
        if speed is not None and acc is not None:
            self._set_speed(speed, acc)

        x, y, z, cartesian_r, servo_j4 = (
            float(x),
            float(y),
            float(z),
            float(cartesian_r),
            float(servo_j4),
        )
        if self.simulation:
            steps = 8
            sx, sy, sz, sr, sj4 = (
                self.pose["x"],
                self.pose["y"],
                self.pose["z"],
                self.pose["r"],
                self.pose.get("j4", self.gripper_angle),
            )
            dist = ((x - sx) ** 2 + (y - sy) ** 2 + (z - sz) ** 2 + (servo_j4 - sj4) ** 2) ** 0.5
            sim_speed = max(5.0, speed if speed else 50.0)
            duration = min(1.8, max(0.2, dist / sim_speed))
            for i in range(1, steps + 1):
                if self.abort_requested:
                    raise InterruptedError("Sequence aborted by user.")
                t = i / steps
                self.pose = {
                    "x": round(sx + (x - sx) * t, 2),
                    "y": round(sy + (y - sy) * t, 2),
                    "z": round(sz + (z - sz) * t, 2),
                    "r": round(sr + (cartesian_r - sr) * t, 2),
                    "j4": round(sj4 + (servo_j4 - sj4) * t, 2),
                }
                time.sleep(duration / steps)
        else:
            self.device.move_to(x, y, z, cartesian_r, wait=True)
            self.pose = {
                "x": round(x, 2),
                "y": round(y, 2),
                "z": round(z, 2),
                "r": round(cartesian_r, 2),
                "j4": round(servo_j4, 2),
            }

    def _move_xyz(self, x: float, y: float, z: float, speed: float, acc: float, compensate_j1: bool = True):
        """
        Moves the arm in (X, Y, Z) while keeping the physical gripper servo angle (J4)
        completely constant. The gripper never rotates while the arm is translating.
        """
        servo_angle = self.gripper_angle
        cartesian_r = self.compute_cartesian_r(x, y, servo_angle, compensate_j1=compensate_j1)
        self._execute_raw_move(x, y, z, cartesian_r, servo_angle, speed, acc)

    def _rotate_gripper(self, target_servo_angle: float, speed: float, acc: float, compensate_j1: bool = True):
        """
        Rotates ONLY the gripper servo to `target_servo_angle` while keeping (X, Y, Z)
        strictly stationary at the current coordinates.
        """
        x, y, z = self.pose["x"], self.pose["y"], self.pose["z"]
        self.gripper_angle = float(target_servo_angle)
        cartesian_r = self.compute_cartesian_r(x, y, self.gripper_angle, compensate_j1=compensate_j1)
        self._execute_raw_move(x, y, z, cartesian_r, self.gripper_angle, speed, acc)

    def _grip(self, close: bool, wait_sec: float = 2.0):
        if self.abort_requested:
            raise InterruptedError("Sequence aborted by user.")
        action = "Closing gripper..." if close else "Opening gripper..."
        self.log(action)
        self.gripper_closed = bool(close)
        if not self.simulation and self.device is not None:
            self.device.grip(bool(close))
        end_time = time.time() + max(0.0, float(wait_sec))
        while time.time() < end_time:
            if self.abort_requested:
                raise InterruptedError("Sequence aborted by user.")
            time.sleep(0.1)

    def manual_action(self, action: str, params: dict):
        if self.running:
            raise RuntimeError("A sequence is already running.")
        if not self.connected:
            raise RuntimeError("Dobot is not connected.")

        p = {**self.default_params, **(params or {})}
        compensate_j1 = bool(p.get("compensate_j1", True))
        with self.lock:
            if action == "home":
                self.log("Moving arm to safe resting position...")
                # First move to home X, Y, Safe Z without rotating gripper
                self._move_xyz(
                    p["home_x"], p["home_y"], p["safe_z"],
                    p["fast_speed"], p["fast_acc"], compensate_j1
                )
                # Then set gripper angle to home_r while stationary
                if abs(self.gripper_angle - float(p["home_r"])) > 0.05:
                    self._rotate_gripper(float(p["home_r"]), p["norm_speed"], p["norm_acc"], compensate_j1)
                self.log("Arm parked safely.")
            elif action == "grip_close":
                self._grip(True, wait_sec=0.5)
            elif action == "grip_open":
                self._grip(False, wait_sec=0.5)
            elif action == "move_to_sample_above":
                idx = int(p.get("sample_index", 0))
                samples = p.get("samples") or self.default_params["samples"]
                if idx < 0 or idx >= len(samples):
                    idx = 0
                s = samples[idx]
                self.log(f"Moving above Sample #{idx + 1} ({s['x']}, {s['y']}, {p['safe_z']})...")
                # Ensure gripper is at home_r before moving, then move XYZ only
                if abs(self.gripper_angle - float(p["home_r"])) > 0.05:
                    self._rotate_gripper(float(p["home_r"]), p["norm_speed"], p["norm_acc"], compensate_j1)
                self._move_xyz(
                    s["x"], s["y"], p["safe_z"],
                    p["norm_speed"], p["norm_acc"], compensate_j1
                )
            else:
                raise ValueError(f"Unknown manual action: {action}")

    def start_exfoliation(self, user_params: dict):
        with self.lock:
            if not self.connected:
                raise RuntimeError("Dobot is not connected. Connect or enable Simulation first.")
            if self.running:
                raise RuntimeError("Exfoliation sequence is already running.")
            self.running = True
            self.abort_requested = False

        params = {**self.default_params, **(user_params or {})}
        thread = threading.Thread(target=self._run_exfoliation_thread, args=(params,), daemon=True)
        thread.start()

    def request_abort(self):
        if self.running:
            self.abort_requested = True
            self.log("Abort requested! Stopping after current command completes...")

    def _set_step(self, index: int, total: int, description: str):
        self.step_index = index
        self.total_steps = total
        self.current_step = description
        self.log(f"[Step {index}/{total}] {description}")

    def _run_exfoliation_thread(self, p: dict):
        try:
            home_x = float(p["home_x"])
            home_y = float(p["home_y"])
            safe_z = float(p["safe_z"])
            home_r = float(p["home_r"])
            compensate_j1 = bool(p.get("compensate_j1", True))

            raw_samples = p.get("samples") or []
            samples = [
                {
                    "x": float(s["x"]),
                    "y": float(s["y"]),
                    "z": float(s["z"]),
                    "idx": i + 1,
                }
                for i, s in enumerate(raw_samples)
                if s.get("enabled", True)
            ]
            if not samples:
                raise RuntimeError("No enabled sample spots configured. Enable or add at least one sample spot.")

            dip_speed = float(p["dip_speed"])
            dip_acc = float(p["dip_acc"])
            dip_angle = float(p["dip_angle"])
            surface_z = float(p["surface_z"])
            dip_depth = float(p["dip_depth"])
            hold_time = float(p.get("hold_time", 0.0))
            dip_cycles = max(1, int(p.get("dip_cycles", 1)))

            fast_speed = float(p["fast_speed"])
            fast_acc = float(p["fast_acc"])
            norm_speed = float(p["norm_speed"])
            norm_acc = float(p["norm_acc"])
            grip_wait = float(p.get("grip_wait", 2.0))

            target_servo_angle = home_r + dip_angle
            bottom_z = surface_z - dip_depth

            # 1 start step + 9 steps per sample + 1 final park step
            total_steps = 2 + 9 * len(samples)
            step = 0

            self.log(
                f"Starting wet exfoliation for {len(samples)} sample(s): "
                f"Dip speed={dip_speed:.1f} mm/s, Angle={target_servo_angle:.1f}°, "
                f"Surface Z={surface_z:.1f} mm, Bottom Z={bottom_z:.1f} mm"
            )

            # Step 1: Move to starting / home position and align gripper to home_r separately
            step += 1
            self._set_step(step, total_steps, "Moving to starting position")
            self._move_xyz(home_x, home_y, safe_z, fast_speed, fast_acc, compensate_j1)
            if abs(self.gripper_angle - home_r) > 0.05:
                self._rotate_gripper(home_r, norm_speed, norm_acc, compensate_j1)
            if self.gripper_closed:
                self._grip(False, wait_sec=0.5)

            # Process each sample in sequence
            for s in samples:
                sx, sy, sz, s_num = s["x"], s["y"], s["z"], s["idx"]
                self.active_sample_index = s_num - 1
                prefix = f"[Sample #{s_num}] "

                # A. Move horizontally at safe_z over sample pickup spot (gripper angle stays locked at home_r)
                step += 1
                self._set_step(step, total_steps, f"{prefix}Moving above pickup spot ({sx:.1f}, {sy:.1f}, {safe_z:.1f})")
                self._move_xyz(sx, sy, safe_z, norm_speed, norm_acc, compensate_j1)

                # B. Lower straight down to sample and close gripper
                step += 1
                self._set_step(step, total_steps, f"{prefix}Lowering to Z={sz:.1f} mm and grabbing sample")
                self._move_xyz(sx, sy, sz, norm_speed, norm_acc, compensate_j1)
                self._grip(True, wait_sec=grip_wait)

                # C. Lift sample straight up to safe height
                step += 1
                self._set_step(step, total_steps, f"{prefix}Lifting sample to safe height (Z={safe_z:.1f} mm)")
                self._move_xyz(sx, sy, safe_z, norm_speed, norm_acc, compensate_j1)

                # D. Move horizontally to dipping station at safe_z (no gripper rotation during transit)
                step += 1
                self._set_step(step, total_steps, f"{prefix}Moving to dipping station ({home_x:.1f}, {home_y:.1f}, {safe_z:.1f})")
                self._move_xyz(home_x, home_y, safe_z, norm_speed, norm_acc, compensate_j1)

                # E. Rotate gripper in place at stationary (home_x, home_y, safe_z)
                step += 1
                self._set_step(step, total_steps, f"{prefix}Rotating gripper in place to dipping angle ({target_servo_angle:.1f}°)")
                self._rotate_gripper(target_servo_angle, norm_speed, norm_acc, compensate_j1)

                # F. Drop vertically to liquid surface height at normal speed (gripper angle locked)
                step += 1
                self._set_step(step, total_steps, f"{prefix}Approaching liquid surface (Z={surface_z:.1f} mm)")
                self._move_xyz(home_x, home_y, surface_z, norm_speed, norm_acc, compensate_j1)

                # G. Slow dip into liquid and pull back up (X, Y, and gripper angle strictly constant)
                step += 1
                for cycle in range(1, dip_cycles + 1):
                    cycle_label = f" (Cycle {cycle}/{dip_cycles})" if dip_cycles > 1 else ""
                    self._set_step(
                        step,
                        total_steps,
                        f"{prefix}Dipping to Z={bottom_z:.1f} mm at {dip_speed:.1f} mm/s{cycle_label}",
                    )
                    self._move_xyz(home_x, home_y, bottom_z, dip_speed, dip_acc, compensate_j1)

                    if hold_time > 0:
                        self.log(f"{prefix}Holding submerged for {hold_time:.1f} s...")
                        end_hold = time.time() + hold_time
                        while time.time() < end_hold:
                            if self.abort_requested:
                                raise InterruptedError("Sequence aborted by user.")
                            time.sleep(0.1)

                    self.log(f"{prefix}Withdrawing sample to liquid surface (Z={surface_z:.1f} mm)...")
                    self._move_xyz(home_x, home_y, surface_z, dip_speed, dip_acc, compensate_j1)

                # H. Raise vertically to safe_z FIRST, THEN rotate gripper back to home_r while stationary
                step += 1
                self._set_step(step, total_steps, f"{prefix}Raising to Z={safe_z:.1f} mm, then resetting gripper angle to {home_r:.1f}°")
                self._move_xyz(home_x, home_y, safe_z, norm_speed, norm_acc, compensate_j1)
                self._rotate_gripper(home_r, norm_speed, norm_acc, compensate_j1)

                # I. Return sample to its pickup station and release gripper (no gripper rotation during transit)
                step += 1
                self._set_step(step, total_steps, f"{prefix}Returning sample to spot ({sx:.1f}, {sy:.1f}, {sz:.1f}) and releasing")
                self._move_xyz(sx, sy, safe_z, norm_speed, norm_acc, compensate_j1)
                self._move_xyz(sx, sy, sz, norm_speed, norm_acc, compensate_j1)
                self._grip(False, wait_sec=grip_wait)
                self._move_xyz(sx, sy, safe_z, norm_speed, norm_acc, compensate_j1)

            # Final Step: Park at safe resting position
            self.active_sample_index = -1
            step += 1
            self._set_step(step, total_steps, "Moving arm to safe resting position")
            self._move_xyz(home_x, home_y, safe_z, fast_speed, fast_acc, compensate_j1)
            self.log("All samples completed. Arm parked safely.")
            self.current_step = "Completed"

        except InterruptedError as e:
            self.log(f"Sequence aborted: {e}")
            self.current_step = "Aborted"
            try:
                self.log("Raising arm to safe height after abort...")
                safe_z = float(p.get("safe_z", 50.0))
                compensate_j1 = bool(p.get("compensate_j1", True))
                self.abort_requested = False
                self._move_xyz(self.pose["x"], self.pose["y"], safe_z, 50.0, 50.0, compensate_j1)
            except Exception:
                pass
        except Exception as e:
            self.log(f"Error during exfoliation: {e}")
            self.current_step = f"Error: {e}"
        finally:
            with self.lock:
                self.running = False
                self.abort_requested = False
                self.active_sample_index = -1

    def status_dict(self):
        return {
            "connected": self.connected,
            "simulation": self.simulation,
            "port": self.port,
            "available_ports": self.list_serial_ports(),
            "pydobot_available": PYDOBOT_AVAILABLE,
            "running": self.running,
            "current_step": self.current_step,
            "step_index": self.step_index,
            "total_steps": self.total_steps,
            "pose": self.pose,
            "gripper_closed": self.gripper_closed,
            "active_sample_index": self.active_sample_index,
            "logs": self.logs[-80:],
            "defaults": self.default_params,
        }


CONTROLLER = ExfoliationController()


HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Dobot Magician — hBN Wet Exfoliation Interface</title>
<style>
  :root {
    --bg: #0f172a;
    --panel: #1e293b;
    --panel-alt: #334155;
    --border: #475569;
    --text: #f8fafc;
    --muted: #94a3b8;
    --accent: #38bdf8;
    --accent-hover: #0284c7;
    --success: #22c55e;
    --warn: #f59e0b;
    --danger: #ef4444;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    background: var(--bg);
    color: var(--text);
    line-height: 1.45;
  }
  header {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: space-between;
    padding: 14px 24px;
    background: #0b1120;
    border-bottom: 1px solid var(--border);
    gap: 12px;
  }
  header h1 {
    margin: 0;
    font-size: 1.25rem;
    font-weight: 600;
    display: flex;
    align-items: center;
    gap: 10px;
  }
  .badge {
    font-size: 0.78rem;
    padding: 4px 10px;
    border-radius: 999px;
    font-weight: 600;
    background: var(--panel-alt);
    color: var(--muted);
  }
  .badge.connected { background: rgba(34, 197, 94, 0.2); color: #4ade80; border: 1px solid #22c55e; }
  .badge.sim { background: rgba(245, 158, 11, 0.2); color: #fbbf24; border: 1px solid #f59e0b; }
  .conn-bar {
    display: flex;
    align-items: center;
    gap: 8px;
    flex-wrap: wrap;
  }
  select, input[type="number"], input[type="text"] {
    background: #0f172a;
    border: 1px solid var(--border);
    color: var(--text);
    padding: 7px 10px;
    border-radius: 6px;
    font-size: 0.88rem;
  }
  select:focus, input:focus {
    outline: none;
    border-color: var(--accent);
  }
  button {
    cursor: pointer;
    border: none;
    border-radius: 6px;
    padding: 7px 13px;
    font-size: 0.86rem;
    font-weight: 600;
    color: var(--text);
    background: var(--panel-alt);
    transition: background 0.15s, opacity 0.15s;
  }
  button:hover:not(:disabled) { background: var(--border); }
  button:disabled { opacity: 0.45; cursor: not-allowed; }
  .btn-primary { background: #0284c7; color: #fff; }
  .btn-primary:hover:not(:disabled) { background: #0369a1; }
  .btn-success { background: #16a34a; color: #fff; font-size: 1rem; padding: 12px 20px; }
  .btn-success:hover:not(:disabled) { background: #15803d; }
  .btn-danger { background: #dc2626; color: #fff; }
  .btn-danger:hover:not(:disabled) { background: #b91c1c; }
  .btn-sm { padding: 4px 9px; font-size: 0.76rem; }

  main {
    max-width: 1420px;
    margin: 0 auto;
    padding: 20px;
    display: grid;
    grid-template-columns: 1.18fr 0.82fr;
    gap: 20px;
  }
  @media (max-width: 1024px) {
    main { grid-template-columns: 1fr; }
  }
  .card {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 18px;
    margin-bottom: 20px;
  }
  .card h2 {
    margin: 0 0 14px 0;
    font-size: 1.05rem;
    color: var(--accent);
    border-bottom: 1px solid rgba(255,255,255,0.08);
    padding-bottom: 8px;
    display: flex;
    justify-content: space-between;
    align-items: center;
  }
  .grid-2 {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 14px;
  }
  .grid-4 {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 10px;
  }
  .field {
    display: flex;
    flex-direction: column;
    gap: 5px;
  }
  .field label {
    font-size: 0.82rem;
    color: var(--muted);
    display: flex;
    justify-content: space-between;
    align-items: baseline;
  }
  .field label span.unit {
    color: var(--accent);
    font-weight: 600;
  }
  .slider-row {
    display: flex;
    align-items: center;
    gap: 10px;
  }
  .slider-row input[type="range"] {
    flex: 1;
    accent-color: var(--accent);
  }
  .slider-row input[type="number"] {
    width: 82px;
  }
  .hint {
    font-size: 0.76rem;
    color: var(--muted);
  }
  .summary-box {
    background: rgba(56, 189, 248, 0.08);
    border: 1px solid rgba(56, 189, 248, 0.3);
    border-radius: 8px;
    padding: 10px 14px;
    margin-top: 12px;
    font-size: 0.85rem;
    display: flex;
    justify-content: space-between;
    flex-wrap: wrap;
    gap: 8px;
  }
  .sample-table {
    width: 100%;
    border-collapse: collapse;
    margin-top: 6px;
    font-size: 0.84rem;
  }
  .sample-table th {
    text-align: left;
    color: var(--muted);
    font-weight: 600;
    padding: 6px 6px;
    border-bottom: 1px solid var(--border);
    font-size: 0.76rem;
  }
  .sample-table td {
    padding: 6px 5px;
    border-bottom: 1px solid rgba(255,255,255,0.06);
    vertical-align: middle;
  }
  .sample-table tr.active-sample {
    background: rgba(251, 191, 36, 0.14);
  }
  .sample-table input[type="number"] {
    width: 78px;
    padding: 5px 7px;
  }
  .pose-readout {
    display: grid;
    grid-template-columns: repeat(5, 1fr);
    gap: 8px;
    background: #0b1120;
    padding: 10px;
    border-radius: 8px;
    border: 1px solid var(--border);
    text-align: center;
    margin-bottom: 12px;
  }
  .pose-item .lbl { font-size: 0.72rem; color: var(--muted); }
  .pose-item .val { font-size: 1.02rem; font-weight: 700; font-family: monospace; color: var(--accent); }
  .action-row {
    display: flex;
    gap: 8px;
    flex-wrap: wrap;
    align-items: center;
  }
  .progress-wrap {
    margin-top: 12px;
    background: #0b1120;
    border-radius: 8px;
    padding: 12px;
    border: 1px solid var(--border);
  }
  .progress-bar-bg {
    height: 8px;
    background: var(--panel-alt);
    border-radius: 4px;
    overflow: hidden;
    margin-top: 8px;
  }
  .progress-bar-fill {
    height: 100%;
    background: var(--accent);
    width: 0%;
    transition: width 0.25s;
  }
  #logBox {
    background: #090d16;
    color: #cbd5e1;
    font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
    font-size: 0.8rem;
    height: 220px;
    overflow-y: auto;
    padding: 10px;
    border-radius: 6px;
    border: 1px solid var(--border);
    white-space: pre-wrap;
  }
  svg.viz {
    width: 100%;
    height: 270px;
    background: #0b1120;
    border-radius: 8px;
    border: 1px solid var(--border);
  }
</style>
</head>
<body>

<header>
  <h1>
    <span>Dobot Magician — Wet Exfoliation Control</span>
    <span id="connBadge" class="badge">Disconnected</span>
  </h1>
  <div class="conn-bar">
    <label for="portSelect" style="font-size:0.84rem;color:var(--muted);">Port:</label>
    <select id="portSelect" onchange="onPortSelectChange()" title="Select detected serial/COM port"></select>
    <input type="text" id="portCustom" value="/dev/ttyUSB0" style="width:130px;" title="Or type any port name (e.g. COM3, /dev/ttyUSB0)">
    <button class="btn-sm" onclick="refreshPorts()" title="Rescan USB / COM ports">&#8635; Scan</button>
    <label style="font-size:0.84rem;display:flex;align-items:center;gap:5px;cursor:pointer;">
      <input type="checkbox" id="simCheck"> Simulation Mode
    </label>
    <button id="btnConnect" class="btn-primary" onclick="connectDobot()">Connect</button>
    <button id="btnDisconnect" class="btn-danger" onclick="disconnectDobot()" disabled>Disconnect</button>
  </div>
</header>

<main>
  <!-- Left Column: Parameters -->
  <div>
    <!-- Primary Dipping Parameters -->
    <div class="card">
      <h2>
        <span>1. Dipping &amp; Liquid Surface Parameters</span>
        <button class="btn-sm" onclick="resetDefaults()">Reset to test_demo.py Defaults</button>
      </h2>

      <div class="grid-2">
        <div class="field">
          <label><span>Dipping Speed (slow_speed)</span> <span class="unit">mm/s</span></label>
          <div class="slider-row">
            <input type="range" id="dip_speed_range" min="1" max="50" step="1" value="5" oninput="syncInput('dip_speed', this.value)">
            <input type="number" id="dip_speed" min="1" max="100" step="1" value="5" oninput="syncInput('dip_speed', this.value)">
          </div>
          <span class="hint">Speed while submerging &amp; withdrawing sample (default: 5)</span>
        </div>

        <div class="field">
          <label><span>Arm Angle During Dipping (ΔR)</span> <span class="unit">deg (°)</span></label>
          <div class="slider-row">
            <input type="range" id="dip_angle_range" min="-90" max="90" step="1" value="45" oninput="syncInput('dip_angle', this.value)">
            <input type="number" id="dip_angle" min="-135" max="135" step="0.5" value="45" oninput="syncInput('dip_angle', this.value)">
          </div>
          <span class="hint">Gripper servo rotation during dip (rotates only while arm is stationary)</span>
        </div>

        <div class="field">
          <label><span>Liquid Surface Height (Z<sub>surface</sub>)</span> <span class="unit">mm</span></label>
          <div class="slider-row">
            <input type="range" id="surface_z_range" min="-30" max="50" step="1" value="15" oninput="syncInput('surface_z', this.value)">
            <input type="number" id="surface_z" min="-60" max="100" step="0.5" value="15" oninput="syncInput('surface_z', this.value)">
          </div>
          <span class="hint">Height where arm slows to dipping speed (test_demo: 50 - 35 = 15 mm)</span>
        </div>

        <div class="field">
          <label><span>Immersion Depth Below Surface (ΔZ<sub>dip</sub>)</span> <span class="unit">mm</span></label>
          <div class="slider-row">
            <input type="range" id="dip_depth_range" min="1" max="50" step="1" value="15" oninput="syncInput('dip_depth', this.value)">
            <input type="number" id="dip_depth" min="0" max="80" step="0.5" value="15" oninput="syncInput('dip_depth', this.value)">
          </div>
          <span class="hint">Distance submerged below liquid surface (default: 15 mm &rarr; Z = 0 mm)</span>
        </div>

        <div class="field">
          <label><span>Number of Dip Cycles per Sample</span> <span class="unit">cycles</span></label>
          <input type="number" id="dip_cycles" min="1" max="50" step="1" value="1" oninput="updateDiagram()">
          <span class="hint">Repeats the slow submerge/withdraw stroke for each sample</span>
        </div>

        <div class="field">
          <label><span>Hold Time Submerged</span> <span class="unit">sec</span></label>
          <input type="number" id="hold_time" min="0" max="120" step="0.5" value="0" oninput="updateDiagram()">
          <span class="hint">Optional pause at lowest point before pulling up</span>
        </div>
      </div>

      <div class="summary-box" id="dipSummary">
        <span>Safe Height Z: <strong id="sumSafeZ">50.0 mm</strong></span>
        <span>Liquid Surface Z: <strong id="sumSurfaceZ">15.0 mm</strong></span>
        <span>Lowest Dip Point Z: <strong id="sumBottomZ">0.0 mm</strong></span>
        <span>Dip Gripper Angle: <strong id="sumTargetR">45.0°</strong></span>
      </div>
    </div>

    <!-- Positions & Multi-Sample Queue -->
    <div class="card">
      <h2>
        <span>2. Sample Spots Queue &amp; Dipping Position</span>
        <button class="btn-primary btn-sm" onclick="addSampleSpot()">+ Add Sample Spot</button>
      </h2>

      <div style="margin-bottom: 16px;">
        <div style="display:flex;justify-content:space-between;align-items:center;">
          <strong style="font-size:0.88rem;color:#fbbf24;">Sample Pickup Spots (Processed in Order: Pick Up &rarr; Dip &rarr; Put Back &rarr; Next Sample)</strong>
        </div>
        <table class="sample-table">
          <thead>
            <tr>
              <th>Run</th>
              <th>Spot</th>
              <th>Sample X (mm)</th>
              <th>Sample Y (mm)</th>
              <th>Grab Z (mm)</th>
              <th>Actions</th>
            </tr>
          </thead>
          <tbody id="sampleTableBody"></tbody>
        </table>
        <span class="hint">For each enabled spot, the arm moves at Safe Z, lowers straight down with the gripper level (no rotation during travel), grabs the sample, dips it, puts it back at the same spot, and moves to the next spot.</span>
      </div>

      <div style="margin-bottom: 12px;">
        <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:6px;">
          <strong style="font-size:0.88rem;color:#38bdf8;">Dipping &amp; Home Resting Position</strong>
          <button class="btn-sm" onclick="useCurrentPoseForHome()">Set X/Y from Current Arm Pose</button>
        </div>
        <div class="grid-4">
          <div class="field">
            <label><span>Dip / Home X</span> <span class="unit">mm</span></label>
            <input type="number" id="home_x" step="1" value="200.0" oninput="updateDiagram()">
          </div>
          <div class="field">
            <label><span>Dip / Home Y</span> <span class="unit">mm</span></label>
            <input type="number" id="home_y" step="1" value="0.0" oninput="updateDiagram()">
          </div>
          <div class="field">
            <label><span>Safe Travel Z</span> <span class="unit">mm</span></label>
            <input type="number" id="safe_z" step="1" value="50.0" oninput="updateDiagram()">
          </div>
          <div class="field">
            <label><span>Neutral Gripper Angle</span> <span class="unit">deg</span></label>
            <input type="number" id="home_r" step="1" value="0.0" oninput="updateDiagram()">
          </div>
        </div>
      </div>

      <div style="margin-top:10px;padding:8px 10px;background:#0b1120;border-radius:6px;border:1px solid var(--border);">
        <label style="font-size:0.82rem;display:flex;align-items:center;gap:8px;cursor:pointer;">
          <input type="checkbox" id="compensate_j1" checked>
          <span><strong>Lock Gripper Angle Against Base Rotation (J1 Compensation)</strong> — prevents Dobot from tilting the gripper servo when moving sideways (Y &ne; 0) to pick up samples.</span>
        </label>
      </div>

      <details style="margin-top:10px;">
        <summary style="cursor:pointer;font-size:0.84rem;color:var(--muted);">Advanced Speed &amp; Acceleration Settings</summary>
        <div class="grid-4" style="margin-top:10px;">
          <div class="field">
            <label><span>Dip Accel</span></label>
            <input type="number" id="dip_acc" value="40">
          </div>
          <div class="field">
            <label><span>Normal Speed</span></label>
            <input type="number" id="norm_speed" value="50">
          </div>
          <div class="field">
            <label><span>Fast Speed</span></label>
            <input type="number" id="fast_speed" value="70">
          </div>
          <div class="field">
            <label><span>Grip Wait (s)</span></label>
            <input type="number" id="grip_wait" step="0.5" value="2.0">
          </div>
        </div>
      </details>
    </div>
  </div>

  <!-- Right Column: Execution, Visualization & Logs -->
  <div>
    <div class="card">
      <h2>
        <span>3. Run Wet Exfoliation Sequence</span>
      </h2>

      <div class="pose-readout">
        <div class="pose-item"><div class="lbl">X (mm)</div><div class="val" id="poseX">200.0</div></div>
        <div class="pose-item"><div class="lbl">Y (mm)</div><div class="val" id="poseY">0.0</div></div>
        <div class="pose-item"><div class="lbl">Z (mm)</div><div class="val" id="poseZ">50.0</div></div>
        <div class="pose-item"><div class="lbl">Gripper Angle</div><div class="val" id="poseJ4">0.0°</div></div>
        <div class="pose-item"><div class="lbl">Gripper</div><div class="val" id="poseGrip">Open</div></div>
      </div>

      <div class="action-row" style="margin-bottom: 12px;">
        <button id="btnStart" class="btn-success" onclick="startExfoliation()" style="flex:1;">
          &#9654; Start Wet Exfoliation Sequence
        </button>
        <button id="btnAbort" class="btn-danger" onclick="abortExfoliation()" disabled style="padding:12px 18px;">
          &#9632; Stop
        </button>
      </div>

      <div class="action-row">
        <button onclick="readPose()">Read Pose</button>
        <button onclick="manualAction('home')">Move to Home</button>
        <button onclick="manualAction('grip_close')">Close Gripper</button>
        <button onclick="manualAction('grip_open')">Open Gripper</button>
      </div>

      <div class="progress-wrap">
        <div style="display:flex;justify-content:space-between;font-size:0.85rem;">
          <span id="stepText">Status: Idle</span>
          <span id="stepCount" style="color:var(--muted);">0 / 11</span>
        </div>
        <div class="progress-bar-bg">
          <div class="progress-bar-fill" id="progressBar"></div>
        </div>
      </div>
    </div>

    <!-- Live Process Schematic -->
    <div class="card">
      <h2><span>Live Trajectory &amp; Liquid Level Preview (Y–Z View)</span></h2>
      <svg class="viz" id="vizSvg" viewBox="0 0 500 240">
        <!-- Safe Z dashed line -->
        <line id="svgSafeLine" x1="25" y1="45" x2="475" y2="45" stroke="#475569" stroke-dasharray="5,5" stroke-width="1.5"/>
        <text id="svgSafeText" x="30" y="39" fill="#94a3b8" font-size="11">Safe Z = 50 mm</text>

        <!-- Dipping Beaker (Left side: Y = 0) -->
        <rect x="95" y="95" width="90" height="110" fill="none" stroke="#94a3b8" stroke-width="2.5" rx="4"/>
        <!-- Liquid inside beaker -->
        <rect id="svgLiquid" x="97" y="130" width="86" height="73" fill="rgba(56, 189, 248, 0.28)" stroke="#38bdf8" stroke-width="1.5"/>
        <text id="svgSurfaceText" x="22" y="134" fill="#38bdf8" font-size="11" font-weight="bold">Surface Z=15</text>
        <!-- Dip target depth line -->
        <line id="svgBottomLine" x1="102" y1="165" x2="178" y2="165" stroke="#f43f5e" stroke-dasharray="3,3" stroke-width="1.5"/>
        <text id="svgBottomText" x="105" y="180" fill="#fda4af" font-size="10">Dip Z=0.0</text>
        <text x="100" y="222" fill="#38bdf8" font-size="12" font-weight="bold">Dipping Pos (Y=0)</text>

        <!-- Dynamic Sample Pedestals Group -->
        <g id="svgSamplesGroup"></g>

        <!-- Robot End-Effector Marker -->
        <g id="svgArmGroup" transform="translate(140, 45)">
          <line x1="0" y1="-40" x2="0" y2="0" stroke="#e2e8f0" stroke-width="4"/>
          <g id="svgWrist" transform="rotate(0)">
            <rect x="-12" y="-6" width="24" height="12" fill="#38bdf8" rx="3"/>
            <line id="svgFingerL" x1="-8" y1="6" x2="-8" y2="18" stroke="#f8fafc" stroke-width="3"/>
            <line id="svgFingerR" x1="8" y1="6" x2="8" y2="18" stroke="#f8fafc" stroke-width="3"/>
            <circle id="svgHeldSample" cx="0" cy="15" r="5" fill="#fbbf24" visibility="hidden"/>
          </g>
        </g>
      </svg>
    </div>

    <!-- Console Log -->
    <div class="card" style="margin-bottom:0;">
      <h2><span>Activity Log</span></h2>
      <div id="logBox"></div>
    </div>
  </div>
</main>

<script>
const SCALAR_PARAM_IDS = [
  'dip_speed', 'dip_acc', 'dip_angle', 'surface_z', 'dip_depth',
  'hold_time', 'dip_cycles', 'home_x', 'home_y', 'safe_z', 'home_r',
  'fast_speed', 'norm_speed', 'grip_wait'
];

let sampleSpots = [
  { x: 200.0, y: 50.0, z: 0.0, enabled: true }
];
let activeSampleIdx = -1;

function renderSampleTable() {
  const tbody = document.getElementById('sampleTableBody');
  tbody.innerHTML = '';
  sampleSpots.forEach((s, idx) => {
    const tr = document.createElement('tr');
    if (idx === activeSampleIdx) tr.className = 'active-sample';
    tr.innerHTML = `
      <td><input type="checkbox" ${s.enabled ? 'checked' : ''} onchange="updateSampleField(${idx}, 'enabled', this.checked)"></td>
      <td><strong style="color:#fbbf24;">#${idx + 1}</strong></td>
      <td><input type="number" step="1" value="${s.x}" oninput="updateSampleField(${idx}, 'x', parseFloat(this.value) || 0)"></td>
      <td><input type="number" step="1" value="${s.y}" oninput="updateSampleField(${idx}, 'y', parseFloat(this.value) || 0)"></td>
      <td><input type="number" step="1" value="${s.z}" oninput="updateSampleField(${idx}, 'z', parseFloat(this.value) || 0)"></td>
      <td>
        <div style="display:flex;gap:4px;">
          <button class="btn-sm" onclick="setSampleFromPose(${idx})" title="Capture current arm X, Y, Z into Sample #${idx + 1}">Set Pose</button>
          <button class="btn-sm" onclick="moveAboveSample(${idx})" title="Move arm to Safe Z above Sample #${idx + 1}">Move Above</button>
          <button class="btn-sm btn-danger" onclick="removeSampleSpot(${idx})" ${sampleSpots.length <= 1 ? 'disabled' : ''} title="Remove spot">&times;</button>
        </div>
      </td>
    `;
    tbody.appendChild(tr);
  });
  updateDiagram();
}

function updateSampleField(idx, field, value) {
  if (!sampleSpots[idx]) return;
  sampleSpots[idx][field] = value;
  updateDiagram();
}

function addSampleSpot() {
  const last = sampleSpots[sampleSpots.length - 1] || { x: 200.0, y: 50.0, z: 0.0 };
  // Offset next sample spot by +25 mm in Y by default
  sampleSpots.push({
    x: last.x,
    y: Math.round((last.y + 25.0) * 10) / 10,
    z: last.z,
    enabled: true
  });
  renderSampleTable();
}

function removeSampleSpot(idx) {
  if (sampleSpots.length <= 1) return;
  sampleSpots.splice(idx, 1);
  renderSampleTable();
}

function syncInput(baseId, val) {
  const numEl = document.getElementById(baseId);
  const rangeEl = document.getElementById(baseId + '_range');
  if (numEl && numEl.value !== val) numEl.value = val;
  if (rangeEl && rangeEl.value !== val) rangeEl.value = val;
  updateDiagram();
}

function getParams() {
  const p = {};
  for (const id of SCALAR_PARAM_IDS) {
    const el = document.getElementById(id);
    if (el) p[id] = parseFloat(el.value);
  }
  p.compensate_j1 = document.getElementById('compensate_j1').checked;
  p.samples = sampleSpots.map(s => ({
    x: parseFloat(s.x) || 0,
    y: parseFloat(s.y) || 0,
    z: parseFloat(s.z) || 0,
    enabled: Boolean(s.enabled)
  }));
  return p;
}

function resetDefaults() {
  const defaults = {
    dip_speed: 5, dip_acc: 40, dip_angle: 45, surface_z: 15, dip_depth: 15,
    hold_time: 0, dip_cycles: 1,
    home_x: 200, home_y: 0, safe_z: 50, home_r: 0,
    fast_speed: 70, norm_speed: 50, grip_wait: 2.0
  };
  for (const [k, v] of Object.entries(defaults)) {
    syncInput(k, v);
  }
  document.getElementById('compensate_j1').checked = true;
  sampleSpots = [{ x: 200.0, y: 50.0, z: 0.0, enabled: true }];
  renderSampleTable();
}

function zToSvgY(z, safeZ) {
  const topZ = Math.max(50, safeZ);
  const minZ = -20;
  const ratio = (topZ - z) / (topZ - minZ);
  return Math.max(25, Math.min(200, 45 + ratio * 145));
}

function yToSvgX(yVal, homeY) {
  // Maps Y from homeY (at SVG X=140) to sample spots on the right (SVG X 260..450)
  const dy = yVal - homeY;
  return Math.max(60, Math.min(460, 140 + dy * 3.2));
}

function updateDiagram(livePose, gripperClosed) {
  const p = getParams();
  const bottomZ = p.surface_z - p.dip_depth;
  const targetR = p.home_r + p.dip_angle;

  document.getElementById('sumSafeZ').textContent = p.safe_z.toFixed(1) + ' mm';
  document.getElementById('sumSurfaceZ').textContent = p.surface_z.toFixed(1) + ' mm';
  document.getElementById('sumBottomZ').textContent = bottomZ.toFixed(1) + ' mm';
  document.getElementById('sumTargetR').textContent = targetR.toFixed(1) + '°';

  const safeSvgY = zToSvgY(p.safe_z, p.safe_z);
  const surfSvgY = zToSvgY(p.surface_z, p.safe_z);
  const botSvgY = zToSvgY(bottomZ, p.safe_z);

  document.getElementById('svgSafeLine').setAttribute('y1', safeSvgY);
  document.getElementById('svgSafeLine').setAttribute('y2', safeSvgY);
  document.getElementById('svgSafeText').setAttribute('y', safeSvgY - 6);
  document.getElementById('svgSafeText').textContent = `Safe Z = ${p.safe_z.toFixed(1)} mm`;

  const liqH = Math.max(8, 203 - surfSvgY);
  document.getElementById('svgLiquid').setAttribute('y', surfSvgY);
  document.getElementById('svgLiquid').setAttribute('height', liqH);
  document.getElementById('svgSurfaceText').setAttribute('y', surfSvgY + 4);
  document.getElementById('svgSurfaceText').textContent = `Surface Z=${p.surface_z.toFixed(1)}`;

  document.getElementById('svgBottomLine').setAttribute('y1', botSvgY);
  document.getElementById('svgBottomLine').setAttribute('y2', botSvgY);
  document.getElementById('svgBottomText').setAttribute('y', Math.min(200, botSvgY + 13));
  document.getElementById('svgBottomText').textContent = `Dip Z=${bottomZ.toFixed(1)}`;

  // Draw all sample spots
  const grp = document.getElementById('svgSamplesGroup');
  grp.innerHTML = '';
  p.samples.forEach((s, idx) => {
    const sx = yToSvgX(s.y, p.home_y);
    const sy = zToSvgY(s.z, p.safe_z);
    const color = s.enabled ? '#fbbf24' : '#64748b';
    const g = document.createElementNS('http://www.w3.org/2000/svg', 'g');
    g.innerHTML = `
      <rect x="${sx - 22}" y="${sy}" width="44" height="${Math.max(12, 205 - sy)}" fill="#1e293b" stroke="${color}" stroke-width="1.8" rx="2"/>
      <rect x="${sx - 10}" y="${sy - 7}" width="20" height="7" fill="${color}" rx="1.5"/>
      <text x="${sx - 14}" y="222" fill="${color}" font-size="11" font-weight="bold">S#${idx + 1} (Y=${s.y.toFixed(0)})</text>
    `;
    grp.appendChild(g);
  });

  if (livePose) {
    const armSvgX = yToSvgX(livePose.y, p.home_y);
    const armSvgY = zToSvgY(livePose.z, p.safe_z) - 14;
    const j4 = (livePose.j4 !== undefined) ? livePose.j4 : livePose.r;
    document.getElementById('svgArmGroup').setAttribute('transform', `translate(${armSvgX.toFixed(1)}, ${armSvgY.toFixed(1)})`);
    document.getElementById('svgWrist').setAttribute('transform', `rotate(${j4.toFixed(1)})`);
    document.getElementById('svgHeldSample').setAttribute('visibility', gripperClosed ? 'visible' : 'hidden');
    document.getElementById('svgFingerL').setAttribute('x1', gripperClosed ? -4 : -8);
    document.getElementById('svgFingerL').setAttribute('x2', gripperClosed ? -4 : -8);
    document.getElementById('svgFingerR').setAttribute('x1', gripperClosed ? 4 : 8);
    document.getElementById('svgFingerR').setAttribute('x2', gripperClosed ? 4 : 8);
  }
}

async function apiPost(path, body) {
  const res = await fetch(path, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body || {})
  });
  const data = await res.json();
  if (!res.ok || data.error) {
    alert(data.error || 'Operation failed');
    throw new Error(data.error || 'Operation failed');
  }
  return data;
}

function onPortSelectChange() {
  const sel = document.getElementById('portSelect');
  if (sel.value) {
    document.getElementById('portCustom').value = sel.value;
  }
}

function populatePortDropdown(ports, currentPort) {
  const sel = document.getElementById('portSelect');
  const prev = document.getElementById('portCustom').value || currentPort;
  sel.innerHTML = '';
  for (const item of (ports || [])) {
    const dev = typeof item === 'string' ? item : item.device;
    const label = typeof item === 'string' ? item : item.label;
    const opt = document.createElement('option');
    opt.value = dev;
    opt.textContent = label;
    if (dev === prev) opt.selected = true;
    sel.appendChild(opt);
  }
  if (!document.getElementById('portCustom').value && sel.value) {
    document.getElementById('portCustom').value = sel.value;
  }
}

async function refreshPorts() {
  const res = await fetch('/api/status');
  const st = await res.json();
  populatePortDropdown(st.available_ports, st.port);
}

async function connectDobot() {
  const port = document.getElementById('portCustom').value.trim() || document.getElementById('portSelect').value;
  const simulation = document.getElementById('simCheck').checked;
  await apiPost('/api/connect', {port, simulation});
  await pollStatus();
}

async function disconnectDobot() {
  await apiPost('/api/disconnect', {});
  await pollStatus();
}

async function startExfoliation() {
  const params = getParams();
  await apiPost('/api/start', params);
  await pollStatus();
}

async function abortExfoliation() {
  await apiPost('/api/abort', {});
}

async function readPose() {
  await apiPost('/api/pose', {});
  await pollStatus();
}

async function manualAction(action, extra) {
  const params = { ...getParams(), ...(extra || {}) };
  await apiPost('/api/manual', {action, params});
  await pollStatus();
}

async function moveAboveSample(idx) {
  await manualAction('move_to_sample_above', { sample_index: idx });
}

async function setSampleFromPose(idx) {
  const data = await apiPost('/api/pose', {});
  const pose = data.pose;
  if (!pose || !sampleSpots[idx]) return;
  sampleSpots[idx].x = pose.x;
  sampleSpots[idx].y = pose.y;
  sampleSpots[idx].z = pose.z;
  renderSampleTable();
}

async function useCurrentPoseForHome() {
  const data = await apiPost('/api/pose', {});
  const pose = data.pose;
  if (!pose) return;
  document.getElementById('home_x').value = pose.x;
  document.getElementById('home_y').value = pose.y;
  updateDiagram();
}

let portsInitialized = false;

async function pollStatus() {
  try {
    const res = await fetch('/api/status');
    const st = await res.json();

    if (!portsInitialized && st.available_ports) {
      document.getElementById('portCustom').value = st.port || '/dev/ttyUSB0';
      populatePortDropdown(st.available_ports, st.port);
      portsInitialized = true;
    }

    if (st.active_sample_index !== activeSampleIdx) {
      activeSampleIdx = st.active_sample_index;
      renderSampleTable();
    }

    const badge = document.getElementById('connBadge');
    if (st.connected) {
      badge.textContent = st.simulation ? 'Simulation Mode' : `Connected (${st.port})`;
      badge.className = st.simulation ? 'badge sim' : 'badge connected';
    } else {
      badge.textContent = 'Disconnected';
      badge.className = 'badge';
    }

    document.getElementById('btnConnect').disabled = st.connected || st.running;
    document.getElementById('btnDisconnect').disabled = !st.connected;
    document.getElementById('btnStart').disabled = !st.connected || st.running;
    document.getElementById('btnAbort').disabled = !st.running;

    if (st.pose) {
      const j4 = (st.pose.j4 !== undefined) ? st.pose.j4 : st.pose.r;
      document.getElementById('poseX').textContent = st.pose.x.toFixed(1);
      document.getElementById('poseY').textContent = st.pose.y.toFixed(1);
      document.getElementById('poseZ').textContent = st.pose.z.toFixed(1);
      document.getElementById('poseJ4').textContent = j4.toFixed(1) + '°';
      document.getElementById('poseGrip').textContent = st.gripper_closed ? 'Closed' : 'Open';
      updateDiagram(st.pose, st.gripper_closed);
    }

    document.getElementById('stepText').textContent = 'Status: ' + st.current_step;
    document.getElementById('stepCount').textContent = `${st.step_index} / ${st.total_steps || 11}`;
    const pct = st.total_steps ? Math.round((st.step_index / st.total_steps) * 100) : 0;
    document.getElementById('progressBar').style.width = pct + '%';

    const logBox = document.getElementById('logBox');
    const logText = (st.logs || []).join('\n');
    if (logBox.textContent !== logText) {
      logBox.textContent = logText;
      logBox.scrollTop = logBox.scrollHeight;
    }
  } catch (e) {
    // Ignore transient poll errors
  }
}

renderSampleTable();
pollStatus();
setInterval(pollStatus, 350);
</script>
</body>
</html>
"""


class RequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def _send_json(self, payload: dict, status: int = 200):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            body = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif parsed.path == "/api/status":
            self._send_json(CONTROLLER.status_dict())
        else:
            self._send_json({"error": "Not found"}, status=404)

    def do_POST(self):
        parsed = urlparse(self.path)
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            data = json.loads(raw.decode("utf-8"))
        except Exception:
            data = {}

        try:
            if parsed.path == "/api/connect":
                CONTROLLER.connect(
                    port=data.get("port", "/dev/ttyUSB0"),
                    simulation=bool(data.get("simulation", False)),
                )
                self._send_json(CONTROLLER.status_dict())
            elif parsed.path == "/api/disconnect":
                CONTROLLER.disconnect()
                self._send_json(CONTROLLER.status_dict())
            elif parsed.path == "/api/start":
                CONTROLLER.start_exfoliation(data)
                self._send_json(CONTROLLER.status_dict())
            elif parsed.path == "/api/abort":
                CONTROLLER.request_abort()
                self._send_json(CONTROLLER.status_dict())
            elif parsed.path == "/api/pose":
                pose = CONTROLLER.get_pose()
                self._send_json({"pose": pose, **CONTROLLER.status_dict()})
            elif parsed.path == "/api/manual":
                CONTROLLER.manual_action(data.get("action", ""), data.get("params", {}))
                self._send_json(CONTROLLER.status_dict())
            else:
                self._send_json({"error": "Endpoint not found"}, status=404)
        except Exception as e:
            self._send_json({"error": str(e)}, status=400)


def main():
    parser = argparse.ArgumentParser(description="Dobot Magician Wet Exfoliation Web Interface")
    parser.add_argument("--host", default="127.0.0.1", help="Host address to bind (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port (default: 8080)")
    parser.add_argument("--no-browser", action="store_true", help="Do not open web browser automatically")
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), RequestHandler)
    url = f"http://{args.host}:{args.port}"
    print(f"Dobot Wet Exfoliation Interface running at {url}")
    print("Press Ctrl+C to stop the server and safely close the Dobot connection.")

    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server...")
    finally:
        CONTROLLER.disconnect()
        server.server_close()


if __name__ == "__main__":
    main()
