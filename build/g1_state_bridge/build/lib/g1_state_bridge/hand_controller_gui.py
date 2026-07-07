"""Tkinter GUI for direct control of the Aero hands via aero_hand_relay's
ZMQ command socket (tcp://<host>:5555). Sends commands immediately as
sliders move -- there is no arm/enable switch or motion ramping.

The hand only has 7 real motors per side (see MOTOR_MAX in the vendor's
g1_dex3_example-style relay and convert_seven_joints_to_sixteen in
aero_hand.py); the other 9 "joints" per hand shown in RViz are mechanically
ganged to one of these 7, so this GUI exposes exactly 7 sliders/hand -- one
per real motor -- rather than 16 sliders that would silently fight each
other.
"""

import math
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

import zmq

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi

# Order/limits must match AeroHandConstants in aero_open_sdk on the robot.
JOINT_NAMES_FULL16 = [
    "thumb_cmc_abd", "thumb_cmc_flex", "thumb_mcp", "thumb_ip",
    "index_mcp_flex", "index_pip", "index_dip",
    "middle_mcp_flex", "middle_pip", "middle_dip",
    "ring_mcp_flex", "ring_pip", "ring_dip",
    "pinky_mcp_flex", "pinky_pip", "pinky_dip",
]
JOINTS_PER_HAND = len(JOINT_NAMES_FULL16)  # 16
TOTAL_FLOATS = 2 * JOINTS_PER_HAND  # 32 (left + right)
CMD_FMT = f"<{TOTAL_FLOATS}d"
STATE_FMT = f"<BQd{TOTAL_FLOATS * 2}d"
STATE_NBYTES = struct.calcsize(STATE_FMT)

# The 7 real motors/hand, in the order set_joint_positions' 7-length form
# expects (see convert_seven_joints_to_sixteen), with their degree limits
# from AeroHandConstants.joint_lower_limits/joint_upper_limits.
DOF_LABELS = ["thumb_cmc_abd", "thumb_cmc_flex", "thumb_bend (mcp+ip)",
              "index", "middle", "ring", "pinky"]
DOF_LOWER_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
DOF_UPPER_DEG = [100.0, 55.0, 90.0, 90.0, 90.0, 90.0, 90.0]
# Index into JOINT_NAMES_FULL16 that represents each DOF's group, used when
# reading state back (all indices in a group carry the same value).
DOF_FULL16_REP_INDEX = [0, 1, 2, 4, 7, 10, 13]

SEND_HZ = 20.0
SEND_PERIOD = 1.0 / SEND_HZ


def compact7_deg_to_full16_rad(compact_deg):
    c = [v * DEG_TO_RAD for v in compact_deg]
    return [
        c[0], c[1], c[2], c[2],
        c[3], c[3], c[3],
        c[4], c[4], c[4],
        c[5], c[5], c[5],
        c[6], c[6], c[6],
    ]


class HandLink:
    """Owns the ZMQ sockets talking to aero_hand_relay on the robot."""

    def __init__(self, host, cmd_port, state_port):
        self.host = host
        self.cmd_port = cmd_port
        self.state_port = state_port
        self.ctx = zmq.Context.instance()
        self.cmd_sock = None
        self._open_cmd_sock()

    def _open_cmd_sock(self):
        if self.cmd_sock is not None:
            self.cmd_sock.close(0)
        self.cmd_sock = self.ctx.socket(zmq.REQ)
        self.cmd_sock.setsockopt(zmq.LINGER, 0)
        self.cmd_sock.setsockopt(zmq.RCVTIMEO, 300)
        self.cmd_sock.setsockopt(zmq.SNDTIMEO, 300)
        self.cmd_sock.connect(f"tcp://{self.host}:{self.cmd_port}")

    def send(self, left_compact_deg, right_compact_deg):
        """Send a command frame. Returns True on ACK, False otherwise."""
        left16 = compact7_deg_to_full16_rad(left_compact_deg)
        right16 = compact7_deg_to_full16_rad(right_compact_deg)
        payload = struct.pack(CMD_FMT, *(left16 + right16))
        try:
            self.cmd_sock.send(payload)
            reply = self.cmd_sock.recv()
            return reply == b"\x01"
        except zmq.ZMQError:
            # REQ socket state is now undefined (e.g. timed out mid-cycle) --
            # throw it away and reconnect fresh next time.
            self._open_cmd_sock()
            return False

    def read_current_compact(self, timeout_s=1.0):
        """Best-effort read of one state frame to seed slider positions."""
        sock = self.ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.setsockopt(zmq.RCVTIMEO, int(timeout_s * 1000))
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(f"tcp://{self.host}:{self.state_port}")
        try:
            payload = sock.recv()
        except zmq.Again:
            return None
        finally:
            sock.close(0)
        if len(payload) != STATE_NBYTES:
            return None
        fields = struct.unpack(STATE_FMT, payload)
        q_rad = fields[3:3 + TOTAL_FLOATS]
        left16, right16 = q_rad[:JOINTS_PER_HAND], q_rad[JOINTS_PER_HAND:]
        to_compact = lambda full16: [full16[i] * RAD_TO_DEG for i in DOF_FULL16_REP_INDEX]
        return to_compact(left16), to_compact(right16)


class HandGui:
    def __init__(self, root, link: HandLink):
        self.link = link
        self.root = root
        root.title("Aero Hand Controller")

        self._lock = threading.Lock()
        self._values = {"left": list(DOF_LOWER_DEG), "right": list(DOF_LOWER_DEG)}
        self._dirty = threading.Event()

        seed = link.read_current_compact()
        if seed is not None:
            self._values["left"], self._values["right"] = list(seed[0]), list(seed[1])

        self.status_var = tk.StringVar(value="starting...")
        self.sliders = {"left": [], "right": []}

        container = ttk.Frame(root, padding=10)
        container.grid(row=0, column=0, sticky="nsew")

        for col, side in enumerate(("left", "right")):
            frame = ttk.LabelFrame(container, text=f"{side.capitalize()} hand", padding=8)
            frame.grid(row=0, column=col, padx=8, sticky="n")
            for i, label in enumerate(DOF_LABELS):
                ttk.Label(frame, text=label).grid(row=i, column=0, sticky="w")
                var = tk.DoubleVar(value=self._values[side][i])
                scale = ttk.Scale(
                    frame, from_=DOF_LOWER_DEG[i], to=DOF_UPPER_DEG[i],
                    orient="horizontal", length=220, variable=var,
                    command=lambda v, s=side, idx=i, var=var: self._on_slider(s, idx, var))
                scale.grid(row=i, column=1, padx=6)
                val_label = ttk.Label(frame, text=f"{self._values[side][i]:5.1f} deg", width=9)
                val_label.grid(row=i, column=2)
                self.sliders[side].append((scale, var, val_label))

        ttk.Label(container, textvariable=self.status_var).grid(
            row=1, column=0, columnspan=2, pady=(10, 0), sticky="w")
        ttk.Button(container, text="Re-center to current pose",
                   command=self._resync).grid(row=2, column=0, columnspan=2, pady=(6, 0))

        self._sender_stop = threading.Event()
        self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self._sender_thread.start()
        root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_slider(self, side, idx, var):
        with self._lock:
            self._values[side][idx] = var.get()
        self.sliders[side][idx][2].config(text=f"{var.get():5.1f} deg")
        self._dirty.set()

    def _resync(self):
        seed = self.link.read_current_compact()
        if seed is None:
            self.status_var.set("resync failed: no state frame received")
            return
        for side, compact in zip(("left", "right"), seed):
            with self._lock:
                self._values[side] = list(compact)
            for i, (scale, var, val_label) in enumerate(self.sliders[side]):
                var.set(compact[i])
                val_label.config(text=f"{compact[i]:5.1f} deg")

    def _sender_loop(self):
        while not self._sender_stop.is_set():
            triggered = self._dirty.wait(timeout=0.5)
            if not triggered:
                continue
            self._dirty.clear()
            with self._lock:
                left = list(self._values["left"])
                right = list(self._values["right"])
            ok = self.link.send(left, right)
            status = "sent OK" if ok else "no ACK / timeout"
            self.root.after(0, lambda s=status: self.status_var.set(s))
            time.sleep(SEND_PERIOD)

    def _on_close(self):
        self._sender_stop.set()
        self.root.destroy()


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.123.164")
    parser.add_argument("--cmd-port", type=int, default=5555)
    parser.add_argument("--state-port", type=int, default=5556)
    args = parser.parse_args()

    link = HandLink(args.host, args.cmd_port, args.state_port)
    root = tk.Tk()
    HandGui(root, link)
    root.mainloop()


if __name__ == "__main__":
    main()
