"""Tkinter GUI for direct control of the G1 neck via neck_server.py's ZMQ
command socket (tcp://<host>:5558). Two sliders (yaw, pitch, in degrees
relative to the calibrated zero pose -- see real_state.launch.py's
neck_yaw_zero_ticks/neck_pitch_zero_ticks) sent immediately as they move.

Requires neck_server.py running on the robot (started by run.sh's neck
pane) -- not the old neck.py, which locks/releases torque instead of
tracking a live goal.
"""
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

import zmq

STATE_FMT = "<Bdii"
STATE_NBYTES = struct.calcsize(STATE_FMT)
CMD_FMT = "<dd"

DOF_LABELS = ["yaw (pan)", "pitch (tilt)"]
# Matches neck_server.py's default --clamp-deg; mechanical limits haven't
# been measured, so widen both together once you know the real safe range.
DOF_LIMIT_DEG = 45.0
SEND_HZ = 20.0
SEND_PERIOD = 1.0 / SEND_HZ


class NeckLink:
    """Owns the ZMQ sockets talking to neck_server.py on the robot."""

    def __init__(self, host, cmd_port, state_port, yaw_zero_ticks, pitch_zero_ticks,
                 yaw_sign, pitch_sign):
        self.host = host
        self.cmd_port = cmd_port
        self.state_port = state_port
        self.zero_ticks = (yaw_zero_ticks, pitch_zero_ticks)
        self.sign = (yaw_sign, pitch_sign)
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

    def send(self, yaw_deg, pitch_deg):
        """Send a goal command. Returns True on ACK, False otherwise."""
        payload = struct.pack(CMD_FMT, yaw_deg, pitch_deg)
        try:
            self.cmd_sock.send(payload)
            reply = self.cmd_sock.recv()
            return reply == b"\x01"
        except zmq.ZMQError:
            self._open_cmd_sock()
            return False

    def read_current_deg(self, timeout_s=1.0):
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
        _locked, _t, id1, id2 = struct.unpack(STATE_FMT, payload)
        yaw_deg = self.sign[0] * (id1 - self.zero_ticks[0]) * (360.0 / 4096.0)
        pitch_deg = self.sign[1] * (id2 - self.zero_ticks[1]) * (360.0 / 4096.0)
        return yaw_deg, pitch_deg


class NeckGui:
    def __init__(self, root, link: NeckLink):
        self.link = link
        self.root = root
        root.title("Neck Controller")

        self._lock = threading.Lock()
        self._values = [0.0, 0.0]
        self._dirty = threading.Event()

        seed = link.read_current_deg()
        if seed is not None:
            self._values = list(seed)

        self.status_var = tk.StringVar(value="starting...")
        self.sliders = []

        container = ttk.Frame(root, padding=10)
        container.grid(row=0, column=0, sticky="nsew")

        for i, label in enumerate(DOF_LABELS):
            ttk.Label(container, text=label).grid(row=i, column=0, sticky="w")
            var = tk.DoubleVar(value=self._values[i])
            scale = ttk.Scale(
                container, from_=-DOF_LIMIT_DEG, to=DOF_LIMIT_DEG,
                orient="horizontal", length=260, variable=var,
                command=lambda v, idx=i, var=var: self._on_slider(idx, var))
            scale.grid(row=i, column=1, padx=6)
            val_label = ttk.Label(container, text=f"{self._values[i]:5.1f} deg", width=9)
            val_label.grid(row=i, column=2)
            self.sliders.append((scale, var, val_label))

        ttk.Label(container, textvariable=self.status_var).grid(
            row=2, column=0, columnspan=3, pady=(10, 0), sticky="w")
        ttk.Button(container, text="Re-center to current pose",
                   command=self._resync).grid(row=3, column=0, columnspan=3, pady=(6, 0))
        ttk.Button(container, text="Zero (look straight)",
                   command=self._zero).grid(row=4, column=0, columnspan=3, pady=(4, 0))

        self._sender_stop = threading.Event()
        self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self._sender_thread.start()
        root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_slider(self, idx, var):
        with self._lock:
            self._values[idx] = var.get()
        self.sliders[idx][2].config(text=f"{var.get():5.1f} deg")
        self._dirty.set()

    def _resync(self):
        seed = self.link.read_current_deg()
        if seed is None:
            self.status_var.set("resync failed: no state frame received")
            return
        with self._lock:
            self._values = list(seed)
        for i, (scale, var, val_label) in enumerate(self.sliders):
            var.set(seed[i])
            val_label.config(text=f"{seed[i]:5.1f} deg")

    def _zero(self):
        with self._lock:
            self._values = [0.0, 0.0]
        for i, (scale, var, val_label) in enumerate(self.sliders):
            var.set(0.0)
            val_label.config(text="0.0 deg")
        self._dirty.set()

    def _sender_loop(self):
        while not self._sender_stop.is_set():
            triggered = self._dirty.wait(timeout=0.5)
            if not triggered:
                continue
            self._dirty.clear()
            with self._lock:
                yaw, pitch = self._values
            ok = self.link.send(yaw, pitch)
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
    parser.add_argument("--cmd-port", type=int, default=5558)
    parser.add_argument("--state-port", type=int, default=5557)
    parser.add_argument("--yaw-zero-ticks", type=int, default=2023)
    parser.add_argument("--pitch-zero-ticks", type=int, default=3688)
    parser.add_argument("--yaw-sign", type=int, default=1, choices=[-1, 1])
    parser.add_argument("--pitch-sign", type=int, default=1, choices=[-1, 1])
    args = parser.parse_args()

    link = NeckLink(args.host, args.cmd_port, args.state_port,
                     args.yaw_zero_ticks, args.pitch_zero_ticks, args.yaw_sign, args.pitch_sign)
    root = tk.Tk()
    NeckGui(root, link)
    root.mainloop()


if __name__ == "__main__":
    main()
