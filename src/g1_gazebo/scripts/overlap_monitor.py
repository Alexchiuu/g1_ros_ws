#!/usr/bin/env python3
"""Warn when the robot's own links interpenetrate.

Position control (see position_bridge.py) drives joints straight to their
commanded angle regardless of what's in the way, so nothing in this sim
stops two of the robot's own links from being commanded into each other --
mass and contact physics still act on the free-floating base and against
external objects, but a kinematically-set joint doesn't feel resistance.
This node is the substitute for that: it doesn't try to prevent overlap, it
just tells you when it's happening.

Gazebo's world-level contact manager already computes every contact (that's
how the physics engine finds collisions to resolve in the first place) and
publishes it on the world's own transport topic -- no per-link bumper
sensors needed. There's no Python protobuf binding for this readily
available here, so this shells out to `gz topic -e <topic>`, which prints
the same messages as plain indented text, and parses that directly rather
than pulling in a full gazebo-transport dependency for one topic.

Self-contact only (both collision names start with "g1::"; a g1-vs-
ground_plane contact is normal standing/lying, not an overlap) and only
above DEPTH_THRESHOLD (interpenetration distance in meters) counts as
something worth a warning -- sub-millimeter contact-solver noise floods the
topic constantly and isn't a real overlap. Each link pair is throttled to
one warning per WARN_PERIOD seconds so a sustained overlap (e.g. a
hand resting on a leg) doesn't spam the log.

The contacts topic reports every contact point on every physics step (up to
1000 Hz), which is far more resolution than a log warning needs and was
expensive enough to noticeably load the machine when streamed continuously
(a sustained ~95% of a core just parsing text, on top of everything else
gzserver/rviz/etc. are already doing). A warning system only cares whether
an overlap is still happening a moment ago, so instead of one long-lived
`gz topic -e` subprocess streamed and parsed forever, a background thread
takes a short SNAPSHOT_DURATION sample every SNAPSHOT_PERIOD seconds and
throws the subprocess away in between -- same detection fidelity for
anything lasting longer than a step, a fraction of the CPU cost.
"""
import re
import select
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

DEPTH_THRESHOLD = 0.005  # meters
WARN_PERIOD = 2.0  # seconds, per link pair
SNAPSHOT_PERIOD = 1.0  # seconds between contact-topic samples
SNAPSHOT_DURATION = 0.3  # seconds to sample the topic each snapshot


def link_name(collision_name):
    # "g1::right_elbow_link::right_elbow_link_collision" -> "right_elbow_link"
    parts = collision_name.split("::")
    return parts[1] if len(parts) > 1 else collision_name


class OverlapMonitor(Node):
    def __init__(self):
        super().__init__("g1_overlap_monitor")
        self.declare_parameter("world_name", "g1_world")
        world_name = self.get_parameter("world_name").value
        self._topic = f"/gazebo/{world_name}/physics/contacts"
        self._last_warned = {}
        self._in_contact = False
        self._depth = 0
        self._brace_depth = 0
        self._collision1 = None
        self._collision2 = None
        self._stop = threading.Event()
        self._sampler = threading.Thread(target=self._sample_loop, daemon=True)
        self._sampler.start()
        self.get_logger().info(f"watching {self._topic} for self-collision overlap")

    def _sample_loop(self):
        while not self._stop.is_set():
            proc = subprocess.Popen(
                ["gz", "topic", "-e", self._topic],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
            )
            deadline = time.monotonic() + SNAPSHOT_DURATION
            self._in_contact = False
            try:
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    # readline() has no timeout of its own, and when there's
                    # simply no contact happening right now (the common,
                    # good case) there's nothing to read for the rest of the
                    # window -- select() first so a quiet snapshot ends at
                    # the deadline instead of blocking on the next line
                    # whenever that arrives.
                    ready, _, _ = select.select([proc.stdout], [], [], remaining)
                    if not ready:
                        break
                    line = proc.stdout.readline()
                    if not line:
                        break
                    self._feed_line(line)
            finally:
                proc.kill()
                proc.wait()
            self._stop.wait(SNAPSHOT_PERIOD)

    def _feed_line(self, line):
        stripped = line.strip()
        if stripped == "contact {":
            self._in_contact = True
            self._brace_depth = 1
            self._collision1 = None
            self._collision2 = None
            self._depth = None
            return
        if not self._in_contact:
            return
        self._brace_depth += line.count("{") - line.count("}")
        if self._brace_depth <= 0:
            self._in_contact = False
            self._maybe_warn()
            return
        # only the contact block's own direct fields matter, not nested
        # position/normal/wrench sub-messages
        m = re.match(r'collision1: "([^"]+)"', stripped)
        if m:
            self._collision1 = m.group(1)
        m = re.match(r'collision2: "([^"]+)"', stripped)
        if m:
            self._collision2 = m.group(1)
        m = re.match(r"depth: ([-0-9.e+]+)", stripped)
        if m and self._depth is None:
            self._depth = float(m.group(1))

    def _maybe_warn(self):
        if not self._collision1 or not self._collision2 or self._depth is None:
            return
        if not (self._collision1.startswith("g1::") and self._collision2.startswith("g1::")):
            return  # contact with the ground/environment, not self-overlap
        if self._depth < DEPTH_THRESHOLD:
            return
        a, b = sorted((link_name(self._collision1), link_name(self._collision2)))
        key = (a, b)
        now = time.monotonic()
        if now - self._last_warned.get(key, -1e9) < WARN_PERIOD:
            return
        self._last_warned[key] = now
        self.get_logger().warn(
            f"overlap: {a} <-> {b} (penetration {self._depth * 1000:.1f} mm)")

    def destroy_node(self):
        self._stop.set()
        self._sampler.join(timeout=SNAPSHOT_DURATION + 1.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = OverlapMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
