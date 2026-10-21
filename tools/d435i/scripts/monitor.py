#!/usr/bin/env python3
"""Live mapping status line for map.sh. Runs inside the container.

Usage: monitor.py /ws/maps/<name>.db
"""
import math
import os
import shutil
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rtabmap_msgs.msg import Info, OdomInfo

DB = sys.argv[1] if len(sys.argv) > 1 else ""
STALE_S = 3.0  # no message for this long -> flag it


def hms(s):
    s = int(s)
    return f"{s // 3600:02d}:{s % 3600 // 60:02d}:{s % 60:02d}"


def human(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.0f} {unit}" if unit == "B" else f"{n:.1f} {unit}"
        n /= 1024


class Monitor(Node):
    def __init__(self):
        super().__init__("mapping_monitor")
        self.t0 = time.time()
        self.nodes = self.wm = 0
        self.loops = self.proximity = 0
        self.last_loop_t = None
        self.map_t = None
        self.odom_t = None
        self.inliers = self.features = None
        self.lost_now = False
        self.lost_events = 0
        self.dist = 0.0
        self.prev_xy = None

        self.create_subscription(Info, "/rtabmap/info", self.on_info, 10)
        self.create_subscription(OdomInfo, "/rtabmap/odom_info", self.on_odom_info, 10)
        self.create_subscription(Odometry, "/rtabmap/odom", self.on_odom, 10)
        self.create_timer(1.0, self.show)

    def elapsed(self):
        return time.time() - self.t0

    def event(self, text):
        # clear the status line, print the event, status line redraws next tick
        print(f"\r\033[K[{hms(self.elapsed())}] {text}", flush=True)

    def on_info(self, m):
        self.map_t = time.time()
        self.nodes = max(self.nodes, m.ref_id)
        self.wm = len(m.wm_state)
        if m.loop_closure_id > 0:
            self.loops += 1
            self.last_loop_t = self.elapsed()
            self.event(f"LOOP CLOSURE: node {m.ref_id} <-> node {m.loop_closure_id}")
        if m.proximity_detection_id > 0:
            self.proximity += 1

    def on_odom_info(self, m):
        self.odom_t = time.time()
        self.inliers, self.features = m.inliers, m.features
        if m.lost and not self.lost_now:
            self.lost_events += 1
            print("\a", end="")
            self.event("ODOMETRY LOST - stop, go back to the last good view, move slowly")
        elif not m.lost and self.lost_now:
            self.event("odometry recovered")
        self.lost_now = m.lost

    def on_odom(self, m):
        p = m.pose.pose.position
        if self.prev_xy is not None and not math.isnan(p.x):
            self.dist += math.hypot(p.x - self.prev_xy[0], p.y - self.prev_xy[1])
        if not math.isnan(p.x):
            self.prev_xy = (p.x, p.y)

    def show(self):
        now = time.time()
        try:
            db = human(os.path.getsize(DB))
        except OSError:
            db = "-"
        free = human(shutil.disk_usage(os.path.dirname(DB) or "/").free)

        if self.odom_t is None:
            odom = "waiting for odom"
        elif now - self.odom_t > STALE_S:
            odom = "NO ODOM DATA"
        elif self.lost_now:
            odom = "LOST"
        else:
            odom = f"ok inl={self.inliers}"
        if self.map_t is not None and now - self.map_t > STALE_S:
            odom += " | NO MAP UPDATES"

        last = hms(self.last_loop_t) if self.last_loop_t is not None else "-"
        line = (f"{hms(self.elapsed())} | nodes {self.nodes} (WM {self.wm}) | "
                f"loops {self.loops} (last {last}) | odom {odom} "
                f"lost x{self.lost_events} | dist {self.dist:.1f} m | "
                f"db {db} | free {free}   [q = stop]")
        print("\r\033[K" + line, end="", flush=True)


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
