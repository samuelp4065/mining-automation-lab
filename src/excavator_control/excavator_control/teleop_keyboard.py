"""Keyboard teleop for the tracks.

Layout matches teleop_twist_keyboard so muscle memory carries over:

    u  i  o          i     drive forward            j / l  spin in place
    j  k  l          ,     drive backward           k      stop
    m  ,  .          u / o forward arc              m / .  reverse arc

Arc keys are specified by INNER TRACK RATIO
-------------------------------------------
The two tables are parameterised differently, on purpose.

``MOVE_BINDINGS`` holds body twists -- (forward, turn) multiples of the speed
and turn settings -- which is what ROS actually consumes.

``ARC_BINDINGS`` instead says which way the machine travels, which track is on
the inside of the curve, and how fast that inner track runs as a fraction of
the outer one. So ``("forward", "left", 0.2)`` reads as *drive forward with
the left track at 20% of the right*, and the twist is derived from it.

That is the natural way to describe a curve on a tracked machine, and it makes
reversing a track impossible by construction rather than by clamping: the
inner track is ``ratio * outer``, so for any ratio in [0, 1] it is only ever
slower than the outer track, never negative. Ratio 1.0 is straight ahead; 0.0
stops the inner track and pivots about it.

The outer track always runs at the current ``speed``, so the machine naturally
slows through a turn -- its centre speed is ``speed * (1 + ratio) / 2``.

j and l are ordinary twists and still counter-rotate the tracks, so spinning on
the spot is unaffected. The ``turn`` parameter applies only to those; arc turn
rates come from the geometry instead.

    ros2 run excavator_control teleop_keyboard
    ros2 run excavator_control teleop_keyboard --ros-args -p speed:=0.15

Speed keys: q/z scale both, w/x speed only, e/c turn only.

Note on the watchdog: motor_node stops the tracks when /cmd_vel goes quiet. A
held key does not repeat instantly -- the terminal waits out its key-repeat
delay first -- so this node republishes at ``repeat_rate`` to keep the tracks
moving smoothly while a key is held.
"""

import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .drive_config import DEFAULT_TRACK_SEPARATION, declare_float

#: Body twists, as (forward, turn) multiples of the speed/turn settings.
MOVE_BINDINGS = {
    "i": (1.0, 0.0),      # forward
    ",": (-1.0, 0.0),     # backward
    "j": (0.0, 1.0),      # spin left, full rate
    "l": (0.0, -1.0),     # spin right, full rate
    "k": (0.0, 0.0),      # stop
    " ": (0.0, 0.0),      # stop
}

#: Arcs, as (travel, inner track, inner ratio). The inner track runs at
#: ``ratio`` times the outer track's speed: 1.0 is straight, 0.0 stops the
#: inner track and pivots about it. Lower the ratio for a tighter curve.
ARC_BINDINGS = {
    "u": ("forward", "left", 0.5),     # forward, left track at 50% of right
    "o": ("forward", "right", 0.5),    # forward, right track at 50% of left
    ".": ("reverse", "left", 0.2),     # reverse, left track at 20% of right
    "m": ("reverse", "right", 0.2),    # reverse, right track at 20% of left
}

SPEED_BINDINGS = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1.0),
    "x": (0.9, 1.0),
    "e": (1.0, 1.1),
    "c": (1.0, 0.9),
}

BANNER = """
Excavator track teleop
----------------------
   u   i   o        i       forward            j / l   spin in place
   j   k   l        ,       backward           k / spc stop
   m   ,   .        u / o   forward arc
                    m / .   reverse arc

Arc keys slow the inner track to a fraction of the outer one, so they curve
rather than pivot and never drive a track backwards.

q/z  scale both     w/x  speed only     e/c  turn only (spins)
CTRL-C to quit
"""


def arc_to_twist(travel, inner, ratio, speed, separation):
    """Inner-track ratio -> (linear.x, angular.z).

    With the outer track at ``speed`` and the inner at ``ratio * speed``:

        v = speed * (1 + ratio) / 2           centre of the machine
        w = speed * (1 - ratio) / separation  signed by which track is inner

    Both tracks keep the sign of ``speed`` for any ratio in [0, 1], which is
    what makes reversal impossible here rather than merely clamped.
    """
    ratio = min(max(ratio, 0.0), 1.0)
    outer = speed if travel == "forward" else -speed

    v = outer * (1.0 + ratio) / 2.0
    w = outer * (1.0 - ratio) / separation

    # Slowing the left track turns the machine left (positive angular.z);
    # slowing the right turns it right. Reverse travel flips both, which the
    # sign of `outer` already carries.
    if inner == "right":
        w = -w
    return v, w


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")

        self.speed = declare_float(self, "speed", 0.3)
        self.turn = declare_float(self, "turn", 0.8)
        repeat_rate = declare_float(self, "repeat_rate", 10.0)
        self.separation = declare_float(self, "track_separation",
                                        DEFAULT_TRACK_SEPARATION)

        # Either ("move", forward, turn) or ("arc", travel, inner, ratio).
        # Stored as the binding rather than a finished twist, so changing
        # speed at runtime re-derives the command.
        self.command = ("move", 0.0, 0.0)
        self.lock = threading.Lock()

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        if repeat_rate > 0.0:
            self.create_timer(1.0 / repeat_rate, self.publish)

    def set_move(self, forward, turn):
        with self.lock:
            self.command = ("move", float(forward), float(turn))

    def set_arc(self, travel, inner, ratio):
        with self.lock:
            self.command = ("arc", travel, inner, float(ratio))

    def current_twist(self):
        with self.lock:
            command = self.command

        if command[0] == "arc":
            _, travel, inner, ratio = command
            return arc_to_twist(travel, inner, ratio, self.speed, self.separation)

        _, forward, turn = command
        return forward * self.speed, turn * self.turn

    def publish(self):
        v, w = self.current_twist()
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    def stop(self):
        self.set_move(0.0, 0.0)
        self.publish()

    def status(self):
        return f"speed {self.speed:.2f}   turn {self.turn:.2f}"


def read_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    settings = termios.tcgetattr(sys.stdin)
    print(BANNER)
    print(node.status())

    try:
        while True:
            key = read_key(settings)

            if key in MOVE_BINDINGS:
                forward, turn = MOVE_BINDINGS[key]
                node.set_move(forward, turn)

            elif key in ARC_BINDINGS:
                travel, inner, ratio = ARC_BINDINGS[key]
                node.set_arc(travel, inner, ratio)

            elif key in SPEED_BINDINGS:
                speed_scale, turn_scale = SPEED_BINDINGS[key]
                node.speed *= speed_scale
                node.turn *= turn_scale
                print(node.status())

            elif key == "\x03":            # CTRL-C
                break

            else:
                # Any unrecognised key stops the machine rather than letting
                # the last command continue.
                node.set_move(0.0, 0.0)

    finally:
        # Always leave the tracks stopped and the terminal usable, even if the
        # loop raised.
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
