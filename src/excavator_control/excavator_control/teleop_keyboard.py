"""Keyboard teleop for the tracks, with gentler arc keys.

Replaces teleop_twist_keyboard for this machine. The layout is deliberately
the same so muscle memory carries over, with one difference:

    u  i  o          i     drive forward            j / l  spin in place
    j  k  l          ,     drive backward           k      stop
    m  ,  .          u / o forward, gentle arc      m / .  reverse, gentle arc

The stock teleop drives the arc keys at full turn *and* full forward at once,
which on a skid-steer means the inside track slips hard and the machine swings
far more than intended. Here the arc keys scale their turn component by
``arc_gain`` (default 0.4), so u and o curve instead of swinging. j and l still
spin at the full rate, so nothing is lost.

The same gain applies to the reverse arcs m and . -- having the forward and
reverse diagonals behave differently would be a nasty surprise mid-manoeuvre.

    ros2 run excavator_control teleop_keyboard
    ros2 run excavator_control teleop_keyboard --ros-args -p arc_gain:=0.25

Speed keys (as upstream): q/z scale both, w/x speed only, e/c turn only.

Note on the watchdog: motor_node stops the tracks when /cmd_vel goes quiet. A
held key does not repeat instantly -- the terminal waits out its key-repeat
delay first -- so this node republishes the current command at ``repeat_rate``
to keep the tracks moving smoothly while a key is held.
"""

import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

#: Full-rate moves, as (forward, turn) multiples of the speed/turn settings.
MOVE_BINDINGS = {
    "i": (1.0, 0.0),      # forward
    ",": (-1.0, 0.0),     # backward
    "j": (0.0, 1.0),      # spin left, full rate
    "l": (0.0, -1.0),     # spin right, full rate
    "k": (0.0, 0.0),      # stop
    " ": (0.0, 0.0),      # stop
}

#: Arc moves. The turn component is multiplied by arc_gain, so these curve
#: rather than swinging the machine around. Kept in a separate table so the
#: gain applies to exactly these four keys and nothing else.
ARC_BINDINGS = {
    "u": (1.0, 1.0),      # forward, arcing left
    "o": (1.0, -1.0),     # forward, arcing right
    ".": (-1.0, 1.0),     # reverse, arcing left
    "m": (-1.0, -1.0),    # reverse, arcing right
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
   u   i   o        i       forward              j / l   spin in place
   j   k   l        ,       backward             k / spc stop
   m   ,   .        u / o   forward, gentle arc
                    m / .   reverse, gentle arc

q/z  scale both     w/x  speed only     e/c  turn only
CTRL-C to quit
"""


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")

        self.declare_parameter("speed", 0.3)
        self.declare_parameter("turn", 0.8)
        # Fraction of a full turn applied by the arc keys. 1.0 reproduces the
        # stock teleop_twist_keyboard behaviour; lower is gentler.
        self.declare_parameter("arc_gain", 0.4)
        self.declare_parameter("repeat_rate", 10.0)

        self.speed = self.get_parameter("speed").value
        self.turn = self.get_parameter("turn").value
        self.arc_gain = self.get_parameter("arc_gain").value
        repeat_rate = self.get_parameter("repeat_rate").value

        self.x = 0.0
        self.th = 0.0
        self.lock = threading.Lock()

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        if repeat_rate > 0.0:
            self.create_timer(1.0 / repeat_rate, self.publish)

    def set_command(self, x, th):
        with self.lock:
            self.x = float(x)
            self.th = float(th)

    def publish(self):
        with self.lock:
            x, th = self.x, self.th
        msg = Twist()
        msg.linear.x = x * self.speed
        msg.angular.z = th * self.turn
        self.pub.publish(msg)

    def stop(self):
        self.set_command(0.0, 0.0)
        self.publish()

    def status(self):
        return (f"speed {self.speed:.2f}   turn {self.turn:.2f}   "
                f"arc_gain {self.arc_gain:.2f}")


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
                x, th = MOVE_BINDINGS[key]
                node.set_command(x, th)

            elif key in ARC_BINDINGS:
                x, th = ARC_BINDINGS[key]
                node.set_command(x, th * node.arc_gain)

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
                node.set_command(0.0, 0.0)

    finally:
        # Always leave the tracks stopped and the terminal usable, even if the
        # loop raised.
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
