"""RC linear actuator PWM for the Jetson, with gpiozero's AngularServo interface.

Why this module exists
----------------------
Every actuator script in this repo was written against ``gpiozero.AngularServo``
on a Raspberry Pi. gpiozero does not run on a Jetson -- its pin factories expect
Broadcom hardware -- so on the Orin Nano all of that code is dead. ``Jetson.GPIO``
is what works here, but it exposes a bare ``GPIO.PWM`` whose duty cycle is a
**percent of the frame**, not a pulse width, and it has no angle abstraction and
no ``detach()``.

:class:`JetsonServo` closes that gap: it accepts the same constructor arguments
and the same ``.angle`` / ``.detach()`` surface as ``AngularServo``, so a script
that drove the arm on the Pi runs here after changing only its import and its pin
numbers.

Pins moved, and that is not optional
------------------------------------
The Pi wiring (BCM 17 boom, 27 stick, 22 bucket) cannot be reproduced on this
header. On the Orin Nano's 40-pin header those land on UART1_RTS and SPI1_SCK,
which have no path to a PWM controller. Only three pins on the whole header do,
and the actuators are now on all three -- see :data:`BOOM_PIN` and friends.

Pin numbers here are **BOARD** (physical position on the header), because that is
the one numbering scheme that is unambiguous across the four this platform uses
(BOARD / BCM / CVM / TEGRA_SOC). The CVM names are noted alongside each constant
since that is what the Jetson pinout diagrams label.

These pins only carry PWM if jetson-io has muxed them. If a constructor raises,
check that first: ``sudo /opt/nvidia/jetson-io/jetson-io.py``, enable pins 15,
32 and 33, save, reboot.

Open-loop means release on exit
-------------------------------
These actuators have no feedback, so a commanded position that is never released
keeps driving. Leaving one driven on exit stalls it against its end stop. That is
why :meth:`detach` disables the PWM output rather than commanding some "safe"
angle, and why every caller must guarantee it runs -- use the context manager, or
``try``/``finally``.
"""

import os

# Jetson.GPIO 2.1.7 identifies the board from its device-tree compatible string,
# and this devkit reports "nvidia,p3768-0000+p3767-0005-super". The library's
# table only lists the string without the "-super" suffix, so the Super devkit is
# unrecognised and the import dies with "Could not determine Jetson model".
# Overriding by name is a supported path in its own get_model(). Must be set
# before the import below, and must not clobber a value the user already chose.
os.environ.setdefault("JETSON_MODEL_NAME", "JETSON_ORIN_NANO")

import Jetson.GPIO as GPIO  # noqa: E402  (see the setdefault above)

__all__ = [
    "JetsonServo", "BOOM_PIN", "ARM_PIN", "BUCKET_PIN",
    "make_arm_servos", "detach_all",
]

#: Boom actuator. CVM name GPIO13, PWM controller 32c0000.pwm.
BOOM_PIN = 33
#: Stick/arm actuator. CVM name GPIO07, PWM controller 32e0000.pwm.
ARM_PIN = 32
#: Bucket actuator. CVM name GPIO12, PWM controller 3280000.pwm.
BUCKET_PIN = 15


class JetsonServo:
    """An ``AngularServo``-compatible RC actuator on a Jetson hardware PWM pin.

    Args:
        pin: BOARD pin number. Must be one muxed to a PWM controller.
        min_pulse_width: pulse length at ``min_angle``, seconds.
        max_pulse_width: pulse length at ``max_angle``, seconds.
        frame_width: period of one frame, seconds. The repo-wide value of
            0.0021 is deliberate and not a typo for 0.021 -- a frame barely
            longer than the pulse gives finer control and less torque. It
            implies a ~476 Hz carrier and a 48-95% duty range, where a
            conventional 50 Hz servo frame would be 5-10%.
        initial_angle: angle to hold immediately, or ``None`` to start
            detached. ``None`` matches how the dig scripts construct these:
            nothing moves until something is commanded.
        min_angle, max_angle: the commanded angle range. The default
            [-90, 90] is the range ``excavator_kinematics.calibration``
            produces, and the range whose ends are the mechanical stops.
    """

    def __init__(self, pin, min_pulse_width=0.001, max_pulse_width=0.002,
                 frame_width=0.0021, initial_angle=None,
                 min_angle=-90.0, max_angle=90.0):
        if max_angle == min_angle:
            raise ValueError("min_angle and max_angle must differ")
        if not 0 < max_pulse_width <= frame_width:
            raise ValueError(
                f"max_pulse_width {max_pulse_width} must be positive and fit "
                f"inside frame_width {frame_width}"
            )

        self.pin = pin
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)
        self._min_pulse = float(min_pulse_width)
        self._max_pulse = float(max_pulse_width)
        self._frame = float(frame_width)
        self._angle = None
        self._started = False

        # setmode() raises if a different mode is already active, so adopt the
        # existing one rather than fighting a module that got there first.
        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BOARD)
        elif GPIO.getmode() != GPIO.BOARD:
            raise RuntimeError(
                f"Jetson.GPIO is already in mode {GPIO.getmode()}; this module "
                f"addresses pins by BOARD number"
            )

        self._pwm = GPIO.PWM(pin, 1.0 / self._frame)

        if initial_angle is not None:
            self.angle = initial_angle

    def _angle_to_duty_percent(self, angle):
        fraction = (angle - self.min_angle) / (self.max_angle - self.min_angle)
        pulse = self._min_pulse + fraction * (self._max_pulse - self._min_pulse)
        return 100.0 * pulse / self._frame

    @property
    def angle(self):
        """Commanded angle in degrees, or ``None`` while detached.

        This is the last value *commanded*, not a measurement. These actuators
        are open-loop; nothing here can tell you where the arm actually is.
        """
        return self._angle

    @angle.setter
    def angle(self, value):
        if value is None:
            self.detach()
            return

        value = float(value)
        if not self.min_angle <= value <= self.max_angle:
            # Deliberately an error, not a clamp. Clamping would let a bad
            # calibration constant drive the actuator into its end stop and
            # hold it there, which looks like a reached position and is the
            # exact silent saturation check_joint_limits() exists to prevent.
            raise ValueError(
                f"angle {value} outside [{self.min_angle}, {self.max_angle}] "
                f"on pin {self.pin}"
            )

        duty = self._angle_to_duty_percent(value)
        if self._started:
            self._pwm.ChangeDutyCycle(duty)
        else:
            self._pwm.start(duty)
            self._started = True
        self._angle = value

    def detach(self):
        """Stop driving the actuator, leaving it unpowered and back-driveable.

        Safe to call repeatedly, including on an already-detached servo, so it
        can go in a ``finally`` without guarding.
        """
        if self._started:
            self._pwm.stop()
            self._started = False
        self._angle = None

    def close(self):
        """Detach and release the PWM channel."""
        self.detach()
        self._pwm = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def make_arm_servos(**kwargs):
    """Construct the boom, stick and bucket actuators on their PWM pins.

    Returns:
        ``(boom, arm, bucket)``, each detached until commanded.

    Every pre-ROS script re-declared these three identically; this is that
    declaration, in one place. Extra keyword arguments go to each
    :class:`JetsonServo`.
    """
    return (
        JetsonServo(BOOM_PIN, **kwargs),
        JetsonServo(ARM_PIN, **kwargs),
        JetsonServo(BUCKET_PIN, **kwargs),
    )


def detach_all(*servos):
    """Detach every servo given, even if one of them raises on the way.

    A plain loop would skip the remaining actuators if an early ``detach()``
    failed, leaving them driven -- which is the failure this is here to prevent.
    """
    error = None
    for servo in servos:
        try:
            servo.detach()
        except Exception as exc:  # keep going; the rest still need releasing
            error = error or exc
    if error is not None:
        raise error
