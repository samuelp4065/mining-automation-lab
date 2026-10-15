"""PCA9685 16-channel PWM driver, and an AngularServo-compatible servo on top.

Why this exists
---------------
The Orin Nano's three hardware-PWM header pins could not be made to drive their
pads -- see the bring-up notes -- so PWM generation moved off-board. The PCA9685
sits on I2C and generates all sixteen channels in hardware, which also removes
the "only three PWM pins for three joints" constraint entirely.

:class:`PCAServo` presents the same ``.angle`` / ``.detach()`` surface as
``gpiozero.AngularServo`` and :class:`~excavator_control.jetson_servo.JetsonServo`,
so the angles in the old Raspberry Pi scripts and everything
``excavator_kinematics.calibration`` produces carry over unchanged.

No third-party dependency: the chip needs only single- and multi-byte register
writes, which go straight to ``/dev/i2c-N`` via an ioctl. Installing smbus would
buy nothing.

Wiring
------
``VCC`` is the logic supply and sets the **output signal level**, while ``V+`` is
the separate servo power rail feeding the 3-pin headers. The L16-R wants 6 V on
V+. VCC may be 3.3 V or 5 V -- 3.3 V is sufficient, as these actuators already
ran from a Raspberry Pi's 3.3 V GPIO.
"""

import fcntl
import os
import time

__all__ = ["PCA9685", "PCAServo", "DEFAULT_BUS", "DEFAULT_ADDRESS"]

DEFAULT_BUS = 7          # header pins 3 (SDA) and 5 (SCL)
DEFAULT_ADDRESS = 0x40   # PCA9685 default; it also answers All Call on 0x70

_I2C_SLAVE = 0x0703

_MODE1 = 0x00
_MODE2 = 0x01
_LED0_ON_L = 0x06
_PRESCALE = 0xFE

_MODE1_ALLCALL = 0x01
_MODE1_SLEEP = 0x10
_MODE1_AI = 0x20         # auto-increment, needed for 4-byte channel writes
_MODE1_RESTART = 0x80

_MODE2_OUTDRV = 0x04     # totem-pole outputs, correct for driving servo inputs

#: The chip's internal oscillator. Prescale is derived from it.
_OSC_HZ = 25_000_000
#: 12-bit resolution: each period is divided into this many steps.
_STEPS = 4096


class PCA9685:
    """Minimal register-level driver for the PCA9685."""

    def __init__(self, bus=DEFAULT_BUS, address=DEFAULT_ADDRESS, frequency=50.0):
        self.address = address
        self._fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        fcntl.ioctl(self._fd, _I2C_SLAVE, address)

        self._write(_MODE1, _MODE1_AI | _MODE1_ALLCALL)
        self._write(_MODE2, _MODE2_OUTDRV)
        time.sleep(0.005)           # oscillator needs time after waking
        self.frequency = frequency

    def _write(self, register, *values):
        os.write(self._fd, bytes([register, *values]))

    def _read(self, register):
        os.write(self._fd, bytes([register]))
        return os.read(self._fd, 1)[0]

    @property
    def frequency(self):
        prescale = self._read(_PRESCALE)
        return _OSC_HZ / (_STEPS * (prescale + 1))

    @frequency.setter
    def frequency(self, hz):
        # Datasheet: prescale = round(osc / (4096 * rate)) - 1, and PRESCALE is
        # only writable while the chip is asleep.
        prescale = int(round(_OSC_HZ / (_STEPS * float(hz)))) - 1
        if not 3 <= prescale <= 255:
            raise ValueError(f"frequency {hz} Hz out of range for the PCA9685")

        old_mode = self._read(_MODE1)
        self._write(_MODE1, (old_mode & ~_MODE1_RESTART) | _MODE1_SLEEP)
        self._write(_PRESCALE, prescale)
        self._write(_MODE1, old_mode)
        time.sleep(0.005)
        self._write(_MODE1, old_mode | _MODE1_RESTART)

    def set_pulse_us(self, channel, microseconds):
        """Emit a pulse of this width on a channel, once per period."""
        period_us = 1_000_000.0 / self.frequency
        counts = int(round(_STEPS * microseconds / period_us))
        counts = max(0, min(_STEPS - 1, counts))
        self.set_counts(channel, 0, counts)

    def set_counts(self, channel, on, off):
        base = _LED0_ON_L + 4 * channel
        self._write(base, on & 0xFF, on >> 8, off & 0xFF, off >> 8)

    def off(self, channel):
        """Stop pulsing a channel entirely (the full-OFF bit)."""
        self.set_counts(channel, 0, 0x1000)

    def all_off(self):
        for channel in range(16):
            self.off(channel)

    def close(self):
        if self._fd is not None:
            try:
                self.all_off()
            finally:
                os.close(self._fd)
                self._fd = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


class PCAServo:
    """One RC actuator on a PCA9685 channel, with AngularServo semantics.

    Args mirror ``gpiozero.AngularServo`` so calibration output and the legacy
    scripts' angles need no translation. Pulse widths are in seconds there, so
    they are here too.
    """

    def __init__(self, controller, channel,
                 min_pulse_width=0.001, max_pulse_width=0.002,
                 min_angle=-90.0, max_angle=90.0, initial_angle=None):
        self._pca = controller
        self.channel = channel
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)
        self._min_us = min_pulse_width * 1e6
        self._max_us = max_pulse_width * 1e6
        self._angle = None

        if initial_angle is not None:
            self.angle = initial_angle

    @property
    def angle(self):
        """Last commanded angle, or ``None`` while detached.

        Commanded, not measured -- these actuators are open loop and nothing
        here can report where the arm actually is.
        """
        return self._angle

    @angle.setter
    def angle(self, value):
        if value is None:
            self.detach()
            return

        value = float(value)
        if not self.min_angle <= value <= self.max_angle:
            # Raise rather than clamp: clamping lets a bad calibration constant
            # hold an actuator against its end stop, which reads as a reached
            # position and is the silent saturation check_joint_limits() guards.
            raise ValueError(
                f"angle {value} outside [{self.min_angle}, {self.max_angle}] "
                f"on channel {self.channel}"
            )

        fraction = (value - self.min_angle) / (self.max_angle - self.min_angle)
        self._pca.set_pulse_us(
            self.channel, self._min_us + fraction * (self._max_us - self._min_us)
        )
        self._angle = value

    def detach(self):
        """Stop driving. Safe to call repeatedly, so it belongs in a finally."""
        self._pca.off(self.channel)
        self._angle = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.detach()
        return False
