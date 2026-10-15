#!/usr/bin/env python3
"""Measure a PWM pin's real output by looping it back into a GPIO input.

Why this is needed
------------------
Reading ``/sys/class/pwm/.../duty_cycle`` back only proves the PWM *controller*
is configured. The controller runs whether or not the pinmux routes it to a
header pad, so a perfect sysfs readback is consistent with a pin that is
electrically dead. This tool closes that gap without a scope: jumper the PWM pin
to a spare input pin and sample it.

It splits the two failure modes that look identical from the outside -- "the
actuator does not move":

* **Pad toggling at the right duty** -> the Jetson side is fine. The fault is
  downstream: signal level (the L16-R wants 5 V, this header drives 3.3 V),
  actuator power, or a missing common ground.
* **Pad not toggling** -> the fault is on the Jetson side, almost certainly the
  pinmux, whatever the overlay appears to say.

Wiring
------
One jumper, PWM pin to input pin. Default is BOARD 15 (bucket) -> BOARD 7, which
is unused by this project. Disconnect the actuator first so nothing is driven
while probing.

Usage::

    python3 tools/actuators/pwm_probe.py                   # pin 15 -> pin 7
    python3 tools/actuators/pwm_probe.py --pwm 33 --in 29  # boom -> pin 29
    python3 tools/actuators/pwm_probe.py --frame-width 0.0021
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "excavator_control"))

from excavator_control.jetson_servo import JetsonServo  # noqa: E402
import Jetson.GPIO as GPIO  # noqa: E402  (jetson_servo sets JETSON_MODEL_NAME first)


def sample(pin, seconds):
    """Sample a GPIO input as fast as Python allows.

    Returns ``(fraction_high, rising_edges, elapsed, count)``. Sampling is
    unsynchronised, so the high fraction is a Monte Carlo estimate of duty
    cycle -- accurate to well under a percent given thousands of samples per
    period, which is what ~200 kHz sampling against a 50 Hz carrier gives.
    """
    read = GPIO.input
    deadline = time.perf_counter() + seconds
    samples = []
    t0 = time.perf_counter()
    while time.perf_counter() < deadline:
        samples.append(read(pin))
    elapsed = time.perf_counter() - t0

    if not samples:
        return 0.0, 0, elapsed, 0
    high = sum(samples)
    edges = sum(1 for a, b in zip(samples, samples[1:]) if not a and b)
    return high / len(samples), edges, elapsed, len(samples)


def probe(pwm_pin, in_pin, frame_width, duration):
    expected_hz = 1.0 / frame_width
    print(f"PWM BOARD {pwm_pin}  ->  input BOARD {in_pin}")
    print(f"frame {frame_width * 1000:.2f} ms ({expected_hz:.1f} Hz), "
          f"{duration:.2f}s per measurement\n")

    GPIO.setwarnings(False)
    if GPIO.getmode() is None:
        GPIO.setmode(GPIO.BOARD)
    GPIO.setup(in_pin, GPIO.IN)

    print("  baseline with PWM idle (expect a steady level, no edges):")
    frac, edges, elapsed, n = sample(in_pin, 0.1)
    print(f"    {frac * 100:6.2f}% high, {edges} rising edges, {n} samples\n")

    servo = JetsonServo(pwm_pin, frame_width=frame_width)
    results = []
    try:
        print("    cmd    pulse    expected    measured    edges    meas.Hz")
        print("    " + "-" * 56)
        for cmd in (-90, -45, 0, 45, 90):
            servo.angle = cmd
            time.sleep(0.05)
            frac, edges, elapsed, n = sample(in_pin, duration)
            pulse_ms = 1.0 + (cmd + 90) / 180.0
            expected = pulse_ms / (frame_width * 1000) * 100
            measured_hz = edges / elapsed if elapsed else 0.0
            results.append((expected, frac * 100, measured_hz))
            print(f"    {cmd:+4}  {pulse_ms:5.3f}ms   {expected:6.2f}%     "
                  f"{frac * 100:6.2f}%   {edges:6}   {measured_hz:7.1f}")
    finally:
        servo.close()
        GPIO.cleanup(in_pin)

    print()
    worst = max(abs(e - m) for e, m, _ in results)
    freqs = [f for _, _, f in results if f > 0]
    toggling = any(f > expected_hz * 0.5 for _, _, f in results)

    if not toggling:
        print("  VERDICT: the pad is NOT toggling.")
        print("  The PWM controller is configured but its output is not reaching")
        print("  the header pin. This is a Jetson-side fault -- almost certainly")
        print("  the pinmux, regardless of what the overlay appears to say.")
        print("  (If you forgot the jumper, that also looks like this. Check it.)")
        return 1

    mean_hz = sum(freqs) / len(freqs)
    print(f"  VERDICT: the pad IS toggling. Carrier {mean_hz:.1f} Hz "
          f"(expected {expected_hz:.1f}),")
    print(f"  duty tracks the command to within {worst:.2f} percentage points.")
    print()
    print("  So the Jetson side is working and the fault is downstream:")
    print("    1. Signal level -- the L16-R wants a 5 V pulse, this pin drives")
    print("       3.3 V. Most likely cause. Needs a level shifter.")
    print("    2. Actuator power -- 6 V present at the actuator's red lead?")
    print("    3. Common ground -- actuator black MUST tie to a Jetson ground.")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--pwm", type=int, default=15,
                   help="BOARD pin driving PWM (default 15, the bucket)")
    p.add_argument("--in", dest="in_pin", type=int, default=7,
                   help="BOARD pin to read the loopback on (default 7)")
    p.add_argument("--frame-width", type=float, default=0.02,
                   help="frame period in seconds (default 0.02 = 50 Hz)")
    p.add_argument("--duration", type=float, default=0.2,
                   help="seconds to sample per command (default 0.2)")
    args = p.parse_args(argv)

    if args.pwm == args.in_pin:
        p.error("--pwm and --in must be different pins")
    try:
        return probe(args.pwm, args.in_pin, args.frame_width, args.duration)
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130


if __name__ == "__main__":
    sys.exit(main())
