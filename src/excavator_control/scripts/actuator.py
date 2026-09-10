from gpiozero import AngularServo
from time import sleep
import keyboard_track
import time



# ---------------- RC linear actuator on GPIO18 ----------------
# 1.0ms..2.0ms pulse range at ~50 Hz
# boom = AngularServo(17, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.02)
arm = AngularServo(27, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.02, initial_angle=0)
# bucket = AngularServo(22, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.02)

try:
    # boom.angle = 45  # initial position
    # sleep(10)

    arm.angle = 60  # initial position
    sleep(10)
    arm.detach()

    # bucket.angle = -90  # initial position
    # sleep(10)

except KeyboardInterrupt:
    # boom.detach()
    arm.detach()
    # bucket.detach()
    pass
finally:
    # always stop motors
    # boom.detach()
    arm.detach()
    # bucket.detach()


