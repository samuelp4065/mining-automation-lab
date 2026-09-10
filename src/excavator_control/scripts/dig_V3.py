from gpiozero import AngularServo
from time import sleep
import time
import curses
import lss
import lss_const as lssc
import tinyik
import numpy as np


PORT = "/dev/ttyUSB0"
BAUD = lssc.LSS_DefaultBaud
SWINGDRIVE_ID = 2
MAX_SPEED = 70  # more realistic for motion


lss.initBus(PORT, BAUD)
swingdrive = lss.LSS(SWINGDRIVE_ID)

swingdrive.setMaxSpeed(MAX_SPEED)

# curr_pos = int(swingdrive.getPosition())

boom = AngularServo(17, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=None)
arm = AngularServo(27, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=None)
bucket = AngularServo(22, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=None)

# actuator = tinyik.Actuator(['z', [1., 0., 0.],
#                        'z', [1., 0., 0.]])

# arm.ee = [0.5, 1.5, 0.0]

MAX_BUCKET_ANGLE = 45

def stop_all():
    boom.detach()
    arm.detach()
    bucket.detach()
    print("All actuators stopped.")

def original_position():
    boom.angle = 90
    arm.angle = -90
    bucket.angle = -90
    sleep(2)
    print("Returned to original position.")

def scoop():
    boom.angle = -90
    sleep(4)
    arm.angle = 35
    sleep(3)
    bucket.angle = 50
    sleep(8)
    print("Boom moved.")

def up():
    boom.angle = 90
    arm.angle = -40
    sleep(8)
    print("Arm moved up.")


def rotate_left(): #higher the number the more to the left
    curr_pos = int(swingdrive.getPosition())
    new_pos = 3500
    print(f"Turning swingdrive to position {new_pos}")
    swingdrive.move(new_pos + curr_pos)
    sleep(4)

def rotate_right(): #higher the number the more to the left
    curr_pos = int(swingdrive.getPosition())
    new_pos = -3500
    print(f"Turning swingdrive to position {new_pos}")
    swingdrive.move(new_pos + curr_pos)
    sleep(4)

def dump():
    boom.angle = -10
    sleep(3)
    bucket.angle = -90
    sleep(9)
    boom.angle = 90
    sleep(3)
    print("Returned to original position.")

def getCurrentAngles():
    pos = swingdrive.getPosition()
    print(f"Current boom angle: {boom.angle}")
    print(f"Current arm angle: {arm.angle}")
    print(f"Current bucket angle: {bucket.angle}")
    print(f"Current swingdrive position: {pos}")

try:
    while True:
            # original_position()
            scoop()
            up()
            rotate_right()
            dump()
            rotate_left()
            original_position()
            # getCurrentAngles()
            # stop_all()
            break
except KeyboardInterrupt:
    print("\nKeyboard interrupt received.")
finally:
    stop_all()