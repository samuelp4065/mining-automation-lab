from gpiozero import AngularServo
from time import sleep
import time
import curses
import lss
import lss_const as lssc

PORT = "/dev/ttyUSB0"
BAUD = lssc.LSS_DefaultBaud
SWINGDRIVE_ID = 2
MAX_SPEED = 70  # more realistic for motion


lss.initBus(PORT, BAUD)
swingdrive = lss.LSS(SWINGDRIVE_ID)

swingdrive.setMaxSpeed(MAX_SPEED)
pos = swingdrive.getPosition()

boom = AngularServo(17, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=None)
arm = AngularServo(27, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=None)
bucket = AngularServo(22, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=None)

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
    print("Returned to original position.")

def bucket_dump():
    bucket.angle = -85
    sleep(1)
    print("Bucket dumped.")


def bucket_scoop():
    boom.angle = -75
    sleep(5)
    arm.angle = 60
    sleep(5)
    bucket.angle = 45
    sleep(6)
    boom.angle = 80
    arm.angle = -80
    print("Bucket scooped.")

def turn_left(): #higher the number the more to the left
    new_pos = 1500
    print(f"Turning swingdrive to position {new_pos}")
    swingdrive.move(new_pos)
    if new_pos == pos:
        print(f"Swingdrive turned position: {pos}" )
    # sleep(3)

def turn_right(): #lower the number the more to the right
    new_pos = -4000
    print(f"Turning swingdrive to position {new_pos}")
    swingdrive.move(new_pos)
    print(f"Swingdrive turned position: {pos}" )
    if new_pos == pos:
        print(f"Swingdrive turned position: {pos}" )
    # sleep(3)

def getCurrentAngles():
    pos = swingdrive.getPosition()
    print(f"Current boom angle: {boom.angle}")
    print(f"Current arm angle: {arm.angle}")
    print(f"Current bucket angle: {bucket.angle}")
    print(f"Current swingdrive position: {pos}")

def Body_Move():
    boom.angle = -28.29
    arm.angle = -75.91
    bucket.angle = -63.75
    sleep(5)

    print("Boom move")


try:
    while True:
        command = input("Enter command: ").strip().lower()
        if command == "o":
            original_position()
        elif command == "dump":
            bucket_dump()
        elif command == "scoop":
            bucket_scoop()
        elif command == "stop":
            stop_all()
        elif command == "1":
            turn_left()
        elif command == "2":
            turn_right()
        elif command == "g":
            getCurrentAngles()
        elif command == "m":
            Body_Move()
        elif command == "exit":
            stop_all()
            break
        else:
            print("Unknown command.")

except KeyboardInterrupt:
    print("\nKeyboard interrupt received.")
finally:
    stop_all()