import time
import curses

#Import LSS library for Servo Motor Control
import lss
import lss_const as lssc

#Import Actuator Library
from gpiozero import AngularServo

PORT = "/dev/ttyUSB0"
BAUD = lssc.LSS_DefaultBaud
SWINGDRIVE_ID = 2
RPM_STEP = 5
BASE_RPM = 10  # more realistic for motion

# Init LSS
lss.initBus(PORT, BAUD)
swingdrive = lss.LSS(SWINGDRIVE_ID)
swingdrive.reset()  # reset position to zero

# Initialize Actuators

boom = AngularServo(17, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=None)
arm = AngularServo(27, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=None)
bucket = AngularServo(22, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=None)

ANGLE_STEP_BOOM = 5
ANGLE_STEP_ARM = 5
ANGLE_STEP_BUCKET = 5

MIN_ANGLE = -90
MAX_ANGLE = 90

curr_angle_boom = 0
curr_angle_arm = 0
curr_angle_bucket = 0

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def stop_all():
    boom.detach()
    arm.detach()
    bucket.detach()
    swingdrive.wheelRPM(0)
    # print("Stopped.")

def run(stdscr):
    global curr_angle_boom, curr_angle_arm, curr_angle_bucket

    curses.curs_set(0)
    stdscr.clear()
    stdscr.nodelay(True)       # don't block on getch
    stdscr.keypad(True)
    stdscr.addstr(0, 0, "Current swingdrive position : ")
    stdscr.addstr(1, 0, "  4 = turn left")
    stdscr.addstr(2, 0, "  6 = turn right")
    stdscr.addstr(3, 0, "  u = boom up") #addsrt(row, col, string) this is a method to print the string at the exact position in the terminal
    stdscr.addstr(4, 0, "  i = boom down")
    stdscr.addstr(6, 0, "  j = arm up")
    stdscr.addstr(7, 0, "  k = arm down")
    stdscr.addstr(9, 0, "  n = bucket up")
    stdscr.addstr(10, 0, "  m = bucket down")      
  
    while True:
        key = stdscr.getch()
        pos = swingdrive.getPosition()
        if key != -1:
            # quit
            if key == ord('q'):
                break
            # boom up
            if key == ord('u'):
                curr_angle_boom = clamp(curr_angle_boom + ANGLE_STEP_BOOM, MIN_ANGLE, MAX_ANGLE)
                boom.angle = curr_angle_boom
                stdscr.addstr(5, 0, f"Boom angle: {curr_angle_boom}   ")      
            # boom down
            if key == ord('i'):
                curr_angle_boom = clamp(curr_angle_boom - ANGLE_STEP_BOOM, MIN_ANGLE, MAX_ANGLE)
                boom.angle = curr_angle_boom
                stdscr.addstr(5, 0, f"Boom angle: {curr_angle_boom}   ")  
            # arm up
            if key == ord('j'):
                curr_angle_arm = clamp(curr_angle_arm + ANGLE_STEP_ARM, MIN_ANGLE, MAX_ANGLE)
                arm.angle = curr_angle_arm
                stdscr.addstr(8, 0, f"Arm angle: {curr_angle_arm}   ")
            # arm down
            if key == ord('k'):
                curr_angle_arm = clamp(curr_angle_arm - ANGLE_STEP_ARM, MIN_ANGLE, MAX_ANGLE)
                arm.angle = curr_angle_arm
                stdscr.addstr(8, 0, f"Arm angle: {curr_angle_arm}   ")  
            # bucket up
            if key == ord('n'):
                curr_angle_bucket = clamp(curr_angle_bucket + ANGLE_STEP_BUCKET, MIN_ANGLE, MAX_ANGLE)
                bucket.angle = curr_angle_bucket
                stdscr.addstr(11, 0, f"Bucket angle: {curr_angle_bucket}   ")
            # bucket down
            if key == ord('m'):
                curr_angle_bucket = clamp(curr_angle_bucket - ANGLE_STEP_BUCKET, MIN_ANGLE, MAX_ANGLE)
                bucket.angle = curr_angle_bucket
                stdscr.addstr(11, 0, f"Bucket angle: {curr_angle_bucket}   ")

            if key == ord('4'):
                swingdrive.wheelRPM(BASE_RPM)

            if key == ord('6'):
                swingdrive.wheelRPM(-BASE_RPM)
                

            if key == ord('0'):
                stop_all()
        stdscr.addstr(0, 30, f"{pos} ")

if __name__ == "__main__":
    curses.wrapper(run)
