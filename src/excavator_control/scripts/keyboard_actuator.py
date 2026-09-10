import curses
from gpiozero import AngularServo
import time

# ---------------- RC linear actuator (ARM example) ----------------
# frame_width is the total period of the PWM signal, meaning it affects the angle step size. the smaller it is 
# the finer the control but also the less torque. 0.0021 is a good compromise.
boom = AngularServo(17, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=0)
arm = AngularServo(27, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=0)
bucket = AngularServo(22, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=0)



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
    arm.detach()
    print("Stopped.")

def control(stdscr):
    global curr_angle_boom, curr_angle_arm, curr_angle_bucket

    stdscr.nodelay(True)  # do NOT block on getch
    stdscr.clear()
    stdscr.addstr(0, 0, "Controls:")
    stdscr.addstr(1, 0, "  u = boom up") #addsrt(row, col, string) this is a method to print the string at the exact position in the terminal
    stdscr.addstr(2, 0, "  i = boom down")
    stdscr.addstr(4, 0, "  j = arm up")
    stdscr.addstr(5, 0, "  k = arm down")
    stdscr.addstr(7, 0, "  n = bucket up")
    stdscr.addstr(8, 0, "  m = bucket down")    

    while True:
        key = stdscr.getch()

        if key != -1:
            # quit
            if key == ord('q'):
                break
            # boom up
            if key == ord('u'):
                curr_angle_boom = clamp(curr_angle_boom + ANGLE_STEP_BOOM, MIN_ANGLE, MAX_ANGLE)
                boom.angle = curr_angle_boom
                stdscr.addstr(3, 0, f"Boom angle: {curr_angle_boom}   ")      
            # boom down
            if key == ord('i'):
                curr_angle_boom = clamp(curr_angle_boom - ANGLE_STEP_BOOM, MIN_ANGLE, MAX_ANGLE)
                boom.angle = curr_angle_boom
                stdscr.addstr(3, 0, f"Boom angle: {curr_angle_boom}   ")  
            # arm up
            if key == ord('k'):
                curr_angle_arm = clamp(curr_angle_arm + ANGLE_STEP_ARM, MIN_ANGLE, MAX_ANGLE)
                arm.angle = curr_angle_arm
                stdscr.addstr(6, 0, f"Arm angle: {curr_angle_arm}   ")

            # arm down
            if key == ord('j'):
                curr_angle_arm = clamp(curr_angle_arm - ANGLE_STEP_ARM, MIN_ANGLE, MAX_ANGLE)
                arm.angle = curr_angle_arm
                stdscr.addstr(6, 0, f"Arm angle: {curr_angle_arm}   ")
            
            # bucket up
            if key == ord('n'):
                curr_angle_bucket = clamp(curr_angle_bucket + ANGLE_STEP_BUCKET, MIN_ANGLE, MAX_ANGLE)
                bucket.angle = curr_angle_bucket
                stdscr.addstr(9, 0, f"Bucket angle: {curr_angle_bucket}   ")
            
            # bucket down
            if key == ord('m'):
                curr_angle_bucket = clamp(curr_angle_bucket - ANGLE_STEP_BUCKET, MIN_ANGLE, MAX_ANGLE)
                bucket.angle = curr_angle_bucket
                stdscr.addstr(9, 0, f"Bucket angle: {curr_angle_bucket}   ")

        time.sleep(0.001)  # reduce CPU load

try:
    curses.wrapper(control)
finally:
    stop_all()
