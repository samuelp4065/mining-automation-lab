from gpiozero import AngularServo
from time import sleep

boom = AngularServo(17, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=None)
arm = AngularServo(27, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=None)
bucket = AngularServo(22, min_pulse_width=0.001, max_pulse_width=0.002, frame_width=0.0021, initial_angle=None)


def stop_all():
    boom.detach()
    arm.detach()
    bucket.detach()
    print("All actuators stopped.")

def bucket_dump():
    global bucket
    print("Dumping bucket...")
    boom.angle = 90
    arm.angle = -45
    bucket.angle = -50
    sleep(1)
    print("Bucket dumped.")


def bucket_scoop():
    global bucket
    print("Scooping bucket...")
    boom.angle = -65
    sleep(3)
    arm.angle = 60
    sleep(2)
    bucket.angle = 35
    print("Bucket scooped.")

try:
    while True:
        command = input("Enter command (dump, scoop, stop, exit): ").strip().lower()
        if command == "dump":
            bucket_dump()
        elif command == "scoop":
            bucket_scoop()
        elif command == "stop":
            stop_all()
        elif command == "exit":
            stop_all()
            break
        else:
            print("Unknown command.")

except KeyboardInterrupt:
    print("\nKeyboard interrupt received.")
finally:
    stop_all()