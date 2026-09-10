from gpiozero import AngularServo
from time import sleep
import time
import curses
import lss
import lss_const as lssc

PORT = "/dev/ttyUSB0"
BAUD = lssc.LSS_DefaultBaud
SWINGDRIVE_ID = 2

lss.initBus(PORT, BAUD)
swingdrive = lss.LSS(SWINGDRIVE_ID)

print(f"The position of the swingdrive is: {swingdrive.getPosition()}")
print(f"The status of the swingdrive is: {swingdrive.getStatus()}")