from roboclaw import RoboClaw
import time

roboclaw1 = RoboClaw(port='/dev/ttyACM0', address=0x80)
# Read the roboclaw manual to understand the trajectory parameters
roboclaw1.drive_to_position_raw(motor=1, accel=0, speed=0, deccel=0, position=25, buffer=1)
while True:
    print(roboclaw1.read_position(1))
    time.sleep(0.5)
