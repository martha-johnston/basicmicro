from roboclaw_3 import Roboclaw
import time

# address of the RoboClaw as set in Motion Studio

address = 128

# Creating the RoboClaw object, serial port and baudrate passed

roboclaw1 = Roboclaw("/dev/ttyACM1", 38400)
roboclaw2 = Roboclaw("/dev/ttyACM1", 38400)

# Starting communication with the RoboClaw hardware

is_open = roboclaw1.Open()
time.sleep(2)
print(is_open)

is_open = roboclaw2.Open()
time.sleep(2)
print(is_open)

# power = 0.3

# Start motor 1 in the forward direction at half speed

roboclaw1.DutyM1(address, 10000)
time.sleep(2)
roboclaw1.DutyM1(address, 0)

time.sleep(1)

roboclaw2.DutyM2(address, 10000)
time.sleep(2)
roboclaw2.DutyM2(address, 0)
