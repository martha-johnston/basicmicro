from roboclaw_3 import Roboclaw
import time

# address of the RoboClaw as set in Motion Studio

address = 128

# Creating the RoboClaw object, serial port and baudrate passed
port= "/dev/ttyACM0"
roboclaw1 = Roboclaw(port, 38400)
roboclaw2 = Roboclaw(port, 38400)

# Starting communication with the RoboClaw hardware

is_open = roboclaw1.Open()
time.sleep(2)
print(is_open)

is_open = roboclaw2.Open()
time.sleep(2)
print(is_open)

# power = 0.3

# Start motor 1 in the forward direction at half speed

roboclaw1.SetM1EncoderMode(address,0)
roboclaw1.SetEncM1(address,1000)

roboclaw1.DutyM1(address, 10000)
time.sleep(2)
roboclaw1.DutyM1(address, 0)

# time.sleep(1)

# roboclaw2.DutyM2(address, 10000)
# time.sleep(2)
# roboclaw2.DutyM2(address, 0)
ticks_per_rotation = 490
rpm = 100.
revolutions = 10.
# If encoders are present, use Roboclaw's SpeedDistance function
motor_channel = 1
desticks = revolutions * ticks_per_rotation
ticks_per_second = rpm * ticks_per_rotation / 60.0
print( int(desticks))
print( int(ticks_per_second))

ticks = 0
status = 0
other = 0
_,encMode,encModeOther = roboclaw1.ReadEncoderModes(address)
print("encMode: ",encMode)
if motor_channel == 1:
    status,ticks,other = roboclaw1.ReadEncM1(address)
elif motor_channel == 2:
    status,ticks,other = roboclaw1.ReadEncM2(address)
print( (status))
print( int(ticks))
print( (other))

goforstatus = True
if motor_channel == 1:
    goforstatus= roboclaw1.SpeedDistanceM1(address, int(ticks_per_second), int(desticks), 1)
elif motor_channel == 2:
    goforstatus =roboclaw1.SpeedDistanceM2(address, int(ticks_per_second), int(desticks), 1)
print("goforstatus: ", goforstatus)
time.sleep(2)


ok, cfg = roboclaw1.GetConfig(address)
print("cfg: ", cfg)