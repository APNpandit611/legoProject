#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.

ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
touch_sensor = TouchSensor(Port.S1)

# Write your program here.
ev3.speaker.beep()


initial_speed = 100 
acceleration = 100  
time_interval = 1  

# Drive forward with gradual acceleration

while True:
    if touch_sensor.pressed():
     for _ in range(5): 
        robot.drive_time(initial_speed, 0, 1000) 
        initial_speed += acceleration 
        time.sleep(time_interval) 

# Beep to signal completion
ev3.speaker.beep()
