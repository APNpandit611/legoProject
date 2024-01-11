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


# Create your objects here.
ev3 = EV3Brick()
UltraSensor=UltrasonicSensor(Port.S4)
robot = DriveBase(Motor(Port.B), Motor(Port.C), wheel_diameter=55.5, axle_track=104)

# Write your program here.
ev3.speaker.beep()
stop_distance_cm = 20

while UltraSensor.distance() > stop_distance_cm:
    robot.drive(100, 0) 

robot.stop()
ev3.speaker.beep()