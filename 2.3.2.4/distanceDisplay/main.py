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
ultrasonic_sensor = UltrasonicSensor(Port.S4)

# Write your program here.
ev3.speaker.beep()

robot = DriveBase(Motor(Port.B), Motor(Port.C), wheel_diameter=55.5, axle_track=104)

while True:
    distance = ultrasonic_sensor.distance()

    ev3.screen.clear()
    ev3.screen.print("Distance: {} cm".format(distance))
    robot.drive(100, 0) 
    wait(200)

    if touch_sensor.pressed():
        break

robot.stop()
ev3.screen.clear()
ev3.speaker.beep()