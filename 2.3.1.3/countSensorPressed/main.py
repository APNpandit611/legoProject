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
touch_sensor = TouchSensor(Port.S1)


# Write your program here.
ev3.speaker.beep()


counting_duration = 5
press_count = 0

while True:
    if touch_sensor.pressed():
        press_count += 1
        wait(200)

    if time.time() - start_time >= counting_duration:
        break

ev3.screen.print("Press Count: {}".format(press_count))
for _ in range(press_count):
    ev3.speaker.play_file(SoundFile.CAT_PURR)
    wait(1000)  

    