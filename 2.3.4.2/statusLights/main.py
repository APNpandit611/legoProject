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


# Write your program here.
ev3.speaker.beep()

while True:
    if Button.CENTER in ev3.buttons.pressed():
        ev3.light.on(Color.RED)
    else:
        ev3.light.on(Color.GREEN)

    if len(ev3.buttons.pressed()) > 1:
        ev3.screen.print("Multiple Buttons Pressed")

    ev3.speaker.wait()

    # Clear the screen after a delay
    wait(1000)
    ev3.screen.clear()
    