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
    pressed_buttons = ev3.buttons.pressed()

    if pressed_buttons:
        button_names = [str(button) for button in pressed_buttons]
        pressed_buttons_text = ', '.join(button_names)
        ev3.screen.clear()
        ev3.screen.print("Pressed Buttons: {}".format(pressed_buttons_text))

    if len(pressed_buttons) > 1:
        ev3.screen.print("Multiple Buttons Pressed")

    ev3.speaker.wait()

    # Clear the screen after a delay
    wait(1000)
    ev3.screen.clear()