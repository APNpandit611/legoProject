#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from umqtt.robust import MQTTClient
import time

#MQTT setup

MQTT_ClientID='testmqtt'
MQTT_Broker='172.26.32.1'
MQTT_Topic_Status='Lego/Status'
client=MQTTClient(MQTT_ClientID,MQTT_Broker,1883)
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()

# Write your program here.
client.connect()
ev3.speaker.beep()
client.publish(MQTT_Topic_Status, "Hello World!!")
ev3.screen.print("Message send...")
time.sleep(2)