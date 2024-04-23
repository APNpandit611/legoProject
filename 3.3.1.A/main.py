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

#MQTTsetup
MQTT_ClientID='RobotA'
MQTT_Broker='172.20.10.7'
MQTT_Topic_Status='Lego/Status'
client=MQTTClient(MQTT_ClientID,MQTT_Broker,1883)

#callback for listen to topics
def listen(topic,msg):
    if topic==MQTT_Topic_Status.encode():
        ev3.screen.print(str(msg.decode()))

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.


client.connect()
ev3.speaker.beep()

client.set_callback(listen)
client.subscribe(MQTT_Topic_Status)
ev3.screen.print('Started')


while True:
    client.check_msg()
    time.sleep(0.5)