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
MQTT_ClientID = 'RobotA'
MQTT_Broker = '192.168.69.113'
MQTT_Topic_Remote1 = 'Lego/remote/control'
MQTT_Topic_Remote2 = 'Lego/remote/sensor'
client = MQTTClient(MQTT_ClientID, MQTT_Broker)


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

#callback for listen to topics
def listen(topic,msg):
    if topic == MQTT_Topic_Remote2.encode():
        ev3.screen.print(str(msg.decode()))

# Create your objects here.
ev3 = EV3Brick()
# init Motor
left_motor = Motor(Port.B)
right_motor = Motor(Port.A)
# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=130)
#init Sensor
UltraSensor = UltrasonicSensor(Port.S4)
TSensor = TouchSensor(Port.S1)
CSensor = ColorSensor(Port.S3)

# Write your program here.
client.connect()
time.sleep(0.5)
client.set_callback(listen)
client.subscribe(MQTT_Topic_Remote2)
ev3.speaker.beep()
ev3.screen.print('listening')

lastButtonPressed = buttonPressed = 'none'
while True:
    client.check_msg()
    try:
        buttonPressed = ev3.buttons.pressed()[0]
    except:
        buttonPressed = 'none'

    if buttonPressed != lastButtonPressed:
        client.publish(MQTT_Topic_Remote1, str(buttonPressed))
        lastButtonPressed = buttonPressed
    time.sleep(0.1)