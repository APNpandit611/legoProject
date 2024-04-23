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

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# MQTT setup
MQTT_ClientID_B = 'RobotB'
MQTT_Broker = '172.20.10.7'
MQTT_Topic_Status = 'Lego/Status'
client_B = MQTTClient(MQTT_ClientID_B, MQTT_Broker, 1883)

# Create objects for Robot B
ev3_B = EV3Brick()
drive_base_B = DriveBase(left_motor=Motor(Port.B), right_motor=Motor(Port.C), wheel_diameter=55.5, axle_track=104)

# Callback for listening to topics on Robot B
def listen_B(topic, msg):
    if topic == MQTT_Topic_Status.encode() and msg.decode() == "StopMovement":
        ev3_B.screen.print("Received StopMovement message")
        drive_base_B.drive(100, 0)
        wait(2000)
        drive_base_B.stop()
# Connect and subscribe to topics on Robot B
client_B.connect()
client_B.set_callback(listen_B)
client_B.subscribe(MQTT_Topic_Status)

# Start the relay race on Robot B
ev3_B.speaker.beep()
ev3_B.screen.print('Started')

while True:
    client_B.check_msg()
    wait(1000)
    
client_B.disconnect()