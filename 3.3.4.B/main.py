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

# MQTT setup for Robot B
MQTT_ClientID_B = 'RobotB'
MQTT_Broker = '172.20.10.6'  
MQTT_Topic_Status_B = 'Lego/Status'
MQTT_Topic_Command_B = 'Lego/Command'
client_B = MQTTClient(MQTT_ClientID_B, MQTT_Broker, 1883)

# Create objects for Robot
ev3_B = EV3Brick()
motor_B = Motor(Port.B)

# Connect to MQTT broker
client_B.connect()

# Function to move the robot B out of the way
def move_away_B():
    motor_B.run(-200)
    time.sleep(2)
    motor_B.stop()

# Callback for listening to status messages
def listen_B(topic, msg):
    if topic == MQTT_Topic_Status_B.encode():
        status = str(msg.decode())
        ev3_B.screen.print("Received status: {}".format(status))

        if status == "StopAndWait":
            move_away_B()
            client_B.publish(MQTT_Topic_Command_B, "Continue")

# Subscribe 
client_B.set_callback(listen_B)
client_B.subscribe(MQTT_Topic_Status_B)

# Disconnect
client_B.disconnect()
