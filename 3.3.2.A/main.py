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


# Create objects for Robot A
ev3 = EV3Brick()
ultrasonic_sensor_A = UltrasonicSensor(Port.S4)
drive_base_A = DriveBase(left_motor=Motor(Port.B), right_motor=Motor(Port.C), wheel_diameter=55.5, axle_track=104)

# Connect and subscribe to topics
client.connect()
client.set_callback(listen)
client.subscribe(MQTT_Topic_Status)

# Start the relay race
ev3.speaker.beep()
ev3.screen.print('Started')

while True:
    client.check_msg()

    # Approach Robot B using the ultrasonic sensor
    distance_to_B = ultrasonic_sensor_A.distance()
    if distance_to_B < 50:  # Adjust the threshold as needed
        ev3.screen.print("Close to Robot B. Sending message.")
        client.publish(MQTT_Topic_Status, "StopMovement")
        break

    # Drive forward
    drive_base_A.drive(100, 0)

    wait(100)  # Adjust the delay as needed

# Disconnect from the MQTT broker
client.disconnect()