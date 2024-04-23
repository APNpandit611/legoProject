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
MQTT_ClientID_A = 'RobotA'
MQTT_Broker = '172.20.10.6'  
MQTT_Topic_Status_A = 'Lego/Status'
MQTT_Topic_Command_A = 'Lego/Command'
client_A = MQTTClient(MQTT_ClientID_A, MQTT_Broker, 1883)

# Create objects for Robot A
ev3_A = EV3Brick()
ultrasonic_sensor_A = UltrasonicSensor(Port.S4)  
motor_A = Motor(Port.B)

#  MQTT broker
client_A.connect()


def stop_robot_A():
    motor_A.stop()

# Callback commands
def listen_A(topic, msg):
    if topic == MQTT_Topic_Command_A.encode():
        command = str(msg.decode())
        ev3_A.screen.print("Received command: {}".format(command))

        if command == "STOP":
            stop_robot_A()

client_A.set_callback(listen_A)
client_A.subscribe(MQTT_Topic_Command_A)

# Approach Robot B
while True:
    distance_to_B = ultrasonic_sensor_A.distance()

    if distance_to_B < 10:  
        ev3_A.screen.print("In front of Robot B. Sending message.")
        client_A.publish(MQTT_Topic_Status_A, "StopAndWait")
        break

    # Move forward
    motor_A.run(200)
    time.sleep(0.1)

# Disconnect
client_A.disconnect()