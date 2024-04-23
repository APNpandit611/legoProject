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
MQTT_ClientID = 'RobotB'
MQTT_Broker = '192.168.69.113'
MQTT_Topic_Remote1 = 'Lego/remote/control'
MQTT_Topic_Remote2 = 'Lego/remote/sensor'
client = MQTTClient(MQTT_ClientID, MQTT_Broker)


#callback for listen to topics
def listen(topic,msg):
    
    if topic == MQTT_Topic_Remote1.encode():
        ev3.screen.print(str(msg.decode()))
        if str(msg.decode()) == str(Button.UP):
            robot.drive(160,0)
        elif str(msg.decode()) == str(Button.DOWN):
            robot.drive(-120,0)
        elif str(msg.decode()) == str(Button.LEFT):
            robot.drive(120,50)
        elif str(msg.decode()) == str(Button.RIGHT):
            robot.drive(120,-50)
        elif str(msg.decode()) == str(Button.CENTER):
            ev3.speaker.play_notes(['C4/4', 'E4/4', 'G4/4', 'C5/4'], tempo=120)
        else:
            robot.stop()


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Objects
ev3 = EV3Brick()
# init Motor
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=130)
#init Sensor
UltraSensor = UltrasonicSensor(Port.S2)
TSensor = TouchSensor(Port.S1)
#CSensor = ColorSensor(Port.S3)


# Write your program here.
client.connect()
time.sleep(0.5)
client.set_callback(listen)
client.subscribe(MQTT_Topic_Remote1)
ev3.speaker.beep()
ev3.screen.print('listening')

# wait until message
while True:
    client.check_msg()
    client.publish(MQTT_Topic_Remote2, str(UltraSensor.distance()))
    time.sleep(0.25)