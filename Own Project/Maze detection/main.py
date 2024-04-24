#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import ev3dev2.motor as motor
import ev3dev2.sensor as sensor
import paho.mqtt.client as mqtt
#from umqtt.robust import MQTTClient
import time


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
# Basic setup for all the ports for the EV3
ev3 = EV3Brick()

#sensors
ultrasonic_sensor = UltrasonicSensor(Port.S1)
color_sensor = ColorSensor(Port.S2 )
touch_sensor = TouchSensor(Port.S3)

"""
MQTT_ClientID = 'RobotA'
MQTT_Broker = '172.20.10.3'
MQTT_Topic_Hallo = 'python/course'
MQTT_Topic_Sensor = "python/message"
client = MQTTClient(MQTT_ClientID, MQTT_Broker)
client.connect()
client.publish(MQTT_Topic_Sensor, "initial_message")

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print("Connection failed with code:", rc)

client = mqtt.Client()
client.on_connect = on_connect


client.connect("your_broker_address", 1883)
client.loop_start()
"""
#ports
left_motor = Motor(Port.B)
right_motor = Motor(Port.C) 
robot = DriveBase(left_motor, right_motor, wheel_diameter = 54, axle_track = 105)
base_speed = 50
slow_speed = 25
setpoint = 120
max_error = 10
obstacle_threshold = 100
turn_angle = 90


#actual code starts from here
ev3.speaker.beep()

#colors
start_color = Color.RED
finish_color = Color.BLACK

#function to follow wall
def follow_wall():
    while True:
        wall_distance = ultrasonic_sensor.distance()
        error = wall_distance - setpoint
        if error < -max_error:  #too close to the wall
            #steer away
            left_motor.dc(base_speed)
            right_motor.dc(base_speed - slow_speed)
        elif error > max_error:  #too far away from the wall
            #steer in
            left_motor.dc(base_speed - slow_speed)
            right_motor.dc(base_speed)
        else:
            #continue following the wall
            left_motor.dc(base_speed)
            right_motor.dc(base_speed)

#function to detect obstacle
def detect_obstacle():
    while True:
        #check if obstacle is detected
        if ultrasonic_sensor.distance() < obstacle_threshold: #obstacle detected!
            ev3.screen.clear()
            ev3.screen.print("OH NOO!!")
            #reversing a little
            robot.drive_time(-base_speed, 0, 500) 
            #turn away from the wall
            robot.turn(Direction.RIGHT, turn_angle)
            #robot.drive_time(0, 45, 1000)
            while ultrasonic_sensor.distance() < obstacle_threshold:
                robot.drive(base_speed // 2, 0) 
            #continue following the wall
            follow_wall()

#function to detect exit with color sensor
def detect_exit():
    while True:
        if color_sensor.color() == finish_color:
            #exit detected!!!
            break

    
follow_wall()
detect_obstacle()
detect_exit()
