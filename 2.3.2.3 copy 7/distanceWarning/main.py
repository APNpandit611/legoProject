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
import time


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
ultrasonic_sensor = UltrasonicSensor(Port.S4)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print("Connection failed with code:", rc)

client = mqtt.Client()
client.on_connect = on_connect

# Replace with your MQTT broker's address and port
client.connect("your_broker_address", 1883)
client.loop_start()


left_motor = motor.OUTPUT_A
right_motor = motor.OUTPUT_D
light_sensor = sensor.EV3_COLOR_SENSOR

# Calibrate or configure the light sensor if necessary
light_sensor.mode = 'RGB_RAW'  # Example mode for raw RGB values (adjust based on sensor type and desired data)

def track_light(threshold):
    while True:
        # Read light sensor values (adjust based on sensor mode)
        red_value, green_value, blue_value = light_sensor.value(0), light_sensor.value(1), light_sensor.value(2)

        # Calculate average light intensity
        average_intensity = (red_value + green_value + blue_value) / 3

        # Determine direction based on light intensity difference
        if average_intensity > threshold + 10:  # Adjust threshold and difference values as needed
            left_motor.run_forever(speed_sp=100)  # Adjust speed as needed
            right_motor.run_forever(speed_sp=-100)  # Reverse direction for turning left
            time.sleep(0.1)  # Adjust turning duration
        elif average_intensity < threshold - 10:
            left_motor.run_forever(speed_sp=-100)
            right_motor.run_forever(speed_sp=100)  # Reverse direction for turning right
            time.sleep(0.1)
        else:
            # Go straight if light intensity is within the threshold range
            left_motor.run_forever(speed_sp=100)
            right_motor.run_forever(speed_sp=100)

        # Publish light sensor data to MQTT topic (optional)
        client.publish("light_sensor_data", f"{red_value},{green_value},{blue_value}")  # Example topic and data format
        time.sleep(0.1)  # Adjust data publishing frequency

# Set a light intensity threshold for tracking
light_threshold = 50  # Adjust based on your sensor setup and lighting conditions

track_light(light_threshold)
