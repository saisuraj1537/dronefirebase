# import required modules
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import math
import socket
import argparse
import geopy.distance
import serial
import numpy as np
import RPi.GPIO as GPIO
from gpiozero import Servo
from time import sleep



# Set the GPIO pin number connected to the servo
servo_pin = 18

# Create a servo object
servo = Servo(servo_pin)

import time
import firebase_admin
from firebase_admin import db,credentials


# authenticate to firebase
cred = credentials.Certificate("credentials.json")
firebase_admin.initialize_app(cred, {"databaseURL": "https://sample-a028e-default-rtdb.firebaseio.com/"})

ref = db.reference("/")
# retrieving data from root node
ref.get()


def connectMyCopter():
    parser =  argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 57600
    print("\nConnecting to vehicle on: %s" % connection_string)
    vehicle = connect(connection_string ,baud=baud_rate ,wait_ready=True)
    return  vehicle

def get_dstance(cord1, cord2):
    # return distance n meter
    return (geopy.distance.geodesic(cord1, cord2).km ) *1000

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)


    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(2)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    time.sleep(3)



    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude *0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


def servo_trigg():
    servo.max()
    # servo.close()



def goto_location(to_lat, to_long):

    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    curr_lat = vehicle.location.global_relative_frame.lat
    curr_lon = vehicle.location.global_relative_frame.lon
    curr_alt = vehicle.location.global_relative_frame.alt

    # set to locaton (lat, lon, alt)
    to_lat = to_lat
    to_lon = to_long
    to_alt = curr_alt

    to_pont = LocationGlobalRelative(to_lat ,to_lon ,to_alt)
    vehicle.simple_goto(to_pont, groundspeed=8)

    to_cord = (to_lat, to_lon)
    while True:
        curr_lat = vehicle.location.global_relative_frame.lat
        curr_lon = vehicle.location.global_relative_frame.lon
        curr_cord = (curr_lat, curr_lon)
        print("curr location: {}".format(curr_cord))
        distance = get_dstance(curr_cord, to_cord)
        print("distance ramaining {}".format(distance))
        if distance <= 2:
            print("Reached within 2 meters of target location...")
            break
        time.sleep(1)

data = db.reference("/").get()

i=0

while i == 0:
    for key, value in data.items():
        if (data[key]['dronestatus']) == "none":
            username = (data[key]['userDetails']['username'])
            shop_lat = (data[key]['shopDetails']['shopLat'])
            shop_long = (data[key]['shopDetails']['shopLong'])
            user_lat = (data[key]['userDetails']['userLat'])
            user_long = (data[key]['userDetails']['userLong'])
            # db.reference("/" + username + "/" + 'dronestatus').set("none")
            break


#goes to the shopkeeper
vehicle = connectMyCopter()
time.sleep(1)
ht = 10
arm_and_takeoff(ht)
time.sleep(1)
goto_location(shop_lat ,shop_long)
time.sleep(2)
vehicle.mode = VehicleMode("LAND")
time.sleep(300)


#goes to the customer
# vehicle = connectMyCopter()
time.sleep(1)
ht = 10
arm_and_takeoff(ht)
time.sleep(1)
goto_location(user_lat ,user_long)
time.sleep(2)
vehicle.mode = VehicleMode("LAND")
time.sleep(150)
servo_trigg()
time.sleep(60)


#comes to the home location
# vehicle = connectMyCopter()
time.sleep(1)
ht = 10
arm_and_takeoff(ht)
time.sleep(1)
vehicle.mode = VehicleMode("RTL")

db.reference("/" + username + "/" + 'dronestatus').set("done")