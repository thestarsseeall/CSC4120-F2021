#!/usr/bin/env python

"""
GoPiGo3 for the Raspberry Pi: an open source robotics platform for the Raspberry Pi.
Copyright (C) 2017  Dexter Industries

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/gpl-3.0.txt>.
"""

from __future__ import print_function
from __future__ import division

import queue
import signal
import threading
from math import *
from statistics import mean
from time import sleep

import picamera

import numpy as np
from curtsies import Input
from di_sensors import inertial_measurement_unit
from easygopigo3 import *
import time

from ipywidgets import widgets, Layout
import easygopigo3 as easy



# module for interfacing with the GoPiGo3 from
# a terminal with a keyboard
from keyboard2 import GoPiGo3WithKeyboard
# module for capturing input events from the keyboard
from curtsies import Input
import signal

from time import sleep

MINIMUM_VOLTAGE = 7.0
DEBUG = False
MOTORS_SPEED = 250 # see documentation
#TODO###############
import csv
#OPTIONAL###########
#for Georgia  -5° 29'
MAGNETIC_DECLINATION =  -5
#TODO###############


#yyyyyyyyTODOCODE##################
import picamera
from PIL import Image
#yyyyyyyyTODOCODE##################

filmvar = 0
#if not filming, set to 0. If filming, set to 1.
filmNum = 1
#tracks how many films have been started and finished in this session.
j = 1
#number of photos done by squarepath when encountering obstacles.
manPhoto = 1
#number of photos manually taken usen the command function

def getNorthPoint(imu):
    """
    Determines the heading of the north point.
    This function doesn't take into account the declination.

    :param imu: It's an InertialMeasurementUnit object.
    :return: The heading of the north point measured in degrees. The north point is found at 0 degrees.

    """

    x, y, z = imu.read_magnetometer()

    # using the x and z axis because the sensor is mounted vertically
    # the sensor's top face is oriented towards the back of the robot
    heading = -atan2(x, -z) * 180 / pi

    # adjust it to 360 degrees range
    if heading < 0:
        heading += 360
    elif heading > 360:
        heading -= 360

    # when the heading is towards the west the heading is negative
    # when the heading is towards the east the heading is positive
    if 180 < heading <= 360:
        heading -= 360

    heading += MAGNETIC_DECLINATION

    return heading


def statisticalNoiseReduction(values, std_factor_threshold = 2):
    """
    Eliminates outlier values that go beyond a certain threshold.

    :param values: The list of elements that are being filtered.
    :param std_factor_threshold: Filtering aggressiveness. The bigger the value, the more it filters.
    :return: The filtered list.

    """

    if len(values) == 0:
        return []

    valarray = np.array(values)
    mean = valarray.mean()
    standard_deviation = valarray.std()
    # just return if we only got constant values
    if standard_deviation == 0:
        return values

    # remove outlier values
    valarray = valarray[(valarray > mean - std_factor_threshold * standard_deviation)
                        & (valarray < mean + std_factor_threshold * standard_deviation)]

    return list(valarray)

#TODOCODE################################################################333
def squarepath(trigger):
    gopigo3_robot = EasyGoPiGo3()
    my_distance_sensor = gopigo3_robot.init_distance_sensor()
    #####j = 0
    
    #set NORTH before staring
    right = 1
    i = 0

    with open('problem1_pathtrace.csv', 'w') as csvfile:
        fieldnames = ['row_num', 'encoder_value','distance_value']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

    def drive_and_turn(i,right,dist=999):
        gopigo3_robot.reset_encoders()
        #drive autonomously until Ctrl-C pressed
        #check if an obstacle faced within 250mm (25cm)
        while not (trigger.is_set() or dist<=250):
            gopigo3_robot.forward()
            dist = my_distance_sensor.read_mm()
            print("Distance Sensor Reading: {} mm ".format(dist))
            #enocder average values to get distance in cm moved
            encoders_read = round(gopigo3_robot.read_encoders_average())
            
            #write sensor values to a file
            row = [i, encoders_read, dist]
            with open('problem1_pathtrace.csv', 'a') as csvFile:
                writer = csv.writer(csvFile)
                writer.writerow(row)
                csvFile.close()
            i+=1

            if(encoders_read) >= 50:
                #if 50 cm crossed, stop and take turn
                gopigo3_robot.stop()
                if(right):
                    gopigo3_robot.turn_degrees(90)
                else:
                    gopigo3_robot.turn_degrees(-90)
                #recursive drive square
                drive_and_turn(i,right)
                break
        #when object within 25cm encountered
        #j = 1
        gopigo3_robot.stop()
        #yyyyyyTODO CODE################
        if dist < 250:
            output = np.empty((480, 640, 3), dtype = np.uint8)
            with picamera.PiCamera() as camera:
                camera.resolution = (640, 480)
                camera.capture(output, format = 'rgb', use_video_port = True)
            img = Image.fromarray(output)
            img.save("./photo_of_incident_"+str(j)+".jpg")
            j+=1
        #yyyyyyyTODO CODE################
        gopigo3_robot.turn_degrees(180)
        #travel back rest of the distance (using enooder value which may not be robust value) 
        #and take a turn(L/R) before continuing the square path again
        gopigo3_robot.drive_cm(encoders_read)
        if(right):
            gopigo3_robot.turn_degrees(-90)
            right = 0
        else:
            gopigo3_robot.turn_degrees(90)
            right = 1
        drive_and_turn(i,right)

        return

    drive_and_turn(i,right)

    #never returns
    
    #Stan here, above says that it never returns in the original code for squarepath, annoying af. 
    #So we need to add a stop command?
    return
#TODOCODE################################################################333

def orientate(trigger, simultaneous_launcher, sensor_queue):
    """
    Thread-launched function for reading the compass data off of the IMU sensor. The data is then
    interpreted and then it's loaded in a queue.

    :param trigger: CTRL-C event. When it's set, it means CTRL-C was pressed and the thread needs to stop.
    :param simultaneous_launcher: It's a barrier used for synchronizing all threads together.
    :param sensor_queue: Queue where the processed data of the compass is put in.
    :return: Nothing.

    """

    time_to_put_in_queue = 0.2 # measured in seconds
    time_to_wait_after_error = 0.5 # measured in seconds

    # try instantiating an InertialMeasurementUnit object
    try:
        imu = inertial_measurement_unit.InertialMeasurementUnit(bus = "GPG3_AD1")
    except Exception as msg:
        print(str(msg))
        simultaneous_launcher.abort()

    # start the calibrating process of the compass
    print("Rotate the GoPiGo3 robot with your hand until it's fully calibrated")
    try:
        compass = imu.BNO055.get_calibration_status()[3]
    except Exception as msg:
        compass = 0
    values_already_printed = []
    max_conseq_errors = 3

    while compass != 3 and not trigger.is_set() and max_conseq_errors > 0:
        state = ""
        if compass == 0:
            state = "not yet calibrated"
        elif compass == 1:
            state = "partially calibrated"
        elif compass == 2:
            state = "almost calibrated"

        if not compass in values_already_printed:
            print("The GoPiGo3 is " + state)
        values_already_printed.append(compass)

        try:
            compass = imu.BNO055.get_calibration_status()[3]
        except Exception as msg:
            max_conseq_errors -= 1
            sleep(time_to_wait_after_error)
            continue

    # if CTRL-C was triggered or if the calibration failed
    # then abort everything
    if trigger.is_set() or max_conseq_errors == 0:
        print("IMU sensor is not reacheable or kill event was triggered")
        simultaneous_launcher.abort()
    else:
        state = "fully calibrated"
        print("The GoPiGo3 is " + state)

    # point of synchronizing all threads together (including main)
    # it fails if abort method was called
    try:
        simultaneous_launcher.wait()
    except threading.BrokenBarrierError as msg:
        print("[orientate] thread couldn't fully start up")

    # while CTRl-C is not pressed and while the synchronization went fine
    while not (trigger.is_set() or simultaneous_launcher.broken):
        five_values = 10
        heading_list = []
        max_conseq_errors = 3

        # get the north point
        # extract a couple of values before going to the next procedure
        while five_values > 0 and max_conseq_errors > 0:
            try:
                heading_list.append(getNorthPoint(imu))
            except Exception as msg:
                max_conseq_errors -= 1
                sleep(time_to_wait_after_error)
                continue
            five_values -= 1
        if max_conseq_errors == 0:
            print("IMU is not reacheable")
            trigger.set()
            break

        # apply some filtering
        heading_list = statisticalNoiseReduction(heading_list)
        heading_avg = mean(heading_list)

        # and then try to put it in the queue
        # if the queue is full, then just go to the next iteration of the while loop
        try:
            sensor_queue.put(heading_avg, timeout = time_to_put_in_queue)
        except queue.Full:
            pass

def Main():
    
    #simultaneous_launcher = threading.Barrier(3) # synchronization object
    #motor_command_queue = queue.Queue(maxsize = 2) # queue for the keyboard commands
    #sensor_queue = queue.Queue(maxsize = 1) # queue for the IMU sensor
    #keyboard_refresh_rate = 20.0 
    
    
    # GoPiGo3WithKeyboard is used for mapping
    # keyboard keys to actual GoPiGo3 commands
    # the keys-to-gopigo3 bindings are defined inside the class
    gopigo3_servos = GoPiGo3WithKeyboard()

    # draws the GoPiGo3 logo
    gopigo3_servos.drawLogo()
    # writes some description on the GoPiGo3
    gopigo3_servos.drawDescription()
    # writes the menu for controlling the GoPiGo3 robot
    # key bindings are shown in here
    gopigo3_servos.drawMenu()

    #orientate_thread = threading.Thread(target = orientate, args = (trigger, simultaneous_launcher, sensor_queue))
    #robotcontrol_thread = threading.Thread(target = robotControl, args = (trigger, simultaneous_launcher, motor_command_queue, sensor_queue))
    #orientate_thread.start()
    # result holds the exit string when we call a gopigo3 command
    # with the GoPiGo3WithKeyboard object
    result = "nothing"
    """
    result can take the following values:
    "complete_turn_servo1", "complete_turn_servo2",
    "gradual_turn_servo1", "gradual_turn_servo2",
    "kill_servos", "exit"
    """

    successful_exit = True
    refresh_rate = 20.0

    with Input(keynames = "curtsies", sigint_event = True) as input_generator:
        while True:
            period = 1 / refresh_rate
            # if nothing is captured in [period] seconds
            # then send() function returns None
            key = input_generator.send(period)

            # if we've captured something from the keyboard
            if key is not None:
                result = gopigo3_servos.executeKeyboardJob(key)

                if result == "exit":
                    break


if __name__ == "__main__":
    # set up a handler for ignoring the Ctrl+Z commands
    signal.signal(signal.SIGTSTP, lambda signum, frame : print("Press the appropriate key for closing the app."))

    try:
        Main()
    except IOError as error:
        # if the GoPiGo3 is not reachable
        # then print the error and exit
        print(str(error))
        exit(1)

    exit(0)
