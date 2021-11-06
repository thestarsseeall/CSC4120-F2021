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

#Team 13 2021

import random
import easygopigo3 as easy
import threading
from time import sleep
import picamera

import numpy as np
from curtsies import Input
from di_sensors import inertial_measurement_unit
from easygopigo3 import *
import time

from ipywidgets import widgets, Layout
my_gpg3 = easy.EasyGoPiGo3()
GPG = easy.EasyGoPiGo3()

import picamera
import matplotlib.pyplot as plt
from PIL import Image

j = 1
#number of photos done by squarepath when encountering obstacles.
manPhoto = 1
#number of photos manually taken usen the command function

MINIMUM_VOLTAGE = 7.0
DEBUG = False
MOTORS_SPEED = 250 # see documentation
#TODO###############
import csv
#OPTIONAL###########
#for Georgia  ##-5 29#######
MAGNETIC_DECLINATION =  -5
#TODO###############

class GoPiGo3WithKeyboard(object):
    """
    Class for interfacing with the GoPiGo3.
    It's functionality is to map different keys
    of the keyboard to different commands of the GoPiGo3.
    """

    KEY_DESCRIPTION = 0
    KEY_FUNC_SUFFIX = 1

    servo1_position = 0
    servo2_position = 0
    servo_increment_step = 5


    def __init__(self):
        """
        Instantiates the key-bindings between the GoPiGo3 and the keyboard's keys.
        Sets the order of the keys in the menu.
        """
        self.gopigo3 = easy.EasyGoPiGo3()
        self.servo1 = self.gopigo3.init_servo("SERVO1")
        self.servo2 = self.gopigo3.init_servo("SERVO2")

        self.keybindings = {
        "<F1>" : ["Turn SERVO1 completely to 0 degrees", "leftservo1_immediately"],
        "<F2>" : ["Turn SERVO1 completely to 180 degrees", "rightservo1_immediately"],

        "<F5>" : ["Turn SERVO2 completely to 0 degrees", "leftservo2_immediately"],
        "<F6>" : ["Turn SERVO2 completely to 180 degrees", "rightservo2_immediately"],

        "a" : ["Turn SERVO1 towards 0 degrees incrementely", "leftservo1_incrementally"],
        "d" : ["Turn SERVO1 towords 180 degrees incrementely", "rightservo1_incrementally"],

        "<LEFT>" : ["Turn SERVO2 towards 0 degrees incrementely", "leftservo2_incrementally"],
        "<RIGHT>" : ["Turn SERVO2 towards 180 degrees incrementely", "rightservo2_incrementally"],

        "<SPACE>" : ["Turn off power supply to both servos", "kill"],

        "<ESC>" : ["Exit", "exit"],
            
        "i" : ["Move robot Forward", "Move_Forward"],
        "j" : ["Turn robot Left", "Turn_Left"],
        "l" : ["Turn robot Right", "Turn_Right"],
        "k" : ["Move robot Back", "Move_Back"],
        "t" : ["Take Photo", "TakePhoto"],
        "c" : ["Change light colors", "Lights"],
        "p" : ["Activate Squarepath routine", "SquarePath"]
        }
        self.order_of_keys = ["<F1>", "<F2>", "<F5>", "<F6>", "a", "d", "<LEFT>", "<RIGHT>", "<SPACE>", "<ESC>" "i", "j", "l", "k", "t", "c", "p"]

    def executeKeyboardJob(self, argument):
        """
        Argument can be any of the strings stored in self.keybindings list.

        For instance: if argument is "w", then the algorithm looks inside self.keybinds dict and finds
        the "forward" value, which in turn calls the "_gopigo3_command_forward" method
        for driving the gopigo3 forward.

        The return values are:
        * "nothing" - when no method could be found for the given argument.
        * "moving" - when the robot has to move forward, backward, to the left or to the right for indefinite time.
        * "path" - when the robot has to move in a direction for a certain amount of time/distance.
        * "static" - when the robot doesn't move in any direction, but instead does static things, such as turning the LEDs ON.
        * "exit" - when the key for exiting the program is pressed.
        """
        method_prefix = "_gopigo3_command_"
        try:
            method_suffix = str(self.keybindings[argument][self.KEY_FUNC_SUFFIX])
        except KeyError:
            method_suffix = ""
        method_name = method_prefix + method_suffix

        method = getattr(self, method_name, lambda : "nothing")

        return method()

    def drawLogo(self):
        """
        Draws the name of the GoPiGo3.
        """
        print("   _____       _____ _  _____         ____  ")
        print("  / ____|     |  __ (_)/ ____|       |___ \ ")
        print(" | |  __  ___ | |__) || |  __  ___     __) |")
        print(" | | |_ |/ _ \|  ___/ | | |_ |/ _ \   |__ < ")
        print(" | |__| | (_) | |   | | |__| | (_) |  ___) |")
        print("  \_____|\___/|_|   |_|\_____|\___/  |____/ ")
        print("                                            ")

    def drawDescription(self):
        """
        Prints details related on how to operate the GoPiGo3.
        """
        print("\nPress the following keys to run the features of the GoPiGo3.")
        print("To rotate the servos, make sure you have a fresh set of batteries powering the GoPiGo3.")
        print("Use servos that can turn to 180 degrees as some have difficulties doing it.\n")

    def drawMenu(self):
        """
        Prints all the key-bindings between the keys and the GoPiGo3's commands on the screen.
        """
        try:
            for key in self.order_of_keys:
                print("\r[key {:8}] :  {}".format(key, self.keybindings[key][self.KEY_DESCRIPTION]))
        except KeyError:
            print("Error: Keys found GoPiGo3WithKeyboard.order_of_keys don't match with those in GoPiGo3WithKeyboard.keybindings.")

    def _gopigo3_command_leftservo1_immediately(self):
        self.servo1_position = 0
        self.servo1.rotate_servo(self.servo1_position)

        return "complete_turn_servo1"

    def _gopigo3_command_rightservo1_immediately(self):
        self.servo1_position = 180
        self.servo1.rotate_servo(self.servo1_position)

        return "complete_turn_servo1"


    def _gopigo3_command_leftservo2_immediately(self):
        self.servo2_position = 0
        self.servo2.rotate_servo(self.servo2_position)

        return "complete_turn_servo2"

    def _gopigo3_command_rightservo2_immediately(self):
        self.servo2_position = 180
        self.servo2.rotate_servo(self.servo2_position)

        return "complete_turn_servo2"

    def _gopigo3_command_leftservo1_incrementally(self):
        self.servo1_position -= self.servo_increment_step
        if self.servo1_position < 0:
            self.servo1_position = 0
        self.servo1.rotate_servo(self.servo1_position)

        return "gradual_turn_servo1"

    def _gopigo3_command_rightservo1_incrementally(self):
        self.servo1_position += self.servo_increment_step
        if self.servo1_position > 180:
            self.servo1_position = 180
        self.servo1.rotate_servo(self.servo1_position)

        return "gradual_turn_servo1"

    def _gopigo3_command_leftservo2_incrementally(self):
        self.servo2_position -= self.servo_increment_step
        if self.servo2_position < 0:
            self.servo2_position = 0
        self.servo2.rotate_servo(self.servo2_position)

        return "gradual_turn_servo2"

    def _gopigo3_command_rightservo2_incrementally(self):
        self.servo2_position += self.servo_increment_step
        if self.servo2_position > 180:
            self.servo2_position = 180
        self.servo2.rotate_servo(self.servo2_position)

        return "gradual_turn_servo2"

    def _gopigo3_command_kill(self):
        self.servo1.disable_servo()
        self.servo2.disable_servo()
        my_gpg3.stop()
        return "kill_servos"

    def _gopigo3_command_exit(self):
        return "exit"

    def _gopigo3_command_Move_Forward(self):
        my_gpg3.forward()
        return "gradual move forward"
    def _gopigo3_command_Turn_Left(self):
        my_gpg3.left()
        return "gradual turn left"
    def _gopigo3_command_Turn_Right(self):
        my_gpg3.right()  
        return "gradual turn right"
    def _gopigo3_command_Move_Back(self):
        my_gpg3.backward()
        return "gradual move back"
    def _gopigo3_command_TakePhoto(self):
        manPhoto=1
        output = np.empty((480, 640, 3), dtype = np.uint8)
        with picamera.PiCamera() as camera:
            camera.resolution = (640, 480)
            camera.capture(output, format = 'rgb', use_video_port = True)
        img = Image.fromarray(output)
        img.save("./manual_taken_photo_"+str(manPhoto)+".jpg")
        print("./manual_taken_photo_"+str(manPhoto)+".jpg saved to file")
        manPhoto+=1
        return "take photo"
    def _gopigo3_command_Lights(self):
        colors = [ (255,0,0), (255,255,0), (255,255,255), (0,255,0), (0,255,255), (0,0,255), (0,0,0)]
        for color in colors:
            GPG.set_eye_color(color)
            GPG.open_eyes()
            time.sleep(0.5)
        return "Activate Light Show"
    def _gopigo3_command_SquarePath(self):
        gopigo3_robot = EasyGoPiGo3()
        my_distance_sensor = gopigo3_robot.init_distance_sensor()
        #####j = 0
        
        #set NORTH before staring
        right = 1
        i = 0
        limitVar = 0 #limits the amount of time it will turn.
        with open('problem1_pathtrace.csv', 'w') as csvfile:
            fieldnames = ['row_num', 'encoder_value','distance_value']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

        def drive_and_turn(i,right,dist=999):
            gopigo3_robot.reset_encoders()
            #drive autonomously until Ctrl-C pressed
            #check if an obstacle faced within 250mm (25cm)
            #while not (trigger.is_set() or dist<=250): #Original line, removed.
            while not (dist<=250):
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
            #drive_and_turn(i,right)
            #issue with drive and turn is that it is recursive. Need a way to stop or disturb the program.
    
            return
        
        #drive_and_turn(i,right)
        while(limitVar<5):
            drive_and_turn(i,right)
            limitVar+=1
            #created the LimitVar variable to restrict the amount of time drive and turn runs.
        #issue with drive and turn is that it is recursive. Need a way to stop or disturb the program.
        #never returns
        
        #Stan here, above says that it never returns in the original code for squarepath, annoying af. 
        #So we need to add a stop command?
        return "Squarepath Routine"
