"""
## License
 GoPiGo for the Raspberry Pi: an open source robotics platform for the Raspberry Pi.
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

#Team 13, 2021

import queue
import signal
import threading
from math import *
from statistics import mean
from time import sleep

import numpy as np
from curtsies import Input
from di_sensors import inertial_measurement_unit
from easygopigo3 import *
import time

from ipywidgets import widgets, Layout
import easygopigo3 as easy

my_gpg3 = easy.EasyGoPiGo3()
GPG = easy.EasyGoPiGo3()

MINIMUM_VOLTAGE = 7.0
DEBUG = False
MOTORS_SPEED = 250 # see documentation
#TODO###############
import csv
#OPTIONAL###########
#for Georgia  ##-5 29#######
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

#self.gopigo3 = easy.EasyGoPiGo3()
#self.servo1 = self.gopigo3.init_servo("SERVO1")
#self.servo2 = self.gopigo3.init_servo("SERVO2")

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
    j = 0
    
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
        j = 1
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


def robotControl(trigger, simultaneous_launcher, motor_command_queue, sensor_queue):
    """
    Thread-launched function for orientating the robot around. It gets commands from the keyboard as well
    as data from the sensor through the sensor_queue queue.

    :param trigger: CTRL-C event. When it's set, it means CTRL-C was pressed and the thread needs to stop.
    :param simultaneous_launcher: It's a barrier used for synchronizing all threads together.
    :param motor_command_queue: Queue containing commands from the keyboard. The commands are read from within main.
    :param sensor_queue: Processed data off of the IMU. The queue is intended to be read.
    :return: Nothing.

    """

    time_to_wait_in_queue = 0.1 # measured in

    # try to connect to the GoPiGo3
    try:
        gopigo3_robot = EasyGoPiGo3()
        #TODO CODE################
        #my_distance_sensor = gopigo3_robot.init_distance_sensor()
        #TODO CODE################
    except IOError:
        print("GoPiGo3 robot not detected")
        simultaneous_launcher.abort()
    except gopigo3.FirmwareVersionError:
        print("GoPiGo3 board needs to be updated")
        simultaneous_launcher.abort()
    except Exception:
        print("Unknown error occurred while instantiating GoPiGo3")
        simultaneous_launcher.abort()

    # synchronizing point between all threads
    # if abort method was called, then the synch will fail
    try:
        simultaneous_launcher.wait()
    except threading.BrokenBarrierError as msg:
        print("[robotControl] thread couldn't be launched")

    # if threads were successfully synchronized
    # then set the GoPiGo3 appropriately
    if not simultaneous_launcher.broken:
        gopigo3_robot.stop()
        gopigo3_robot.set_speed(MOTORS_SPEED)

    direction_degrees = None
    move = False
    acceptable_error_percent = 8
    command = "stop"
    rotational_factor = 0.30
    accepted_minimum_by_drivers = 6

    # while CTRL-C is not pressed, the synchronization between threads didn't fail and while the batteries' voltage isn't too low
    while not (trigger.is_set() or simultaneous_launcher.broken or gopigo3_robot.volt() <= MINIMUM_VOLTAGE):
        # read from the queue of the keyboard
        self = trigger
        self.gopigo3 = easy.EasyGoPiGo3()
        self.servo1 = self.gopigo3.init_servo("SERVO1")
        self.servo2 = self.gopigo3.init_servo("SERVO2")
        try:
            command = motor_command_queue.get(timeout = time_to_wait_in_queue)
            motor_command_queue.task_done()
        except queue.Empty:
            pass

        # make some selection depending on what every command represents
        if command == "stop":
            move = False
            my_gpg3.stop()
        elif command == "move":
            move = True
            my_gpg3.forward()
        if command == "west":
            direction_degrees = -90.0
        elif command == "east":
            direction_degrees = 90.0
        elif command == "north":
            direction_degrees = 0.0
        elif command == "south":
            direction_degrees = 180.0
        elif command == "left":
            my_gpg3.left()
        elif command == "back":
            my_gpg3.backward()
        elif command == "right":
            my_gpg3.right()        
        elif command == "squarepath":
            squarepath(trigger)
            # should command 1 iteration of the squarepath command.
        elif command == "film":
            if filmvar == 0:
                print("Filming started")
                with picamera.PiCamera() as camera:
                    camera.resolution = (640, 480)
                    #camera.start_recording('2021Module3P2V3.h264')
                    camera.start_recording("./film_"+str(filmNum)+".h264")
                    camera.wait_recording(30)
                    filmNum+=1
            elif filmvar == 1:
                camera.stop_recording()
                print("Film ended.")
        elif command == "Serv1_left":
            self.servo1_position -= self.servo_increment_step
            if self.servo1_position < 0:
                self.servo1_position = 0
            self.servo1.rotate_servo(self.servo1_position)
        elif command == "Serv1_right":
            self.servo1_position += self.servo_increment_step
            if self.servo1_position > 180:
                self.servo1_position = 180
            self.servo1.rotate_servo(self.servo1_position)
        elif command == "Serv2_left":
            self.servo2_position -= self.servo_increment_step
            if self.servo2_position < 0:
                self.servo2_position = 0
            self.servo2.rotate_servo(self.servo2_position)
        elif command == "Serv2_right":
            self.servo2_position += self.servo_increment_step
            if self.servo2_position > 180:
                self.servo2_position = 180
            self.servo2.rotate_servo(self.servo2_position)
        elif command == "takephoto":
            output = np.empty((480, 640, 3), dtype = np.uint8)
            with picamera.PiCamera() as camera:
                camera.resolution = (640, 480)
                camera.capture(output, format = 'rgb', use_video_port = True)
            img = Image.fromarray(output)
            img.save("./manual_taken_photo_"+str(manPhoto)+".jpg")
            print("./manual_taken_photo_"+str(manPhoto)+".jpg saved to file")
            manPhoto+=1
        elif command == "lights":
            colors = [ (255,0,0), (255,255,0), (255,255,255), (0,255,0), (0,255,255), (0,0,255), (0,0,0)]
            for color in colors:
                GPG.set_eye_color(color)
                GPG.open_eyes()
                time.sleep(0.5)
        elif command =="geninfo":
            print("Manufacturer    : ", GPG.get_manufacturer()    ) # read and display the serial number
            print("Board           : ", GPG.get_board()           ) # read and display the serial number
            print("Serial Number   : ", GPG.get_id()              ) # read and display the serial number
            print("Hardware version: ", GPG.get_version_hardware()) # read and display the hardware version
            print("Firmware version: ", GPG.get_version_firmware()) # read and display the firmware version
            print("Battery voltage : ", GPG.get_voltage_battery() ) # read and display the current battery voltage
            print("5v voltage      : ", GPG.get_voltage_5v()      ) # read and display the current 5v regulator voltage
        # if a valid orientation was selected
        if direction_degrees is not None:
            # read data and calculate orientation
            heading = sensor_queue.get()
            if direction_degrees == 180.0:
                heading_diff = (direction_degrees - abs(heading)) * (-1 if heading < 0 else 1)
                error = abs(heading_diff / direction_degrees) * 100
            else:
                heading_diff = direction_degrees - heading
                error = abs(heading_diff / 180) * 100

            how_much_to_rotate = int(heading_diff * rotational_factor)

            if DEBUG is True:
                print("direction_degrees {} heading {} error {} heading_diff {}".format(direction_degrees, heading, error, heading_diff))

            # check if the heading isn't so far from the desired orientation
            # if it needs correction, then rotate the robot
            if error >= acceptable_error_percent and abs(how_much_to_rotate) >= accepted_minimum_by_drivers:
                gopigo3_robot.turn_degrees(how_much_to_rotate, blocking = True)

        # command for making the robot move of stop
        
        #This is the part that initializes squarepath. Try stopping or editing this part out so that it can continue to recieve commands?
        #Compare and contrast with the files from the other area.
        if move is False:
            gopigo3_robot.stop()
        else:
            #TODOCODE###########
            #set NORTH when staring
            #squarepath(trigger)
            #TODOCODE###########            
            sleep(0.001)

    # if the synchronization wasn't broken
    # then stop the motors in case they were running
    if not simultaneous_launcher.broken:
        gopigo3_robot.stop()


def Main(trigger):
    """
    Main thread where the other 2 threads are started, where the keyboard is being read and
    where everything is brought together.

    :param trigger: CTRL-C event. When it's set, it means CTRL-C was pressed and all threads are ended.
    :return: Nothing.

    """
    simultaneous_launcher = threading.Barrier(3) # synchronization object
    motor_command_queue = queue.Queue(maxsize = 50) # queue for the keyboard commands
    sensor_queue = queue.Queue(maxsize = 10) # queue for the IMU sensor
    keyboard_refresh_rate = 20.0 # how many times a second the keyboard should update
    available_commands = {"<LEFT>": "west",
                          "<RIGHT>": "east",
                          "<UP>": "north",
                          "<DOWN>": "south",
                          "<SPACE>": "stop",
                          "w": "move",
                          "a": "left",
                          "s": "back",
                          "d": "right",
                          "p": "squarepath",
                          "f": "film",
                          "l": "lights",
                          "j": "Serv1_left",
                          "k": "Serv1_right",
                          "i": "Serv2_left",
                          "o": "Serv2_right",
                          "g": "geninfo",
                          "t": "takephoto"} # the selectable options within the menu
                          #p is for path, as in squarepath. wasd directions, arrows for cardinal directions, 
                        
    menu_order = ["<LEFT>", "<RIGHT>", "<UP>", "<DOWN>", "<SPACE>", "w", "a", "s", "d", "p", "f", "j", "k", "i", "o", "l","g", "t"] # and the order of these options

    print("   _____       _____ _  _____         ____  ")
    print("  / ____|     |  __ (_)/ ____|       |___ \ ")
    print(" | |  __  ___ | |__) || |  __  ___     __) |")
    print(" | | |_ |/ _ \|  ___/ | | |_ |/ _ \   |__ < ")
    print(" | |__| | (_) | |   | | |__| | (_) |  ___) |")
    print("  \_____|\___/|_|   |_|\_____|\___/  |____/ ")
    print("                                            ")

    # starting the workers/threads
    orientate_thread = threading.Thread(target = orientate, args = (trigger, simultaneous_launcher, sensor_queue))
    robotcontrol_thread = threading.Thread(target = robotControl, args = (trigger, simultaneous_launcher, motor_command_queue, sensor_queue))
    orientate_thread.start()
    robotcontrol_thread.start()
    #with picamera.PiCamera() as camera:
    #    camera.resolution = (640, 480)
    #    camera.start_recording('2021Module3P2V3.h264')
    #    camera.wait_recording(30)
    #    camera.stop_recording()

    # if the threads couldn't be launched, then don't display anything else
    try:
        simultaneous_launcher.wait()

        print("Press the following keys for moving/orientating the robot by the 4 cardinal points")
        for menu_command in menu_order:
            print("{:8} - {}".format(menu_command, available_commands[menu_command]))
    except threading.BrokenBarrierError:
        pass
    
    result = "nothing"
    successful_exit = True
    refresh_rate = 20.0
    #above 3 lines added to see if it will help with latency.
    
    
    # read the keyboard as long as the synchronization between threads wasn't broken
    # and while CTRL-C wasn't pressed
    #adding in sigint to see if it will help timing.
    with Input(keynames = "curtsies", sigint_event = True) as input_generator:
        while not (trigger.is_set() or simultaneous_launcher.broken):
            period = 1 / keyboard_refresh_rate
            key = input_generator.send(period)

            if key in available_commands:
                try:
                    motor_command_queue.put_nowait(available_commands[key])
                except queue.Full:
                    pass

    # exit codes depending on the issue
    if simultaneous_launcher.broken:
        sys.exit(1)
    sys.exit(0)
#original main runner. Editing in new version in the hopes that it runs better.
#if __name__ == "__main__":
#    trigger = threading.Event() # event used when CTRL-C is pressed
#    signal.signal(signal.SIGINT, lambda signum, frame : trigger.set()) # SIGINT (CTRL-C) signal handler
#    Main(trigger)


if __name__ == "__main__":
    trigger = threading.Event() # event used when CTRL-C is pressed 
    # Stan here, added in this above line, not previously in this version of main. 
    #Originally this main didn't have a trigger, so integrating it with the orignal main didn't work.
    
    # set up a handler for ignoring the Ctrl+Z commands
    #signal.signal(signal.SIGTSTP, lambda signum, frame : print("Press the appropriate key for closing the app."))

    try:
        Main(trigger)
    except IOError as error:
        # if the GoPiGo3 is not reachable
        # then print the error and exit
        print(str(error))
        exit(1)

    exit(0)
    
    
"""Based on our current assessment of the code, whenever a command is given, such as g for general info, the command is carried out continuously due to the "while" function, which may fill up the queue. 
THere appear to be better results as the maximum amount of possible commands in the queue is raised.

As of 11/6/21: Do not take photos. Results in a referenced before assignment error (?) I assigned the value at the very top of this page? 
Film, of course, doesn't work.
Error when trying to move the servos. Servos not defined or assigned, I think?

"""