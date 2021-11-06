# CSC4120-F2021
Repository for files pertaining to Georgia State University's Intro to Robotics course.

Stanley Hu

Checkpoint 1 is intended to include all the material of the previous 3 modules, by combining them together so that the user may execute them from a single function.

Checkpoint1, original version, allows the user to change the light color, print out general info about the robot, navigate by the cardinal directions, and also continuously travel in a square path, altering its course if obstacles are detected within 25 cm. 

However, the code within it that allows the user to change the directions for the distance sensor and the camera are currently non functional, as are the functions that would allow the user to independantly control and direct the movements of the robot.

Checkpoint1v2, comes along with a second file, keyboard2, which is used to store the commands available to the user. To activate, the user merely needs to run Checkpoint1v22 on python, and it will import keyboard2 as long as they are contained in the same folder.

Currently, through the combined use of keyboard2 and Checkpoint1v2, the user may turn both the distance sensor and the camera independantly, take photos, print general info, flash the lights of the robot, control the robots movements through the ijkl keys for forward, right, back, and left, and also direct the robot to move in a square path, changing trajectory should an obstacle be detected within 25cm.

However, although the code to do so is contained in the file, at the present moment the robot cannot detect the cardinal directions using the v2 file, and it cannot orient itself along those lines.
