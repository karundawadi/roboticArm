#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from math import degrees, radians, sqrt, pow, atan, acos, pi

arm_length = 0.165 # in m
wrist_length = 0.145 # in m
joint1 = Motor(Port.A,Direction.CLOCKWISE) # Using our actual figure to derive this 
joint2 = Motor(Port.B,Direction.CLOCKWISE) # Using our actual figure to derive this 

ev3 = EV3Brick()

# Both x and y are in float and in m 
# Condition for x and y :
# Both can never be zero as our lenght of arm and wrist do not reach 0,0 and set of couple of places ; if needed we can add padding 
def inverseKinematics(x,y):
    
    if x < arm_length-wrist_length and y < arm_length-wrist_length: # Cause we never can reach this point  
        print("Cannot perform this as it is beyond the computational range")
        return False 
    
    # Using the derivation sheet 
    beta = pi/2 # Means that initially the angle is 90% for case in which x == 0
    if x != 0:
        beta = atan(y/x) # Cause if X is 0 means we are undefined; which is 90
    alpha = acos(
        (
            pow(arm_length,2) + pow(x,2) + pow(y,2) - pow(wrist_length,2)
            )/(
            2*arm_length*sqrt(pow(x,2)+pow(y,2))
        )
    )
    theta_one = alpha + beta 
    
    theta_two = acos(
        (
            pow(x,2) + pow(y,2) - pow(arm_length,2) - pow(wrist_length,2)
        )/(
            2 * arm_length * wrist_length
        )
    )
    
    # Cause both are in radians 
    return (degrees(theta_one),degrees(theta_two))

# Iterator is the amount to increment by
# So if this is 0.1 we have 1, 1.1, 1.2 and so on
def interpolateLine(A,B,_iterator):
    x_A,y_A = A
    x_B,y_B = B
    # Distance is a float 
    distance = sqrt(
        pow((x_B-x_A),2)+pow((y_B-y_A),2)
    ) 
    # n is the amount of increments we want 
    n = 0
    x,y = -1,-1
    cords = []
    while (x,y) != (x_B,y_B) and n < distance:
        x = x_A + (n/distance * (x_B - x_A))
        y = y_A + (n/distance * (y_B-y_A))
        n+=_iterator # How much we want to increase by 
        cords.append((x,y))
    cords.append((x_B,y_B))
    return cords

# Returns the overall array; need to go through it 
# The provided points needs to be in the correct order 
def interpolate(provided_points,increment_amount):
    
    # To make points subsequent 
    def path_cleanup(points):
        j = 1
        new_array = []
        new_array.append(points[0])
        while j < len(points):
            if points[j-1] == points[j]:
                pass
            else:
                new_array.append(points[j])
            j+=1
        return new_array
    
    overall_path = []
    for i in range(len(provided_points)):
        j = i+1
        if j < len(provided_points):
            value = interpolateLine(provided_points[i],provided_points[j],increment_amount)
            for i in value:
                overall_path.append(i)
    
    # Cleaning up the path; if we have simultanous same points make it one 
    overall_path = path_cleanup(overall_path)
    return overall_path

def walk_cords(path):
    ran_first_time = False 
    for i in path:
        x,y = i 
        print(x,y)
        joint1_angle , joint2_angle  = inverseKinematics(x,y)
        # Moving to the angle location 
        joint2.run_target(10,-joint2_angle,then=Stop.HOLD,wait=False) # Cause we need to move to the opposite angle 
        joint1.run_target(10,joint1_angle,then=Stop.HOLD,wait=True)
        
        if ran_first_time ==  False:
            ev3.speaker.say("Location reached; place pen")
            wait(8000)
            ev3.speaker.say("Starting to draw")
        wait(400)
        ran_first_time = True 

# Should be in the correct order 
provided_points = [(0.05,0.03),(0.12,0.03),(0.07,0.08),(0.12,0.08),(0.05,0.03)]

# Increment amount is the amount to move by; provided at the end 
path = interpolate(provided_points,0.005)

walk_cords(path)

# Indicates program end 
ev3.speaker.beep()
