# For use with Airsim for a custom path not using Airsim's path algorithms

# -*- coding: utf-8 -*-
"""
Created on Fri May 22 10:31:29 2020

@author: lukef
"""


import setup_path 
import airsim
import cv2
import numpy as np
import os
import pprint
from math import atan2,degrees
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
# from Plotter import plotUpdate
from squaternion import Quaternion
from scipy.interpolate import interp1d
from EntropyCalculation import calEntropy, ImageReturn



# Current Settings Json
# {
#   "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
#   "SettingsVersion": 1.2,
#   "RpcEnabled": true,
#   "SimMode": "Multirotor",
# "Vehicles": {
#     "Drone2": {
#       "VehicleType": "SimpleFlight",
#       "X": 25,
#       "Y": 25,
#       "Z": -2,
#       "Yaw":90,
    #   "AllowAPIAlways": true
    # },
    # "Drone1": {
    #   "VehicleType": "SimpleFlight",
    #   "X": 150,
    #   "Y": 25,
    #   "Z": -2,
    #   "Yaw":90,
    #   "AllowAPIAlways": true
    # },
    # "Drone3": {
    #   "VehicleType": "SimpleFlight",
    #   "X": -100,
    #   "Y": 25,
    #   "Z": -2,
    #   "Yaw": 90,
#       "AllowAPIAlways": true
#     }

#   }
# }

"""
Current Drone Orientation is


Drone3          Drone2               Drone1
"""
#Initial Positions on Drones must match settings file




pos0 = np.zeros((1,3))

q_state = np.zeros(3) #Preallocate state vectors
q_orien = np.zeros(4) #Preallocate orientation vectors
qxy = np.zeros(2)     #Preallocate position vectors

# Drone 1
pos0[0,0] = 150
pos0[0,1] = 25.0
pos0[0,2] = -2.0





waypoints = np.loadtxt(fname = "waypoints.txt")



with open('entropySaveCustomPath.csv', 'w') as csvfile:
    fileWrite = csv.writer(csvfile, delimiter=' ', quotechar='|')
    fileWrite.writerow(['X1','Y1'])

    



# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, "Drone1")
client.armDisarm(True, "Drone1")




iteration = 0
beginningTime = client.getMultirotorState(vehicle_name = 'Drone1').timestamp






    
#Drone 1



########################## Note: positions are relative to world frame and getImu function is how to call realtive to Drone frame
#Get x-y-z coordinates
pos1 = client.simGetGroundTruthKinematics(vehicle_name = 'Drone1')
q1 = np.array([pos1.position.x_val, pos1.position.y_val, pos1.position.z_val])
qo1 = np.array([pos1.orientation.w_val, pos1.orientation.x_val, pos1.orientation.y_val, pos1.orientation.z_val])
currentTime = client.getMultirotorState(vehicle_name = 'Drone1').timestamp 
timeDiff = (currentTime-beginningTime)/1000000000

 # Add initial coordinates
# Drone 1
qd1 = q1 + pos0[0,:]

# 3D and 2D state vector
# Drone 1
q_state[0:3] =  qd1.copy()
qxy[0:2] = np.array([qd1[0], qd1[1]])  
q_orien[0:4] =  qo1.copy()




qua1 = Quaternion(q_orien[0], q_orien[1], q_orien[2], q_orien[3])
euler1 = qua1.to_euler(degrees = False)



posUAV1 = np.array([qxy[0],qxy[1],euler1[2]])


if(iteration % 5 == 0):
    with open('entropySave.csv', 'a') as csvfile:
        fileWrite = csv.writer(csvfile, delimiter=' ', quotechar='|')
        fileWrite.writerow([qxy[0],qxy[1]])





f1 = client.moveOnPathAsync(waypoints, 2, timeout_sec = 3e+38, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), vehicle_name = 'Drone1'):

f1.join()


client.armDisarm(False, "Drone1")

client.reset()


client.enableApiControl(False, "Drone1")



# Plots at end of Simulation to have only one Python file call in terminal
# import Plotter


