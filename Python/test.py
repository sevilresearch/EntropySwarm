# -*- coding: utf-8 -*-
"""
Created on Fri May 22 10:31:29 2020

@author: lukef
"""
# Working on it being iterative for more than 3 uavs as of 6/30

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
numUAV = 3

# Cost Function Parameters
# Total Distance Traveled 
# f2UAV1 = 0
# f2UAV2 = 0
# f2UAV3 = 0
#UAV1, UAV2, UAV3
f2 = ([0,0,0])
# Sum of Vx and Vy, so control inputs
# f1UAV1 = 0
# f1UAV2 = 0
# f1UAV3 = 0

f1 = ([0,0,0])


pos0 = np.zeros((3,3))

q_state = np.zeros(9) #Preallocate state vectors
q_orien = np.zeros(12) #Preallocate orientation vectors
qxy = np.zeros(6)     #Preallocate position vectors



# Drone 1
pos0[0,0] = 150
pos0[0,1] = 25.0
pos0[0,2] = -2.0


# Drone 2
pos0[1,0] = 30.0
pos0[1,1] = 25.0
pos0[1,2] = -2.0

    
# Drone 3  
pos0[2,0] = -100.0
pos0[2,1] = 25.0
pos0[2,2] = -2.0

Vmax = 0.5 
# Vmax = 0.75
# Vmax = 1
# Vmax = 5
flag = 0
# Waypoint Array
# Note: (0,500) is the test point. The rest of the points are the waypoints for the corners of a square
x_waypoint = np.array([0, -50, -50, 50, 50, -50])
y_waypoint = np.array([500, 50, 200, 200, 50, 50])
# Original Waypoint
# a = 0
# a2 = a +10
# a3 = a - 10

# b = 500
# b2 = b
# b3 = b

a = x_waypoint[0]
a2 = a +10
a3 = a - 10

b = y_waypoint[0]
b2 = b
b3 = b





commandTime = 0.1
z = -2



with open('entropySave.csv', 'w') as csvfile:
    fileWrite = csv.writer(csvfile, delimiter=' ', quotechar='|')
    fileWrite.writerow(['X1','Y1','X2','Y2','X3','Y3','EntropyDrone1','EntropyDrone2','EntropyDrone3','TimeDifference','EntropyDrone1D1','EntropyDrone1D2','EntropyDrone1D3','EntropyDrone2D1','EntropyDrone2D2','EntropyDrone2D3','EntropyDrone3D1','EntropyDrone3D2','EntropyDrone3D3','Angle1','Angle2','Angle3'])

    
count = 0



# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
for i in range(numUAV):
	 name = "Drone" + str(i+1)
	client.enableApiControl(True,name)
	client.armDisarm(True,name)


# client.enableApiControl(True, "Drone1")
# client.armDisarm(True, "Drone1")
# client.enableApiControl(True, "Drone2")
# client.armDisarm(True, "Drone2")
# client.enableApiControl(True,"Drone3")
# client.armDisarm(True,"Drone3")







iteration = 0
beginningTime = client.getMultirotorState(vehicle_name = 'Drone1').timestamp

   
while iteration < (500/commandTime):
# while True:
    # client.simPause(True)
    iteration = iteration + 1
    print("Iteration = ",iteration)
    if iteration ==1:
        print("Starting Formation Control")
    
    
    #Get Drone positions for entropy formation
        
 
    

 ########################## Note: positions are relative to world frame and getImu function is how to call realtive to Drone frame
    #Get x-y-z coordinates
    # pos1 = client.simGetGroundTruthKinematics(vehicle_name = 'Drone1')
    # q1 = np.array([pos1.position.x_val, pos1.position.y_val, pos1.position.z_val])
    # qo1 = np.array([pos1.orientation.w_val, pos1.orientation.x_val, pos1.orientation.y_val, pos1.orientation.z_val])
    # currentTime = client.getMultirotorState(vehicle_name = 'Drone1').timestamp 
    # timeDiff = (currentTime-beginningTime)/1000000000
    
    # pos2 = client.simGetGroundTruthKinematics(vehicle_name = 'Drone2')
    # q2 = np.array([pos2.position.x_val, pos2.position.y_val, pos2.position.z_val])
    # qo2 = np.array([pos2.orientation.w_val, pos2.orientation.x_val, pos2.orientation.y_val, pos2.orientation.z_val])
     
     
     
    # pos3 = client.simGetGroundTruthKinematics(vehicle_name = 'Drone3')
    # q3 = np.array([pos3.position.x_val, pos3.position.y_val, pos3.position.z_val])
    # qo3 = np.array([pos3.orientation.w_val, pos3.orientation.x_val, pos3.orientation.y_val, pos3.orientation.z_val])
    #  # Add initial coordinates
    # # Drone 1
    # qd1 = q1 + pos0[0,:]
    # # Drone 2
    # qd2 = q2 + pos0[1,:]
    # # Drone 3
    # qd3 = q3 + pos0[2,:]
        
    # # 3D and 2D state vector
    # # Drone 1
    # q_state[0:3] =  qd1.copy()
    # qxy[0:2] = np.array([qd1[0], qd1[1]])  
    # q_orien[0:4] =  qo1.copy()
    
    # # Drone 2
    # q_state[3:6] =  qd2.copy()
    # qxy[2:4] = np.array([qd2[0], qd2[1]])  
    # q_orien[4:8] =  qo2.copy()
    
    # # Drone 3
    # q_state[6:9] =  qd3.copy()
    # qxy[4:6] = np.array([qd3[0], qd3[1]])  
    # q_orien[8:12] =  qo3.copy()
    
    currentTime = client.getMultirotorState(vehicle_name = 'Drone1').timestamp 
    timeDiff = (currentTime-beginningTime)/1000000000
    
     for i in range(numUAV):
	  name = "Drone" + str(i+1)
	  
	  # Get x-y-z coordinates
	  pos = client.simGetGroundTruthKinematics(vehicle_name = name)   
	  qi = np.array([pos['position']['x_val'], pos['position']['y_val'], pos['position']['z_val']])
	  qoi = np.array([pos['orientation']['w_val'], pos['orientation']['x_val'], pos['orientation']['y_val'], pos['orientation']['z_val']])
	  
	  # Add initial coordinates
	  qd = qi + pos0[i,:]
	  
	  # 3D and 2D state vector
	  q[3*i:3*i+3] =  qd.copy()
	  qxy[2*i:2*i+2] = np.array([qd[0], qd[1]])  
	  qo[4*i:4*i+4] =  qoi.copy()

	  qua = Quaternion(qo[4*i], qo[4*i+1], qo[4*i+2], qo[4*i+3])
	  euler = qua.to_euler(degrees = False)
	  posUAVs[4*i:4*i+4] = np.array([qxy[0],qxy[1],euler1[2]])


    # qua1 = Quaternion(qo[0], qo[1], qo[2], qo[3])
    # euler1 = qua1.to_euler(degrees = False)

    
    
  
    # qua2 = Quaternion(qo[4], qo[5], qo[6], qo[7])
    # euler2 = qua2.to_euler(degrees = False)
   
  

    # qua3 = Quaternion(qo[8], qo[9], qo[10], qo[11])
    # euler3 = qua3.to_euler(degrees = False)



    # posUAV1 = np.array([qxy[0],qxy[1],euler1[2]])
    # posUAV2 = np.array([qxy[2],qxy[3],euler2[2]])
    # posUAV3 = np.array([qxy[4],qxy[5],euler3[2]])


                   




  
  # For transitioning multiple waypoints
    # waypoint_proxmity = 0.8
    # if(posUAV2[0]> waypoint_proxmity *a and posUAV2[0]> waypoint_proxmity*b):
    #     k = k + 1
    #     a = x_waypoint[k]
    #     b = y_waypoint[k]
    # if(posUAV1[0]> waypoint_proxmity *a2 and posUAV1[0]> waypoint_proxmity*b2):
    #     j = j + 1
    #     a2 = x_waypoint[j] +10
    #     b2 = y_waypoint[j]
    # if(posUAV3[0]> waypoint_proxmity *a3 and posUAV3[0]> waypoint_proxmity*b3):
    #     l = l + 1
    #     a3 = x_waypoint[l] -10
    #     b3 = y_waypoint[l] - 10



    




 	posUAV1 = posUAVs[0,:]
	posUAV2 = posUAVs[1,:]
	posUAV3 = posUAV3[2,:]
	
    # print("Drone 1")
    entropyDrone1 = calEntropy(posUAV1,posUAV2,posUAV3,a2,b2,Vmax,flag)
    # print("Drone 2")
    entropyDrone2 = calEntropy(posUAV2,posUAV3,posUAV1,a,b,Vmax,flag)
    # print("Drone 3")
    entropyDrone3 = calEntropy(posUAV3,posUAV1,posUAV2,a3,b3,Vmax,flag)


    # Assigning to an array for angle
    entropyDroneAngle[0] = entropyDrone1[1] 
    entropyDroneAngle[1] = entropyDrone2[1] 
    entropyDroneAngle[2] = entropyDrone3[1] 

     # Assigning to an array for velocity
    entropyVelocity[0] = entropyDrone1[0] 
    entropyVelocity[1] = entropyDrone2[0] 
    entropyVelocity[2] = entropyDrone3[0]


    # Assigning to an array for overall Entropy
    entropyOverall[0] = entropyDrone1[2]
    entropyOverall[1] = entropyDrone2[2]
    entropyOverall[2] = entropyDrone3[2]



  



   
    if(iteration % 5 == 0):
        with open('entropySave.csv', 'a') as csvfile:
            fileWrite = csv.writer(csvfile, delimiter=' ', quotechar='|')
            fileWrite.writerow([qxy[0],qxy[1],qxy[2],qxy[3],qxy[4],qxy[5],entropyDrone1[2],entropyDrone2[2],entropyDrone3[2],timeDiff,entropyDrone1[3],entropyDrone1[4],entropyDrone1[5],entropyDrone2[3],entropyDrone2[4],entropyDrone2[5],entropyDrone3[3],entropyDrone3[4],entropyDrone3[5],entropyDrone1[1],entropyDrone2[1],entropyDrone3[1]])




    # # From navigate.py for obstacle detection
    # result1 = client.simGetImage("0", airsim.ImageType.DepthVis, vehicle_name="Drone1")
    # result2 = client.simGetImage("0", airsim.ImageType.DepthVis, vehicle_name="Drone2")
    # result3 = client.simGetImage("0", airsim.ImageType.DepthVis, vehicle_name="Drone3")



    #  # Only returns [current,minimum]
    # if(iteration % 5 == 0):
    #     imageInfo1 = ImageReturn(result1)
    #     imageInfo2 = ImageReturn(result2)
    #     imageInfo3 = ImageReturn(result3)




    #     # Obstacle Avoidance Control
      
    #     if (imageInfo1[0] > 240):
    #         if (imageInfo1[1] == 0):
    #                 change = -2 * np.pi / 10
    #         elif (imageInfo1[1] == 1):
    #             change = -np.pi / 10
    #         elif (imageInfo1[1] == 2):
    #             change = 0 # center strip, go straight
    #         elif (imageInfo1[1] == 3):
    #             change = np.pi / 10
    #         else:
    #             change = 2*np.pi/10
           

    #         entropyDrone1[1] = entropyDrone1[1] +change

    #     elif (imageInfo2[0] >240 ):
    #         if (imageInfo2[1] == 0):
    #                 change = -2 * np.pi / 10
    #         elif (imageInfo2[1] == 1):
    #             change = -pi / 10
    #         elif (imageInfo2[1] == 2):
    #             change = 0 # center strip, go straight
    #         elif (imageInfo2[1] == 3):
    #             change = np.pi / 10
    #         else:
    #             change = 2*np.pi/10
       
    #         entropyDrone2[1] = entropyDrone2[1] +change

    #     elif (imageInfo3[0] >240):
    #         if (imageInfo3[1] == 0):
    #                 change = -2 * np.pi / 10
    #         elif (imageInfo3[1] == 1):
    #             change = -np.pi / 10
    #         elif (imageInfo3[1] == 2):
    #             change = 0 # center strip, go straight
    #         elif (imageInfo3[1] == 3):
    #             change = np.pi / 10
    #         else:
    #             change = 2*np.pi/10
           
    #         entropyDrone3[1] = entropyDrone3[1] +change

    #     # Regular Entropy Control

    #     print("Current 1")
    #     print(imageInfo1[0])
    #     print("Current 2")
    #     print(imageInfo2[0])
    #     print("Current 3")
    #     print(imageInfo3[0])



   
    # Rounding all values
    # entropyDrone1[0] = round(entropyDrone1[0],4)
    # entropyDrone1[1] = round(entropyDrone1[1],4)
    # entropyDrone2[0] = round(entropyDrone2[0],4)
    # entropyDrone2[1] = round(entropyDrone2[1],4)

    # entropyDrone3[0] = round(entropyDrone3[0],4)
    # entropyDrone3[1] = round(entropyDrone3[1],4)

    for i in range(2):
    	entropyDrone1[i] = round(entropyDrone1[i],4)
    	entropyDrone2[i] = round(entropyDrone2[i],4)
    	entropyDrone3[i] = round(entropyDrone3[i],4)

 





    # Necessary due to .join() and 0.5 leads to a smooth transition between phases, tried 0.1 but that is too small of an increment and 1.0 leads to  spinning in some cases
  
 

    for i in range(numUAV):
    	if(iteration > 1):
    		desRad[i] = posUAVs[i,2] + entropyDroneAngle[i]*0.5
    	else:
    		desRad[i] = posUAVs[i,2]
    	# Conversion from -pi to pi to 0 to 2 pi
    	desRad[i] = np.where(desRad[i]<0,2*np.pi+desRad[i], desRad[i])
    	# Necessary because multiples of pi will result in sim rotating multiple times
    	if(desRad[i] > 2 *np.pi):
        	desRad[i] = desRad[i]% (2*np.pi)

        vDes[2*i:2*i+1] =  np.array([entropyVelocity[i] * np.cos(desRad[i]),entropyVelocity[i]* np.sin(desRad[i])])

    # if(iteration > 1):
    #     desRad1 = posUAV1[2]+entropyDrone1[1]*0.5
    #     desRad2 = posUAV2[2]+entropyDrone2[1]*0.5
    #     desRad3 = posUAV3[2]+entropyDrone3[1]*0.5
    # else:
    #     desRad1 = posUAV1[2]
    #     desRad2 = posUAV2[2]
    #     desRad3 = posUAV3[2]




# Conversion from -pi to pi to 0 to 2 pi

    # desRad1 = np.where(desRad1<0 , 2*np.pi+desRad1, desRad1)
    # desRad2 = np.where(desRad2<0 , 2*np.pi+desRad2, desRad2)
    # desRad3 = np.where(desRad3<0 , 2*np.pi+desRad3, desRad3)


    # # Necessary because multiples of pi will result in sim rotating multiple times
    # if(desRad1 > 2 *np.pi):
    #     desRad1 = desRad1% (2*np.pi)
    # if(desRad2 > 2 *np.pi):
    #     desRad2 = desRad2% (2*np.pi)
    # if(desRad3 > 2 *np.pi):
    #     desRad3 = desRad3% (2*np.pi)
  

  


    # v1Des = np.array([entropyDrone1[0] * np.cos(desRad1),entropyDrone1[0] * np.sin(desRad1)])
    # v2Des = np.array([entropyDrone2[0] * np.cos(desRad2),entropyDrone2[0] * np.sin(desRad2)])
    # v3Des = np.array([entropyDrone3[0] * np.cos(desRad3),entropyDrone3[0] * np.sin(desRad3)])
  

    # v1Des[0] = round(v1Des[0],4)
    # v1Des[1] = round(v1Des[1],4)
    # v2Des[0] = round(v2Des[0],4)
    # v2Des[1] = round(v2Des[1],4)
    # v3Des[0] = round(v3Des[0],4)
    # v3Des[1] = round(v3Des[1],4)

    for i in range(2*numUAV):
    	 vDes[i] = round(vDes[i],4)
    	


    # Summation of |Vx and Vy| for cost function and f2 for distance cost function


    # f1UAV1 = np.sqrt((v1Des[0]**2) + (v1Des[1]**2))  +f1UAV1
    # f1UAV2 = np.sqrt((v2Des[0]**2) + (v2Des[1]**2)) +f1UAV2
    # f1UAV3 = np.sqrt((v3Des[0]**2) + (v3Des[1]**2)) + f1UAV3

    for i in range(numUAV):
    	f1[i] = np.sqrt((vDes[i]**2) + (vDes[i+1]**2))  +f1[i]
    	f2[i] = np.sqrt((posUAV[i,i]-pos0[i,0])**2 +(posUAV[i,i+1]-pos0[i,1])**2) +f2[i]

    # Change in X plus Change in Y for UAV1,2 & 3
    # f2UAV1 = np.sqrt((posUAV1[0]-pos0[0,0])**2 +(posUAV1[1]-pos0[0,1])**2) +f2UAV1
    # f2UAV2 = np.sqrt((posUAV2[0]-pos0[1,0])**2+(posUAV2[1]-pos0[1,1])**2) + f2UAV2
    # f2UAV3 = np.sqrt((posUAV3[0]-pos0[2,0])**2+(posUAV3[1]-pos0[2,1])**2)+ f2UAV3

  
 
    # Note:  def moveByVelocityAsync(self, vx, vy, vz, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), vehicle_name = ''):
    #         """
    #         Args:
    #             vx (float): desired velocity in world (NED) X axis
    #             vy (float): desired velocity in world (NED) Y axis
    #             vz (float): desired velocity in world (NED) Z axis
    #             duration (float): Desired amount of time (seconds), to send this command for
    #             drivetrain (DrivetrainType, optional):
    #             yaw_mode (YawMode, optional):
    #             vehicle_name (str, optional): Name of the multirotor to send this command to
    # #Note: Negative Axis is up

   
    # client.simPause(False)
    for i in range(numUAV):
    	name = "Drone" + str(i+1)
	    f = client.moveByVelocityZAsync(vDes[i],vDes[i+1], z,commandTime,airsim.DrivetrainType.MaxDegreeOfFreedom,airsim.YawMode(False,np.degrees(desRad[i])), vehicle_name=name)
	

	    # Not sure this bit will work as intended

	    f.join()
	    





# f1 = client.moveByVelocityZAsync(v1Des[0],v1Des[1], z,commandTime,airsim.DrivetrainType.MaxDegreeOfFreedom,airsim.YawMode(False,np.degrees(desRad1)), vehicle_name="Drone1")
# 	    f2 = client.moveByVelocityZAsync(v2Des[0],v2Des[1], z,commandTime, airsim.DrivetrainType.MaxDegreeOfFreedom,airsim.YawMode(False,np.degrees(desRad2)),vehicle_name="Drone2")
# 	    f3 = client.moveByVelocityZAsync(v3Des[0],v3Des[1], z,commandTime, airsim.DrivetrainType.MaxDegreeOfFreedom,airsim.YawMode(False,np.degrees(desRad3)),vehicle_name="Drone3")



# 	    f1.join()
# 	    f2.join()
# 	    f3.join()


csvfile.close()
# with open('ParameterAnalysisResults.csv', 'a') as csvfileparams:
#         fileWriteparam = csv.writer(csvfileparams, delimiter=' ', quotechar='|')
#         fileWriteparam.writerow([f1UAV1,f1UAV2,f1UAV3,f2UAV1,f2UAV2,f2UAV3])

with open('ParameterAnalysisResults.csv', 'a') as csvfileparams:
        fileWriteparam = csv.writer(csvfileparams, delimiter=' ', quotechar='|')
        fileWriteparam.writerow([f1[0],f1[1],f1[2],f2[0],f2[1],f2[2]])
       


client.armDisarm(False, "Drone1")
client.armDisarm(False, "Drone2")
client.armDisarm(False,"Drone3")
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, "Drone1")
client.enableApiControl(False, "Drone2")
client.enableApiControl(False,"Drone3")


