# import setup_path 
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
from EntropyCalculationSixUAVs import calEntropy



# Current Settings Json
# {
#   "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
#   "SettingsVersion": 1.2,
#   "RpcEnabled": true,
#   "SimMode": "Multirotor",
# "Vehicles": {
"""
"Drone1": {
      "VehicleType": "SimpleFlight",
      "X": 150,
      "Y": 25,
      "Z": -2,
      "Yaw":90,
      "AllowAPIAlways": true
    }
    "Drone2": {
      "VehicleType": "SimpleFlight",
      "X": 100,
      "Y": 25,
      "Z": -2,
      "Yaw":90,
      "AllowAPIAlways": true
    },
    ,
    "Drone3": {
      "VehicleType": "SimpleFlight",
      "X": 50,
      "Y": 25,
      "Z": -2,
      "Yaw": 90,
      "AllowAPIAlways": true
    },
    "Drone4": {
      "VehicleType": "SimpleFlight",
      "X": 0,
      "Y": 25,
      "Z": -2,
      "Yaw": 90,
      "AllowAPIAlways": true
    }
"Drone5": {
      "VehicleType": "SimpleFlight",
      "X": -50,
      "Y": 25,
      "Z": -2,
      "Yaw": 90,
      "AllowAPIAlways": true
    }
"Drone6": {
      "VehicleType": "SimpleFlight",
      "X": -100,
      "Y": 25,
      "Z": -2,
      "Yaw": 90,
      "AllowAPIAlways": true
    }

  }
}
"""
"""
Current Drone Orientation is


UAV6         UAV5          UAV4       UAV3          UAV2               UAV1
"""
#Initial Positions on Drones must match settings file
numUAV = 6

# Cost Function Parameters
# Total Distance Traveled 
# f2UAV1 = 0
# f2UAV2 = 0
# f2UAV3 = 0
#UAV1, UAV2, UAV3
# f2 = ([0,0,0])
# Sum of Vx and Vy, so control inputs
# f1UAV1 = 0
# f1UAV2 = 0
# f1UAV3 = 0

# f1 = ([0,0,0])
f1 = np.zeros(numUAV)
f2 = np.zeros(numUAV)


pos0 = np.zeros((numUAV,3))
posUAVs = np.zeros((numUAV,3))



q = np.zeros(3*numUAV) #Preallocate state vectors
qo = np.zeros(4*numUAV) #Preallocate orientation vectors
qxy = np.zeros(2*numUAV)     #Preallocate position vectors

a = np.zeros(numUAV)
b = np.zeros(numUAV)

entropyDroneAngle = np.zeros(numUAV)
entropyVelocity = np.zeros(numUAV)
entropyOverall = np.zeros(numUAV)
desRad = np.zeros(numUAV)
vDes = np.zeros((numUAV,2))
posUAVPrev = posUAVs



# For Line Starting position assignment
for i in range(numUAV):
	pos0[i,0] =  150 -50*i
	pos0[i,1] =  25.0
	pos0[i,2] = -2.0




Vmax = 0.5 
# Vmax = 0.75
# Vmax = 1
# Vmax = 5
flag = 0
# Waypoint Array
# Note: (0,500) is the test point. The rest of the points are the waypoints for the corners of a square
x_waypoint = np.array([0, -50, -50, 50, 50, -50])
y_waypoint = np.array([100, 50, 200, 200, 50, 50])


a[0] = x_waypoint[0]
a[1] = a[0] + 12
for i in range(numUAV-2):
	a[i+2] = a[0] -12*(i+1)
	

# b = y_waypoint[0]
# b2 = b
# b3 = b

for i in range(numUAV):
	b[i] = y_waypoint[0]


endLoop = False
commandTime = 0.1
z = -2



with open('entropySave.csv', 'w') as csvfile:
    fileWrite = csv.writer(csvfile, delimiter=' ', quotechar='|')
    fileWrite.writerow(['X1','Y1','X2','Y2','X3','Y3','X4','Y4','X5','Y5','X6','Y6','EntropyDrone1','EntropyDrone2','EntropyDrone3','EntropyDrone4','EntropyDrone5','EntropyDrone6','TimeDifference','EntropyDrone1D1','EntropyDrone1D2','EntropyDrone1D3','EntropyDrone2D1','EntropyDrone2D2','EntropyDrone2D3','EntropyDrone3D1','EntropyDrone3D2','EntropyDrone3D3','Angle1','Angle2','Angle3'])

    
count = 0



# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True,"Drone1")
client.armDisarm(True,"Drone1")
client.enableApiControl(True,"Drone2")
client.armDisarm(True,"Drone2")
client.enableApiControl(True,"Drone3")
client.armDisarm(True,"Drone3")
client.enableApiControl(True,"Drone4")
client.armDisarm(True,"Drone4")
client.enableApiControl(True,"Drone5")
client.armDisarm(True,"Drone5")
client.enableApiControl(True,"Drone6")
client.armDisarm(True,"Drone6")









iteration = 0
beginningTime = client.getMultirotorState(vehicle_name = 'Drone1').timestamp

   
# Change back to endLoop, only using iteration limit for preliminary testing
# while (not endLoop):
while (not endLoop):
    iteration = iteration + 1
    if iteration ==1:
        print("Starting Formation Control")
    elif (iteration % 10 == 0):
        print("Iteration = ",iteration)
    
    
    #Get Drone positions for entropy formation
        
  # For Single Waypoint
    distWaypoint = 20
    # distWaypoint = 40
    


    currentTime = client.getMultirotorState(vehicle_name = 'Drone1').timestamp 
    timeDiff = (currentTime-beginningTime)/1000000000
    
    for i in range(numUAV):
        name = "Drone" + str(i+1)
	  
	    # Get x-y-z coordinates
        pos = client.simGetGroundTruthKinematics(vehicle_name = name)
        qi = np.array([pos.position.x_val, pos.position.y_val, pos.position.z_val])
        qoi = np.array([pos.orientation.w_val, pos.orientation.x_val, pos.orientation.y_val, pos.orientation.z_val])
	  
	    # Add initial coordinates
        qd = qi + pos0[i,:]
        

	    # 3D and 2D state vector

        # qxy[2*i:2*i+2] = np.array([qd[i,0], qd[i,1]])
        qo[4*i:4*i+4] =  qoi.copy()
        qua = Quaternion(qo[4*i], qo[4*i+1], qo[4*i+2], qo[4*i+3])
        euler = qua.to_euler(degrees = False)
        # posUAVs[4*i:4*i+4] = np.array(qd[0], qd[1],euler[2])
        # posUAVs = np.array(qd[0], qd[1],euler[2])
        posUAVs[i,0] = qd[0]
        posUAVs[i,1] = qd[1]
        posUAVs[i,2] = euler[2]
     

 

    




    posUAV1 = posUAVs[0,:]
    posUAV2 = posUAVs[1,:]
    posUAV3 = posUAVs[2,:]
    posUAV4 = posUAVs[3,:]
    posUAV5 = posUAVs[4,:]
    posUAV6 = posUAVs[5,:]

	
    
    
    
    arraytuple1 = [posUAV1,posUAV2,posUAV3,posUAV4,posUAV5,posUAV6]
    arrayspos1 = np.vstack(arraytuple1)

    arraytuple2 = [posUAV2,posUAV3,posUAV4,posUAV5,posUAV6,posUAV1]
    # arraytuple2 = [posUAV2,posUAV3,posUAV1,posUAV4,posUAV5,posUAV6]
    arrayspos2 = np.vstack(arraytuple2)

    arraytuple3 = [posUAV3,posUAV4,posUAV2,posUAV6,posUAV5,posUAV1]
    # arraytuple3 = [posUAV3,posUAV4,posUAV5,posUAV6,posUAV1,posUAV2]
    arrayspos3 = np.vstack(arraytuple3)

    arraytuple4 = [posUAV4,posUAV5,posUAV3,posUAV1,posUAV2,posUAV3]
    # arraytuple4 = [posUAV4,posUAV5,posUAV6,posUAV1,posUAV2,posUAV3]
    arrayspos4 = np.vstack(arraytuple4)

    arraytuple5= [posUAV5,posUAV6,posUAV4,posUAV3,posUAV2,posUAV1]
    # arraytuple5= [posUAV5,posUAV6,posUAV1,posUAV2,posUAV3,posUAV4]
    arrayspos5 = np.vstack(arraytuple5)

    # arraytuple6 = [posUAV6,posUAV1,posUAV2,posUAV3,posUAV4,posUAV5]
    arraytuple6 = [posUAV6,posUAV5,posUAV4,posUAV3,posUAV2,posUAV1]
    arrayspos6 = np.vstack(arraytuple6)


  
    entropyDrone1 = calEntropy(arrayspos1,a[1],b[1],Vmax,flag,numUAV)
    entropyDrone2 = calEntropy(arrayspos2,a[0],b[0],Vmax,flag,numUAV)
    entropyDrone3 = calEntropy(arrayspos3,a[2],b[2],Vmax,flag,numUAV)
    entropyDrone4 = calEntropy(arrayspos4,a[3],b[3],Vmax,flag,numUAV)
    entropyDrone5 = calEntropy(arrayspos5,a[4],b[4],Vmax,flag,numUAV)
    entropyDrone6 = calEntropy(arrayspos6,a[5],b[5],Vmax,flag,numUAV)
    # Assigning to an array for angle

    entropyDroneAngle[0] = entropyDrone1[1] 
    entropyDroneAngle[1] = entropyDrone2[1] 
    entropyDroneAngle[2] = entropyDrone3[1] 
    entropyDroneAngle[3] = entropyDrone4[1] 
    entropyDroneAngle[4] = entropyDrone5[1] 
    entropyDroneAngle[5] = entropyDrone6[1] 

    # Assigning to an array for velocity
    entropyVelocity[0] = entropyDrone1[0] 
    entropyVelocity[1] = entropyDrone2[0] 
    entropyVelocity[2] = entropyDrone3[0]
    entropyVelocity[3] = entropyDrone4[0] 
    entropyVelocity[4] = entropyDrone5[0] 
    entropyVelocity[5] = entropyDrone6[0]

    # Assigning to an array for overall Entropy
    entropyOverall[0] = entropyDrone1[2]
    entropyOverall[1] = entropyDrone2[2]
    entropyOverall[2] = entropyDrone3[2]
    entropyOverall[3] = entropyDrone4[2]
    entropyOverall[4] = entropyDrone5[2]
    entropyOverall[5] = entropyDrone6[2]


  



   
    if(iteration % 5 == 0):
        with open('entropySave.csv', 'a') as csvfile:
            fileWrite = csv.writer(csvfile, delimiter=' ', quotechar='|')
            fileWrite.writerow([posUAVs[0,0],posUAVs[0,1],posUAVs[1,0],posUAVs[1,1],posUAVs[2,0],posUAVs[2,1],posUAVs[3,0],posUAVs[3,1],posUAVs[4,0],posUAVs[4,1],posUAVs[5,0],posUAVs[5,1],entropyDrone1[2],entropyDrone2[2],entropyDrone3[2],entropyDrone4[2],entropyDrone5[2],entropyDrone6[2],timeDiff])




  



   

    for i in range(2):
    	entropyDrone1[i] = round(entropyDrone1[i],4)
    	entropyDrone2[i] = round(entropyDrone2[i],4)
    	entropyDrone3[i] = round(entropyDrone3[i],4)
    	entropyDrone4[i] = round(entropyDrone4[i],4)
    	entropyDrone5[i] = round(entropyDrone5[i],4)
    	entropyDrone6[i] = round(entropyDrone6[i],4)

 





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

        
        vDes[i,0] =  entropyVelocity[i] * np.cos(desRad[i])
        vDes[i,1] =  entropyVelocity[i]* np.sin(desRad[i])






  


 

    for i in range(numUAV):
        for j in range(2):
            vDes[i,j] = round(vDes[i,j],4)
    	


    # for i in range(numUAV):
    # 	f1[i] = np.sqrt((vDes[i,0]**2) + (vDes[i,1]**2))  +f1[i]
    	

    # 	if(iteration == 1):
    # 		f2[i] = np.sqrt((posUAVs[i,0]-pos0[i,0])**2 +(posUAVs[i,1]-pos0[i,1])**2) +f2[i]
    # 	else:
    # 		f2[i] = np.sqrt((posUAVs[i,0]-posUAVPrev[i,i])**2 +(posUAVs[i,1]-posUAVPrev[i,i+1])**2) +f2[i]
    # for i in range(numUAV):
    # 	for j in range(2):
    # 		posUAVPrev[i,j] = posUAVs[i,j]

  
 
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

 
  
    f1 = client.moveByVelocityZAsync(vDes[0,0],vDes[0,1], z,commandTime,airsim.DrivetrainType.MaxDegreeOfFreedom,airsim.YawMode(False,np.degrees(desRad[0])), vehicle_name="Drone1")
    f2 = client.moveByVelocityZAsync(vDes[1,0],vDes[1,1], z,commandTime,airsim.DrivetrainType.MaxDegreeOfFreedom,airsim.YawMode(False,np.degrees(desRad[1])), vehicle_name="Drone2")
    f3 = client.moveByVelocityZAsync(vDes[2,0],vDes[2,1], z,commandTime,airsim.DrivetrainType.MaxDegreeOfFreedom,airsim.YawMode(False,np.degrees(desRad[2])), vehicle_name="Drone3")
    f4 = client.moveByVelocityZAsync(vDes[3,0],vDes[3,1], z,commandTime,airsim.DrivetrainType.MaxDegreeOfFreedom,airsim.YawMode(False,np.degrees(desRad[3])), vehicle_name="Drone4")
    f5 = client.moveByVelocityZAsync(vDes[4,0],vDes[4,1], z,commandTime,airsim.DrivetrainType.MaxDegreeOfFreedom,airsim.YawMode(False,np.degrees(desRad[4])), vehicle_name="Drone5")
    f6 = client.moveByVelocityZAsync(vDes[5,0],vDes[5,1], z,commandTime,airsim.DrivetrainType.MaxDegreeOfFreedom,airsim.YawMode(False,np.degrees(desRad[5])), vehicle_name="Drone6")
    f1.join()
    f2.join()
    f3.join()
    f4.join()
    f5.join()
    f6.join()
  
	   # Ends Loop
    if ((np.sqrt(((posUAV2[0] - a[0] )**2) + ((posUAV2[1] - b[0] )**2 ))< distWaypoint) and (np.sqrt(((posUAV1[0] - a[1] )**2) + ((posUAV1[1] - b[1])**2 ))< distWaypoint) and (np.sqrt(((posUAV3[0] - a[2] )**2) + ((posUAV3[1] - b[2] )**2 ))< distWaypoint) and (np.sqrt(((posUAV4[0] - a[3] )**2) + ((posUAV4[1] - b[3] )**2 ))< distWaypoint) and (np.sqrt(((posUAV5[0] - a[4] )**2) + ((posUAV5[1] - b[4] )**2 ))< distWaypoint) and (np.sqrt(((posUAV6[0] - a[5] )**2) + ((posUAV6[1] - b[5] )**2 ))< distWaypoint)): 
        endLoop = True

    







csvfile.close()

# with open('ParameterAnalysisResults.csv', 'a') as csvfileparams:
#         fileWriteparam = csv.writer(csvfileparams, delimiter=' ', quotechar='|')
#         fileWriteparam.writerow([f1[0],f1[1],f1[2],f2[0],f2[1],f2[2]])
       


client.armDisarm(False, "Drone1")
client.armDisarm(False, "Drone2")
client.armDisarm(False,"Drone3")
client.armDisarm(False, "Drone4")
client.armDisarm(False, "Drone5")
client.armDisarm(False,"Drone6")
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, "Drone1")
client.enableApiControl(False, "Drone2")
client.enableApiControl(False,"Drone3")
client.enableApiControl(False, "Drone4")
client.enableApiControl(False, "Drone5")
client.enableApiControl(False,"Drone6")

import PlotterSixUAVs