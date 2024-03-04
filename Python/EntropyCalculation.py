# -*- coding: utf-8 -*-
"""
Created on Fri May 22 13:14:09 2020

@author: lukef
"""
import numpy as np
import cv2


# Derived from Dr. Hakki Erhan Sevil Entropy Matlab Code
# function [V_com,theta_com,St] = fcn(u,u2,u3,a,b,Vmax,flag)
def calEntropy (u,u2,u3,a,b,Vmax,flag):
    x = np.zeros(3,dtype= float)
    y = np.zeros(3,dtype = float)
    t = np.zeros(3,dtype = float)
    d = np.zeros(3,dtype = float)
    Th = np.zeros(3,dtype = float)
    V_com = 0.0
    theta_com = 0.0
    e = np.zeros(3,dtype = float)
    # minimumDistance = 10#Minimum Distance between UAVs
    

    maxDistance = 100#Maximum Desired Distance between UAVs
    minimumDistance  = 12
    # minimumDistance  = 24
    # minimumDistance  = 30
    # maxDistance = 250
    

    x_cur = u[0]
    y_cur = u[1]
    theta_cur = u[2]
    x[1] = u2[0]
    y[1] = u2[1]
    t[1] = u2[2]
    
    x[2] = u3[0]
    y[2] = u3[1]
    t[2] = u3[2]
    
    # Distance Calculations
    d[0] = np.sqrt(((x[1]-x[2])**2)+((y[1]-y[2])**2))
   
    d[1]=np.sqrt(((x[1]-x_cur)**2)+((y[1]-y_cur)**2))
    # print(d[1])
    Th[1]=np.arctan2( (y[1]-y_cur), (x[1]-x_cur) )
   

    d[2]=np.sqrt(((x[2]-x_cur)**2)+((y[2]-y_cur)**2))
    Th[2]=np.arctan2((y[2]-y_cur), (x[2]-x_cur))


 
    if d[1] < d[2]:
        minimum=d[1]
        maximum=d[2]
        index=1
        index2=2
    else:
        minimum=d[2]
        maximum=d[1]
        index=2
        index2=1
    
    
    for i in range(3):
       
        if d[i] >=maxDistance:
            e[i]=maxDistance
        elif (d[i] < maxDistance and d[i] > minimumDistance):
            e[i]=d[i]
        elif d[i] <= minimumDistance:
            e[i]= minimumDistance

        # Old
    #for i in range(3):
       
        # if (maxDistance-d[i])<0:
        #     e[i]=0
        # elif(maxDistance-d[i])>minallowable:
        #     e[i]=minallowable
        # else:
        #     e[i]=maxDistance-d[i]


 

    # Highest case 4
    # Lowest Case -0.1026
    # Threshold = 0.5
    Threshold = 0.8
    # Threshold = 1.0
    # Threshold = 1.5
    # Threshold = 3

    
    # q= 0.9
    # Original
    q= 0.5
    # q= 0.75
    # q= 0.6
    # summation = ( (e[1]/100)**q ) + ( (e[2]/100)**q )
    summation= ( (e[0]/maxDistance)**q ) + ( (e[1]/maxDistance)**q ) + ( (e[2]/maxDistance)**q )

    # St = ((1-summation)/(1-q))
    St = ((1-summation)/(q-1))#7/1/20
    St_temp = St
    
    xd = (a-x_cur)
    yd = (b-y_cur)
 
    d_left = np.sqrt((xd**2)+(yd**2))
 
    theta_targ = np.arctan2(yd,xd)
 
    # # if flag==1:
    # #     St=St+0.2
    # #     # minimum=11;

    # # Mutiple Waypoints
    if St<Threshold:
        #If within entropy threshold and further than minimum distance, then move toward waypoint
        if minimum>minimumDistance:
            theta_com=theta_targ-theta_cur
            # theta_com=theta_targ-theta_cur/2
            if theta_com>np.pi:
                theta_com=(-2*np.pi+theta_com)
            elif theta_com<-np.pi:
                theta_com=(2*np.pi+theta_com)
            if d_left>100:
                d_left=100
        
            V_com= Vmax# * (d_left/100)
        #If within entropy threshold and too close, move apart
        else:
            theta_com=(Th[index]-theta_cur) + np.pi
            # theta_com=((Th[index]-theta_cur/2) + np.pi)
            # theta_com=(((Th[index]+theta_targ)/2)-theta_cur) + np.pi
            if theta_com>np.pi:
                theta_com=-2*np.pi+theta_com + np.pi
            elif theta_com<-np.pi:
                theta_com=2*np.pi+theta_com + np.pi
            if d[index]>100:
                d[index]=100
        
            V_com= Vmax# * (d(index)/100)

    
    #If not in threshold and closer UAV isn't close enough, move toward closer UAV
    elif minimum>minimumDistance:
        theta_com=Th[index]-theta_cur
        # theta_com=Th[index]-theta_cur/2
  
        if theta_com>np.pi:
            theta_com=-2*np.pi+theta_com
        elif theta_com<-np.pi:
            theta_com=2*np.pi+theta_com
        if d[index]>100:
            d[index]=100
    
        V_com=2 * Vmax# * (d[index]/100)
    #If not in threshold and closer UAV is close enough and further UAV is not close enough, move toward further UAV
    elif maximum>minimumDistance:
        theta_com=Th[index2]-theta_cur
        # theta_com=Th[index2]-theta_cur/2
        if theta_com>np.pi:
            theta_com=-2*np.pi+theta_com
        elif theta_com<-np.pi:
            theta_com=2*np.pi+theta_com
        if d[index2]>100:
            d[index2]=100
   
        V_com=2* Vmax# * (d(index2)/100);
    
    #If not in threshold and all 3 UAVs are close enough but too close move apart
    elif (minimum<minimumDistance or maximum<minimumDistance):
        theta_com=(Th[index]-theta_cur) + np.pi
        # theta_com=(Th[index]-theta_cur/2) + np.pi
        if theta_com>np.pi:
            theta_com=-2*np.pi+theta_com + np.pi
        elif theta_com<-np.pi:
            theta_com=2*np.pi+theta_com + np.pi
        if d[index]>100:
            d[index]= 100
        V_com=1/2*Vmax# * (d(index)/100);
     #St=St_temp;
   
    return [V_com, theta_com,St,d[0],d[1],d[2]]


   # # # Original, Parameter Analysis
    # if St<Threshold:
    #     #If within entropy threshold and further than minimum distance, then move toward waypoint
    #     if minimum>minimumDistance:
    #         theta_com=theta_targ-theta_cur
    #         # theta_com=theta_targ-theta_cur/2
    #         if theta_com>np.pi:
    #             theta_com=(-2*np.pi+theta_com)
    #         elif theta_com<-np.pi:
    #             theta_com=(2*np.pi+theta_com)
    #         if d_left>100:
    #             d_left=100
        
    #         V_com= Vmax# * (d_left/100)
    #     #If within entropy threshold and too close, move apart
    #     else:
    #         theta_com=(Th[index]-theta_cur) + np.pi
    #         # theta_com=((Th[index]-theta_cur/2) + np.pi)
    #         # theta_com=(((Th[index]+theta_targ)/2)-theta_cur) + np.pi
    #         if theta_com>np.pi:
    #             theta_com=-2*np.pi+theta_com + np.pi
    #         elif theta_com<-np.pi:
    #             theta_com=2*np.pi+theta_com + np.pi
    #         if d[index]>100:
    #             d[index]=100
        
    #         V_com=1/2* Vmax# * (d(index)/100)
    
    # #If not in threshold and closer UAV isn't close enough, move toward closer UAV
    # elif minimum>minimumDistance:
    #     theta_com=Th[index]-theta_cur
    #     # theta_com=Th[index]-theta_cur/2
  
    #     if theta_com>np.pi:
    #         theta_com=-2*np.pi+theta_com
    #     elif theta_com<-np.pi:
    #         theta_com=2*np.pi+theta_com
    #     if d[index]>100:
    #         d[index]=100
    
    #     V_com=1/2 * Vmax# * (d[index]/100)
    # #If not in threshold and closer UAV is close enough and further UAV is not close enough, move toward further UAV
    # elif maximum>minimumDistance:
    #     theta_com=Th[index2]-theta_cur
    #     # theta_com=Th[index2]-theta_cur/2
    #     if theta_com>np.pi:
    #         theta_com=-2*np.pi+theta_com
    #     elif theta_com<-np.pi:
    #         theta_com=2*np.pi+theta_com
    #     if d[index2]>100:
    #         d[index2]=100
   
    #     V_com=1/2 * Vmax# * (d(index2)/100);
    
    # #If not in threshold and all 3 UAVs are close enough but too close move apart
    # elif (minimum<minimumDistance or maximum<minimumDistance):
    #     theta_com=(Th[index]-theta_cur) + np.pi
    #     # theta_com=(Th[index]-theta_cur/2) + np.pi
    #     if theta_com>np.pi:
    #         theta_com=-2*np.pi+theta_com + np.pi
    #     elif theta_com<-np.pi:
    #         theta_com=2*np.pi+theta_com + np.pi
    #     if d[index]>100:
    #         d[index]= 100
    #     V_com=2*Vmax# * (d(index)/100);
    #  #St=St_temp;
   
    # return [V_com, theta_com,St,d[0],d[1],d[2]]

    
def ImageReturn(result):

    rawImage = np.fromstring(result, np.int8)
    png = cv2.imdecode(rawImage, cv2.IMREAD_UNCHANGED)
    gray = cv2.cvtColor(png, cv2.COLOR_BGR2GRAY)

    # slice the image so we only check what we are headed into (and not what is down on the ground below us).

    top = np.vsplit(gray, 2)[0]

    # now look at 4 horizontal bands (far left, left, right, far right) and see which is most open.
    # the depth map uses black for far away (0) and white for very close (255), so we invert that
    # to get an estimate of distance.
    bands = np.hsplit(top, [50,100,150,200]);
    maxes = [np.max(x) for x in bands]
    minimum = np.argmin(maxes)    
    distance = 255 - maxes[minimum]

    # sanity check on what is directly in front of us (slot 2 in our hsplit)
    current = 255 - maxes[2]
    return [current,minimum]