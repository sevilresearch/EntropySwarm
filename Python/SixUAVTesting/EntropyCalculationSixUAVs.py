import numpy as np
import cv2

def calEntropy (u,a,b,Vmax,flag,uavCount):
    x = np.zeros(uavCount,dtype= float)
    y = np.zeros(uavCount,dtype = float)
    t = np.zeros(uavCount,dtype = float)
    d = np.zeros(uavCount,dtype = float)
    Th = np.zeros(uavCount,dtype = float)
    V_com = 0.0
    theta_com = 0.0
    summation = 0
    e = np.zeros(uavCount,dtype = float)
 
    

    # maxDistance = 100#Maximum Desired Distance between UAVs
    maxDistance = 200#Maximum Desired Distance between UAVs
  
    # minimumDistance  = 18
    minimumDistance  = 12
    # minimumDistance  = 20


    for i in range(uavCount):
    	x[i] = u[i,0]
    	y[i] = u[i,1]
    	t[i] = u[i,2]

   

    # d[0] = np.sqrt(((x[1]-x[5])**2)+((y[1]-y[5])**2))
    d[0] = np.sqrt(((x[1]-x[2])**2)+((y[1]-y[2])**2))
    for i in range(1,uavCount):
        d[i] = np.sqrt(((x[i]-x[0])**2)+((y[i]-y[0])**2))
        Th[i] = np.arctan2( (y[i]-y[0]), (x[i]-x[0]) )
  

    x_cur = x[0]
    y_cur = y[0]
    theta_cur = t[0]
 
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

    



    for i in range(uavCount):
       
        if d[i] >=maxDistance:
            e[i]=maxDistance
        elif (d[i] < maxDistance and d[i] > minimumDistance):
            e[i]=d[i]
        elif d[i] <= minimumDistance:
            e[i]= minimumDistance
    

  

    # Highest case 4
    # Lowest Case -0.1026
    # Threshold = 3
    Threshold = 4
    # Threshold = 2
    # Threshold = 6.5 #Goes Directly to Waypoint
 
    # Original
    # q= 0.9
    q= 0.5
    # q= 0.75
    # q= 0.6


    summation = ((e[0]/maxDistance)**q)+ ((e[1]/maxDistance)**q)+  ((e[2]/maxDistance)**q)+ ((e[3]/maxDistance)**q)+((e[4]/maxDistance)**q)+((e[5]/maxDistance)**q)
    # Groups into two groups of 3
    # summation = ((e[1]/maxDistance)**q)+  ((e[2]/maxDistance)**q)+ ((e[3]/maxDistance)**q)+((e[4]/maxDistance)**q)+((e[5]/maxDistance)**q)
 
 

    # St = ((1-summation)/(1-q))
    St = ((1-summation)/(q-1))#7/1/20
    
    
    xd = (a-x_cur)
    yd = (b-y_cur)

 
    d_left = np.sqrt((xd**2)+(yd**2))
 
    theta_targ = np.arctan2(yd,xd)
 

    # Currently Waypoint Algorithm
    
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
        
            V_com=Vmax# * (d(index)/100)
    
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
   
        V_com=2 * Vmax# * (d(index2)/100);
    
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
        V_com=1/2 * Vmax# * (d(index)/100);
     #St=St_temp;
   
    return [V_com, theta_com,St]



