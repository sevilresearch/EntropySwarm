# -*- coding: utf-8 -*-
"""
Created on Thu May 28 16:27:51 2020

@author: lukef
"""

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation
# import time


x1 = []
y1 = []
x2 =[]
y2 =[]
x3 =[]
y3 =[]




plt.style.use('fivethirtyeight')
# ax = plt.axes()

# ax.set(xlim=(-110, 160), ylim=(-5, 150))
fig = plt.figure(figsize = (15,10))
# ax1 = fig.add_subplot(3, 1, 1)
# ax2 = fig.add_subplot(3, 1, 2)
# ax3 = fig.add_subplot(3, 1, 3)


ax1 = fig.add_subplot(2, 1, 1)
ax2 = fig.add_subplot(2, 1, 2)
def animate(i):
        
        data = pd.read_csv('entropySave.csv',delimiter = " ")

        # Live Data Setting
        x1 = data['X1']
        y1 = data['Y1']
        x2 = data['X2']
        y2 = data['Y2']
        x3 = data['X3']
        y3 = data['Y3']
        e1 = data['EntropyDrone1']
        e2 = data['EntropyDrone2']
        e3 = data['EntropyDrone3']
        time = data['TimeDifference']
        ax1.cla()
    
        ax1.plot(x1, y1,linestyle='-',color = 'c')
        ax1.plot(x2, y2,linestyle='-',color = 'r')
        ax1.plot(x3, y3,linestyle='-',color = '#fcbe03')
        




         # Live Data Setting
    
    
        ax2.cla()
    
        plt.plot(time, e1, linestyle='-',color = 'c')
        plt.plot(time, e2, linestyle='--',color = 'r')
        plt.plot(time, e3, linestyle='--',color = '#fcbe03')
        
        ax1.set(xlim=([-110, 160]), ylim=([20, 200]))
        ax1.legend(["UAV 1 ","UAV 2","UAV 3"])
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.set_title('Positions')

        ax2.legend(["Entropy 1 ","Entropy 2","Entropy 3"])
        ax2.set_xlabel("Time(s)")
        ax2.set_ylabel("Entropy")
        ax2.set_title('Entropy for each UAV')
        # time.sleep(5) # keep refresh rate of 5 seconds 
        plt.tight_layout()

def animateEntropy(i):
        
        data = pd.read_csv('entropySave.csv',delimiter = " ")

        # Live Data Setting
        e1 = data['EntropyDrone1']
        e2 = data['EntropyDrone2']
        e3 = data['EntropyDrone3']
        time = data['TimeDifference']
    
        plt.cla()
    
        plt.plot(time, e1, linestyle='-',color = 'c')
        plt.plot(time, e2, linestyle='--',color = 'r')
        plt.plot(time, e3, linestyle='--',color = '#fcbe03')
        
        ax2.legend(["Entropy 1 ","Entropy 2","Entropy 3"])
        ax2.set_xlabel("Time")
        ax2.set_ylabel("Entropy")
        plt.title('Entropy for each UAV')
        # time.sleep(5) # keep refresh rate of 5 seconds 
        plt.tight_layout()

def animateDistances(i):
        data = pd.read_csv('entropySave.csv',delimiter = " ")
  

        drone1Distance1 = data['EntropyDrone1D1']
        drone1Distance2 = data['EntropyDrone1D2']
        drone1Distance3 = data['EntropyDrone1D3']
        drone2Distance1 = data['EntropyDrone2D1']
        drone2Distance2 = data['EntropyDrone2D2']
        drone2Distance3 = data['EntropyDrone2D3']
        drone3Distance1 = data['EntropyDrone3D1']
        drone3Distance2 = data['EntropyDrone3D2']
        drone3Distance3 = data['EntropyDrone3D3']
        time = data['TimeDifference']

        plt.cla()

        ax1.cla()
        ax1.plot(time, drone1Distance1, linestyle='-')
        ax1.plot(time, drone1Distance2,linestyle='-')
        ax1.plot(time, drone1Distance3,linestyle='-')

        ax2.cla()
        ax2.plot(time, drone2Distance1, linestyle='-')
        ax2.plot(time, drone2Distance2,linestyle='-')
        ax2.plot(time, drone2Distance3,linestyle='-')

        ax3.cla()
        ax3.plot(time, drone3Distance1, linestyle='-')
        ax3.plot(time, drone3Distance2,linestyle='-')
        ax3.plot(time, drone3Distance3,linestyle='-')
     

        ax1.legend(["Distance 1 ","Distance 2","Distance 3"])
        ax1.set_xlabel("Time")
        ax1.set_ylabel("Distances")
        ax1.set_title('Distances Calculated from Drone 1')

        ax2.legend(["Distance 1 ","Distance 2","Distance 3"])
        ax2.set_xlabel("Time")
        ax2.set_ylabel("Distances")
        ax2.set_title('Distances Calculated from Drone 2')

        ax3.legend(["Distance 1 ","Distance 2","Distance 3"])
        ax3.set_xlabel("Time")
        ax3.set_ylabel("Distances")
        ax3.set_title('Distances Calculated from Drone 3')

# plt.figure(1)    
ani = FuncAnimation(plt.gcf(),animate,interval = 1000)
# plt.tight_layout()
# plt.show()
# plt.figure(2)
# ax2 = plt.axes()
# # plt.autoscale(enable = True, axis = 'both')
# aniEntropy = FuncAnimation(plt.gcf(),animateEntropy,interval = 1000)


# plt.autoscale(enable = True, axis = 'both')
# aniEntropy = FuncAnimation(plt.gcf(),animateDistances,interval = 1000)
plt.tight_layout()
plt.show()