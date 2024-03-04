# -*- coding: utf-8 -*-
"""
Created on Thu May 28 16:27:51 2020

@author: lukef
"""


import pandas as pd


import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter




drone1Distance1 = []
drone1Distance2 = []
drone1Distance3 = []
drone2Distance1 = []
drone2Distance2 = []
drone2Distance3 = []
drone3Distance1 = []
drone3Distance2 = []
drone3Distance3 = []
# e1 =[]
# e2 =[]
# e3 =[]
time =[]



plt.style.use('fivethirtyeight')
# ax = plt.axes()
fig = plt.figure(figsize = (15,10))
# subplots(rows,columns)
ax1 = fig.add_subplot(3, 1, 1)
ax2 = fig.add_subplot(3, 1, 2)
ax3 = fig.add_subplot(3, 1, 3)

ax1.set(xlim=(-110, 160), ylim=(-5, 450))
ax2.set(xlim=(-110, 160), ylim=(-5, 450))

data = pd.read_csv('entropySave.csv',delimiter = " ")

# Live Data Setting
drone1Distance1Full = data['EntropyDrone1D1']
drone1Distance2Full = data['EntropyDrone1D2']
drone1Distance3Full = data['EntropyDrone1D3']
drone2Distance1Full = data['EntropyDrone2D1']
drone2Distance2Full = data['EntropyDrone2D2']
drone2Distance3Full = data['EntropyDrone2D3']
drone3Distance1Full = data['EntropyDrone3D1']
drone3Distance2Full = data['EntropyDrone3D2']
drone3Distance3Full = data['EntropyDrone3D3']
e1Full = data['EntropyDrone1']
e2Full = data['EntropyDrone2']
e3Full = data['EntropyDrone3']
timeFull = data['TimeDifference']



global i
i = 0

def animate(i):


    drone1Distance1.append(drone1Distance1Full[i])
    drone1Distance2.append(drone1Distance2Full[i])
    drone1Distance3.append(drone1Distance3Full[i])
    drone2Distance1.append(drone2Distance1Full[i])
    drone2Distance2.append(drone2Distance2Full[i])
    drone2Distance3.append(drone2Distance3Full[i])
    drone3Distance1.append(drone3Distance1Full[i])
    drone3Distance2.append(drone3Distance2Full[i])
    drone3Distance3.append(drone3Distance3Full[i])

    # e1.append(e1Full[i])
    # e2.append(e2Full[i])
    # e3.append(e3Full[i])
    time.append(timeFull[i])



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


    # ax1.set(xlim=([-110, 160]), ylim=([23, 50]))
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


    plt.tight_layout()
    i = i+1

ani = FuncAnimation(fig, animate,frames = 1000, interval = 20,repeat = False)
plt.pause(10)
writerFull= PillowWriter(fps = 30)
# ani.save(filename = 'plotting.gif',writer =writerFull)
plt.show()

