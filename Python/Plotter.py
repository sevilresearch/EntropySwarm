# -*- coding: utf-8 -*-
"""
Created on Thu May 28 16:27:51 2020

@author: lukef
"""


import pandas as pd


import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter




x1 = []
y1 = []
x2 =[]
y2 =[]
x3 =[]
y3 =[]
x1b = []
y1b = []
x2b =[]
y2b =[]
x3b =[]
y3b =[]


e1 =[]
e2 =[]
e3 =[]
time =[]



plt.style.use('fivethirtyeight')
# ax = plt.axes()
fig = plt.figure(figsize = (15,10))
# subplots(rows,columns)
ax1 = fig.add_subplot(2, 1, 1)
ax2 = fig.add_subplot(2, 1, 2)

ax1.set(xlim=(-110, 160), ylim=(-5, 450))

ax2.set(xlim=(-110, 160), ylim=(-5, 450))

data = pd.read_csv('entropySave.csv',delimiter = " ")

# Live Data Setting
x1Full = data['X1']
y1Full = data['Y1']
x2Full = data['X2']
y2Full = data['Y2']
x3Full = data['X3']
y3Full = data['Y3']
e1Full = data['EntropyDrone1']
e2Full = data['EntropyDrone2']
e3Full = data['EntropyDrone3']
timeFull = data['TimeDifference']




# line = ax.plot(x1Full[0,:],y1Full[0,:] , color='k', lw=2)[0]

global i
i = 0
behindby = 1
def animate(i):
    # line.set_ydata(x1Full[0,i],y1Full[0,i])
 
    x1.append(x1Full[i])
    y1.append(y1Full[i])
    x2.append(x2Full[i])
    y2.append(y2Full[i])
    x3.append(x3Full[i])
    y3.append(y3Full[i])

    # To create a line following the markers to look better
    if(i>behindby):
        x1b.append(x1Full[i-behindby])
        y1b.append(y1Full[i-behindby])
        x2b.append(x2Full[i-behindby])
        y2b.append(y2Full[i-behindby])
        x3b.append(x3Full[i-behindby])
        y3b.append(y3Full[i-behindby])
        

        

    e1.append(e1Full[i])
    e2.append(e2Full[i])
    e3.append(e3Full[i])
    time.append(timeFull[i])



    ax1.cla()
   
    ax1.plot(x1b, y1b, linestyle='-',color = 'c')
    ax1.plot(x2b, y2b,linestyle='-',color = 'r')
    ax1.plot(x3b, y3b,linestyle='-',color = '#fcbe03')
  
    
    
    # Commment out these three lines if too many markers cluttering plot
    # ax1.plot(x2, y2,linestyle='-',marker = 'o',markerfacecolor = "k",color = 'r')

    # ax1.plot(x1, y1, linestyle='-',marker = 'o',markerfacecolor = "k",color = 'c')
    
    # ax1.plot(x3, y3,linestyle='-',marker = 'o',markerfacecolor = "k",color = '#fcbe03')
    

   

    ax2.cla()
    ax2.plot(time, e1, linestyle='-',color = 'c')
    ax2.plot(time, e2, linestyle='--',color = 'r')
    ax2.plot(time, e3, linestyle='--',color = '#fcbe03')

    ax1.set(xlim=([-110, 160]), ylim=([20, 200]))
    ax1.legend(["UAV 1 ","UAV 2","UAV 3"])
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title('Positions')

    ax2.legend(["Entropy 1 ","Entropy 2","Entropy 3"])
    ax2.set_xlabel("Time(s)")
    ax2.set_ylabel("Entropy")
    ax2.set_title('Entropy for each UAV')


    plt.tight_layout()
    # i = i+1



ani = FuncAnimation(fig, animate,frames = 10000, interval = 20,repeat = False)

# writerFull= PillowWriter(fps = 30)
# ani.save(filename = 'plotting.gif',writer =writerFull)
ani.save('plotting.mp4', writer="ffmpeg")
# plt.show()

