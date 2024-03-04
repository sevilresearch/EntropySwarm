



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

x4 = []
y4 = []
x5 =[]
y5 =[]
x6 =[]
y6 =[]
x4b = []
y4b = []
x5b =[]
y5b =[]
x6b =[]
y6b =[]


e1 =[]
e2 =[]
e3 =[]
e4 =[]
e5 =[]
e6 =[]
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
x4Full = data['X4']
y4Full = data['Y4']
x5Full = data['X5']
y5Full = data['Y5']
x6Full = data['X6']
y6Full = data['Y6']
e1Full = data['EntropyDrone1']
e2Full = data['EntropyDrone2']
e3Full = data['EntropyDrone3']
e4Full = data['EntropyDrone4']
e5Full = data['EntropyDrone5']
e6Full = data['EntropyDrone6']
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
    x4.append(x4Full[i])
    y4.append(y4Full[i])
    x5.append(x5Full[i])
    y5.append(y5Full[i])
    x6.append(x6Full[i])
    y6.append(y6Full[i])


    # To create a line following the markers to look better
    if(i>behindby):
        x1b.append(x1Full[i-behindby])
        y1b.append(y1Full[i-behindby])
        x2b.append(x2Full[i-behindby])
        y2b.append(y2Full[i-behindby])
        x3b.append(x3Full[i-behindby])
        y3b.append(y3Full[i-behindby])
        x4b.append(x4Full[i-behindby])
        y4b.append(y4Full[i-behindby])
        x5b.append(x5Full[i-behindby])
        y5b.append(y5Full[i-behindby])
        x6b.append(x6Full[i-behindby])
        y6b.append(y6Full[i-behindby])

    e1.append(e1Full[i])
    e2.append(e2Full[i])
    e3.append(e3Full[i])
    e4.append(e4Full[i])
    e5.append(e5Full[i])
    e6.append(e6Full[i])
    time.append(timeFull[i])



    ax1.cla()
   
    # ax1.plot(x1b, y1b, linestyle='-',color = 'c')
    # ax1.plot(x2b, y2b,linestyle='-',color = 'r')
    # ax1.plot(x3b, y3b,linestyle='-',color = '#fcbe03')

    # ax1.plot(x1, y1, linestyle='-',color = 'c')
    # ax1.plot(x2, y2,linestyle='-',color = 'r')
    # ax1.plot(x3, y3,linestyle='-',color = '#fcbe03')
    # ax1.plot(x4, y4, linestyle='-',color = 'g')
    # ax1.plot(x5, y5,linestyle='-',color = 'm')
    # ax1.plot(x6, y6,linestyle='-',color = '#FF4500')


    ax1.plot(x1b, y1b, linestyle='-',color = 'c')
    ax1.plot(x2b, y2b,linestyle='-',color = 'r')
    ax1.plot(x3b, y3b,linestyle='-',color = '#fcbe03')
    ax1.plot(x4b, y4b, linestyle='-',color = 'g')
    ax1.plot(x5b, y5b,linestyle='-',color = 'm')
    ax1.plot(x6b, y6b,linestyle='-',color = '#FF4500')
  
    
    
    # Commment out these three lines if too many markers cluttering plot
    # ax1.plot(x2, y2,linestyle='-',marker = 'o',markerfacecolor = "k",color = 'r')

    # ax1.plot(x1, y1, linestyle='-',marker = 'o',markerfacecolor = "k",color = 'c')
    
    # ax1.plot(x3, y3,linestyle='-',marker = 'o',markerfacecolor = "k",color = '#fcbe03')
    

   

    ax2.cla()
    ax2.plot(time, e1, linestyle='-',color = 'c')
    ax2.plot(time, e2, linestyle='--',color = 'r')
    ax2.plot(time, e3, linestyle='--',color = '#fcbe03')
    ax2.plot(time, e4, linestyle='-',color = 'g')
    ax2.plot(time, e5, linestyle='--',color = 'm')
    ax2.plot(time, e6, linestyle='--',color = '#FF4500')

    ax1.set(xlim=([-110, 160]), ylim=([20, 200]))
    ax1.legend(["UAV 1 ","UAV 2","UAV 3","UAV 4 ","UAV 5","UAV 6"])
    ax1.set_xlabel("X values")
    ax1.set_ylabel("Y Values")
    ax1.set_title('Positions')

    ax2.legend(["Entropy 1 ","Entropy 2","Entropy 3","Entropy 4 ","Entropy 5","Entropy 6"])
    ax2.set_xlabel("Time")
    ax2.set_ylabel("Entropy")
    ax2.set_title('Entropy for each UAV')


    plt.tight_layout()
    # i = i+1



ani = FuncAnimation(fig, animate,frames = 10000, interval = 20,repeat = False)

# writerFull= PillowWriter(fps = 30)
# ani.save(filename = 'plotting.gif',writer =writerFull)
ani.save('plotting.mp4', writer="ffmpeg")
# plt.show()
