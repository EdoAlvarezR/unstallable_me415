import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
#import controller
#import controller
from controller import autopilot
heading = 0
velocity = 14
alt=10
AP=autopilot(velocity,heading,alt)
#Length of simulation (Iterations)
lSim=100
#Set Default values
#GPS Frequency
gpsfreq=1

#Control Surface Starting Point
rudder=0

#Simulation Values
timestep=0 #Start at 0 Time
windspeed = 0 #To implement
#Calculate dt
dt=1.0/gpsfreq
#Initalize Autopilot Class


def plotter(AP,targets,fig):
    plt.figure(fig)
    plt.scatter(AP.latitude,AP.longitude)

#Data Save
lat=np.zeros(lSim)
log=np.zeros(lSim)
hed=np.zeros(lSim)
ht=np.zeros(lSim)
while lSim>timestep:
    #Call Controller
    AP.control()
    #rudder=controller.controller(AP)
    #Step Simulation Forward
    AP.simulationStep(dt)
    #Log Lat and Longitude
    lat[timestep]=AP.currentGPS.latitude
    log[timestep]=AP.currentGPS.longitude
    hed[timestep]=AP.currentGPS.ground_course
    ht[timestep]=AP.harget
    #Step Forward
    timestep+=1
plat=lat
plog=log
fig1=plt.figure()
ax1 = fig1.add_subplot(111, aspect='equal')
plt.plot(plat,plog,label='Position')
sc=0.0001
#Calculate heading line
for j in range(0,len(hed)):
    plt.plot([plat[j],plat[j]+sc*np.cos(hed[j])],[plog[j],plog[j]+sc*np.sin(hed[j])],color='red',linewidth='0.5')
    plt.plot([plat[j],plat[j]+sc*np.sin(ht[j])],[plog[j],plog[j]+sc*np.cos(ht[j])],color='green',linewidth='0.5')
#plt.plot(plat+5*np.sin(hed),plog+5*np.cos(hed),label='Heading')
plt.legend()
plt.scatter(40.267198,-111.635642,color='k')
plt.scatter(40.267638,-111.635170,color='red')
circ=plt.Circle((40.267638,-111.635170),0.00012,fill=False,color='r',linestyle='--')
ax1.add_patch(circ)
plt.show()
print 'Done'
