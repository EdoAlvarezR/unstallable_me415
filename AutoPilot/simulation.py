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
    print AP.currentGPS.latitude,AP.currentGPS.longitude,AP.command.z
    #Step Forward
    timestep+=1
plt.plot(lat-40,log+111)
#Calculate heading line
plt.plot(lat+5*np.cos(hed),log+5*np.sin(hed))

plt.scatter(40.267638,-111.635170,color='red')
plt.show()
print 'Done'
