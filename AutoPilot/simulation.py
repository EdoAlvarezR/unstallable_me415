import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import controller
class GPSreading:
    def __init__(self,lat,log,heading,vel,alt):
        self.lat=lat
        self.log=log
        self.heading=heading
        self.vel=vel
        self.alt=alt

#Length of simulation (Iterations)
lSim=100
#Set Default values
#GPS Frequency
gpsfreq=10
#Starting Values
heading=0
Velocity=1
lat=0
log=0
alt=10
#Control Surface Starting Point
rudder=0

#Simulation Values
timestep=0 #Start at 0 Time
windspeed = 0 #To implement
#Calculate dt
dt=1.0/gpsfreq
#Initalize GPS Class
GPS = GPSreading(lat,log,heading,Velocity,alt)

def simulationStep(dt,GPS,rudder):
    #Simplified Rudder Model
    GPS.heading=GPS.heading+rudder
    #Calculate New Position
    GPS.lat=GPS.lat+GPS.vel*np.cos(GPS.heading)*dt
    GPS.log=GPS.log+GPS.vel*np.sin(GPS.heading)*dt
    return GPS
def plotter(GPS,targets,fig):
    plt.figure(fig)
    plt.scatter(GPS.lat,GPS.log)

#Data Save
lat=np.zeros(lSim)
log=np.zeros(lSim)
while lSim>timestep:
    #Call Controller
    rudder=controller.controller(GPS)
    #Step Simulation Forward
    GPS=simulationStep(dt,GPS,rudder)
    #Log Lat and Longitude
    lat[timestep]=GPS.lat
    log[timestep]=GPS.log

    print GPS.lat,GPS.log
    #Step Forward
    timestep+=1
plt.scatter(lat,log)
plt.scatter(4,5,color='red')
plt.show()
print 'Done'
