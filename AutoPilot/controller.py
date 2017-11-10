import numpy as np

class GPSreading:
    def __init__(self,lat,log,heading,vel,alt):
        self.lat=lat
        self.log=log
        self.heading=heading
        self.vel=vel
        self.alt=alt
#Cacluate angle between position and waypoint
def eangle(lat,log,targetx,targety):
    angle=np.arctan2(log-targety,lat-targetx)
    return angle

#Build controller here
def controller(GPS):
    #Controller Setup
    #Controller Sensatvity Values
    krudder=0.01

    #Target Locations
    targetx=4
    targety=5
    print GPS.heading
    #Get heading direction
    htarget=eangle(GPS.lat,GPS.log,targetx,targety)
    #Calculate heading error
    error=GPS.heading-htarget
    #proportional controller
    rudder=krudder*error
    return rudder
