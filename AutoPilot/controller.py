#!/usr/bin/env python
'''
import rospy
from rosplane_msgs.msg import State
from rosflight_msgs.msg import GPS, Command
'''
import numpy as np
class currentGPS:
    def __init__(self):
        self.latitude=0
        self.longitude=0
        self.altitude=0
        self.speed=0
        self.ground_course=0
class command:
    def __init__(self):
        self.F=0
        self.x=0
        self.y=0
        self.z=0

class autopilot:
    def __init__(self,speed,heading,alt):
        self.currentGPS=currentGPS()
        self.command=command()
        self.update_rate = 20
        self.currentGPS.latitude=40.267198
        self.currentGPS.longitude=-111.635642# Deg
        self.currentGPS.altitude=alt # m
        self.currentGPS.speed=speed # m/s
        self.currentGPS.ground_course=heading # rad clockwise from the north

        #------------------------------------------------------------------------------
        #Target Selection
        self.target=0 #Home/Starting Point
        #Target Locations
        self.targetlats=[40.267198,40.267638,40.267738,40.26752]
        self.targetlogs=[-111.635642,-111.635170,-111.635670,-111.635870]
        #Initialization Changes
        self.isFirst=True
        #Running Average Size
        self.tsize=5
        #Running Average Data Storage
        self.lats=np.ones(self.tsize)
        self.logs=np.ones(self.tsize)
        self.ha=np.ones(self.tsize)
        self.alt=np.ones(self.tsize)
        #--------------------------------------------------------------------------------
        #self.state_subscriber = rospy.Subscriber("/fixedwing/state", State, self.stateCallback, queue_size=20)
        #self.gps_subscriber = rospy.Subscriber("/fixedwing/gps", GPS, self.gpsCallback, queue_size=10)
        #self.currentState = State()
        #self.currentGPS = GPS()

        #rospy.Timer(rospy.Duration(1./self.update_rate), self.control)
        #self.command_publisher = rospy.Publisher("/fixedwing/command", Command, queue_size=1)
    def simulationStep(self,dt):
        #Simplified Rudder Model
        self.currentGPS.ground_course=self.currentGPS.ground_course-self.command.z/5
        #Current Vel~= 100m/s
        relVel=self.currentGPS.speed*0.0000007
        lat=self.currentGPS.latitude
        log=self.currentGPS.longitude
        gc=self.currentGPS.ground_course
        log=log+relVel*np.sin(gc)*dt
        lat=lat+relVel*np.cos(gc)*dt
        #Calculate New Position
        self.currentGPS.latitude=lat
        self.currentGPS.longitude=log

    def stateCallback(self, msg):
        'dummy'
        #self.currentState = msg

    def gpsCallback(self, msg):
        if msg.fix and msg.NumSat > 3:
            'empty'
            #self.currentGPS = msg

    def control(self):#, event):
        #command = Command()
        #command.mode = command.MODE_PASS_THROUGH
        #command.ignore = command.IGNORE_NONE
        #----------------------------------------------------
        # edit starting right here!!!!
        #Controller Sensatvity Values
        krudder=5

        #Read in Data
        lat=self.currentGPS.latitude
        log=self.currentGPS.longitude
        head=self.currentGPS.ground_course
        alti=self.currentGPS.altitude

        if self.isFirst:
            self.isFirst=False
            #Initalize Time Averaging at inital point
            self.lats=self.lats*lat
            self.logs=self.logs*log
            self.ha=self.ha*head
            self.alt=self.alt*alti
        #Timeaverage Data
        #Shift Data Backwards
        self.lats=np.roll(self.lats,1)
        self.logs=np.roll(self.logs,1)
        self.ha=np.roll(self.ha,1)
        self.alt=np.roll(self.alt,1)
        #Rewrite oldest point
        self.lats[0]=lat
        self.logs[0]=log
        self.ha[0]=head
        self.alt[0]=alti
        #Calculate Average Value
        lat=np.average(self.lats)
        log=np.average(self.logs)
        head=np.average(self.ha)
        alti=np.average(self.alt)

        #select target location
        tlat=self.targetlats[self.target]
        tlog=self.targetlogs[self.target]

        #Calculate Distance
        radius=6371*1000.0
        dlat=np.radians(lat-tlat)
        dlon=np.radians(log-tlog)
        a = np.sin(dlat/2) * np.sin(dlat/2) + np.cos(np.radians(tlat)) \
        * np.cos(np.radians(lat)) * np.sin(dlon/2) * np.sin(dlon/2)
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
        d = radius * c

        #Test for locations
        if d<7:
            self.target+=1
            print "Waypoint Reached"
            if self.target>3:
                print "Going Home"
                self.target=0


        #Calculate Target Angle
        htarget=np.arctan2(tlog-log,tlat-lat)
        #Calculate heading error
        error=htarget-head
        #Debugging Code
        print 'error: ',error
        self.harget=htarget
        #Proportional Controller
        rudder=-krudder*error
        #Normalization of Rudder Control
        if rudder>5:
            rudder=5
        elif rudder<-5:
            rudder=-5
        rudderscale=rudder/5.0
        #Altitue Check
        hmin=200*0.3048
        hmax=300*0.3048
        #Throttles are arbitrary values
        if hmin>alti:
            throttle=0.8
        elif hmax<alti:
            throttle=0.4
        else:
            throttle=0.6

        # you can get GPS sensor information:
        # self.currentGPS.latitude # Deg
        # self.currentGPS.longitude # Deg
        # self.currentGPS.altitude # m
        # self.currentGPS.speed # m/s
        # self.currentGPS.ground_course # rad clockwise from the north

        # you can get the state estimate information:
        # self.currentState.position[0:3] # north, east, down (m) from starting location
        # self.currentState.Va		# Airspeed (m/s)
        # self.currentState.phi		# Roll angle (rad)
        # self.currentState.theta		# Pitch angle (rad)
        # self.currentState.chi_deg		# Wrapped (-180 to 180) course angle (deg)
        # self.currentState.p		# Body frame rollrate (rad/s)
        # self.currentState.q		# Body frame pitchrate (rad/s)
        # self.currentState.r		# Body frame yawrate (rad/s)

        self.command.F = throttle # throttle command 0.0 to 1.0
        self.command.x = 0.0 # aileron servo command -1.0 to 1.0  positive rolls to right
        self.command.y = 0.0 # elevator servo command -1.0 to 1.0  positive pitches up

        self.command.z = rudderscale # rudder servo command -1.0 to 1.0  positive yaws to left
        #-------------------------------------------------------------------------------------------------------------
        #self.command_publisher.publish(command)
'''
if __name__ == '__main__':
    rospy.init_node('autopilot_py', anonymous=True)
    ap = autopilot()
    rospy.spin()
'''
