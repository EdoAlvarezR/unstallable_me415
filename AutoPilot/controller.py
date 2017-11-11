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

        #self.state_subscriber = rospy.Subscriber("/fixedwing/state", State, self.stateCallback, queue_size=20)
        #self.gps_subscriber = rospy.Subscriber("/fixedwing/gps", GPS, self.gpsCallback, queue_size=10)
        #self.currentState = State()
        #self.currentGPS = GPS()

        #rospy.Timer(rospy.Duration(1./self.update_rate), self.control)
        #self.command_publisher = rospy.Publisher("/fixedwing/command", Command, queue_size=1)
    def simulationStep(self,dt):
        #Simplified Rudder Model
        self.currentGPS.ground_course=self.currentGPS.ground_course+self.command.z
        relVel=self.currentGPS.speed*0.07
        lat=self.currentGPS.latitude-40.0
        log=self.currentGPS.longitude+111.
        gc=self.currentGPS.ground_course
        lat=lat-relVel*np.cos(gc)*dt
        log=log+relVel*np.sin(gc)*dt
        #Calculate New Position
        self.currentGPS.latitude=lat+40.0
        self.currentGPS.longitude=log-111.0

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

        # edit starting right here!!!!
        #Controller Sensatvity Values
        krudder=2
        #Target Locations
        targetx=40.267638-40
        targety=-111.635170+111
        lat=self.currentGPS.latitude-40
        log=self.currentGPS.longitude+111
        #Calculate Target Angle
        htarget=np.arctan2(lat-targetx,log-targety)
        #Calculate heading error
        error=self.currentGPS.ground_course-htarget
        print 'error: ',error
        rudder=-krudder*error
        if rudder>5:
            rudder=5
        elif rudder<-5:
            rudder=-5
        rudderscale=rudder/5.0

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

        self.command.F = 0.75 # throttle command 0.0 to 1.0
        self.command.x = 0.0 # aileron servo command -1.0 to 1.0  positive rolls to right
        self.command.y = -0.03 # elevator servo command -1.0 to 1.0  positive pitches up
        self.command.z = rudderscale # rudder servo command -1.0 to 1.0  positive yaws to left

        #self.command_publisher.publish(command)
'''
if __name__ == '__main__':
    rospy.init_node('autopilot_py', anonymous=True)
    ap = autopilot()
    rospy.spin()
'''
