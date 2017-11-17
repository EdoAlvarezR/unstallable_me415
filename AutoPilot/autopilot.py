#!/usr/bin/env python
import rospy
from rosflight_msgs.msg import GPS, Command
import numpy as np


class autopilot:
    def __init__(self):
        self.update_rate = 10  # Hz

        self.gps_subscriber = rospy.Subscriber("/fixedwing/gps", GPS, self.gpsCallback, queue_size=10)
        self.currentGPS = GPS()

        rospy.Timer(rospy.Duration(1./self.update_rate), self.control)
        rospy.Timer(rospy.Duration(1./self.update_rate), self.check_status)
        self.command_publisher = rospy.Publisher("/fixedwing/command", Command, queue_size=1)

        self.wp_state_machine = 0
        self.minAlt = rospy.get_param('~minAlt', 1387.0 + 30.48)  # default is 100 ft
        self.maxAlt = rospy.get_param('~maxAlt', 1387.0 + 121.92)  # default is 400 ft
        self.wp1_lat = rospy.get_param('~wp1_lat', 40.267110)
        self.wp1_lon = rospy.get_param('~wp1_lon', -111.634983)
        self.wp2_lat = rospy.get_param('~wp2_lat', 40.267492)
        self.wp2_lon = rospy.get_param('~wp2_lon', -111.635755)
        self.wp3_lat = rospy.get_param('~wp3_lat', 40.266725)
        self.wp3_lon = rospy.get_param('~wp3_lon', -111.635809)
        self.wp4_lat = rospy.get_param('~wp4_lat', 40.267993)
        self.wp4_lon = rospy.get_param('~wp4_lon', -111.634930)
        #------------------------------------------------------------------------------
        #Target Selection
        self.target=0 #Home/Starting Point
        #Target Locations
        #self.targetlats=[40.267198,40.267638,40.267738,40.26752]
        #self.targetlogs=[-111.635642,-111.635170,-111.635670,-111.635870]
        #Gazebo Test Locations
        self.targetlats=[self.wp1_lat,self.wp2_lat,self.wp3_lat,self.wp4_lat]
        self.targetlogs=[self.wp1_lon,self.wp2_lon,self.wp3_lon,self.wp4_lon]
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

    def gpsCallback(self, msg):
        if msg.fix and msg.NumSat > 3:
            self.currentGPS = msg

    def control(self, event):

        command = Command()
        command.mode = command.MODE_PASS_THROUGH
        command.ignore = command.IGNORE_NONE

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
        rudderscale=rudder/20.0
        #Altitue Check
        hmin=200*0.3048
        hmax=300*0.3048
        #Throttles are arbitrary values
        if hmin>alti:
            throttle=1.0
        elif hmax<alti:
            throttle=0.4
        else:
            throttle=0.75

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
        elevator=0.0
        dalt=np.average(self.alt[0:2])-np.average(self.alt[2:4])
        elevator = -dalt/50
        print dalt
        if elevator>0.7:
            elevator=0.7
        elif elevator<-0.7:
            elevator=-0.7


        command.F = 1.0#throttle # throttle command 0.0 to 1.0
        command.x = 0.0 # aileron servo command -1.0 to 1.0  positive rolls to right
        command.y = elevator-0.02 # elevator servo command -1.0 to 1.0  positive pitches up

        command.z = rudderscale #rudderscale # rudder servo command -1.0 to 1.0  positive yaws to left
        #-------------------------------------------------------------------------------------------------------------

        self.command_publisher.publish(command)

    def check_status(self, event):

        if self.currentGPS.altitude < self.minAlt:
            print "altitude too low!"
            return

        if self.currentGPS.altitude > self.maxAlt:
            print "altitude too high!"
            return

        if self.wp_state_machine == 0 and self.distance(self.wp1_lat, self.wp1_lon) < 10:
            self.wp_state_machine = 1
            print "achieved waypoint 1!"

        elif self.wp_state_machine == 1 and self.distance(self.wp2_lat, self.wp2_lon) < 10:
            self.wp_state_machine = 2
            print "achieved waypoint 2!"

        elif self.wp_state_machine == 2 and self.distance(self.wp3_lat, self.wp3_lon) < 10:
            self.wp_state_machine = 3
            print "achieved waypoint 3!"

        elif self.wp_state_machine == 3 and self.distance(self.wp4_lat, self.wp4_lon) < 10:
            self.wp_state_machine = 4
            print "achieved waypoint 4!"
            print "all waypoints achieved !!!"



    def distance(self, wp_lat, wp_lon):
        EARTH_RADIUS = 6371000.0
        distN = EARTH_RADIUS*(self.currentGPS.latitude - wp_lat)*np.pi/180.0
        distE = EARTH_RADIUS*np.cos(wp_lat*np.pi/180.0)*(self.currentGPS.longitude - wp_lon)*np.pi/180.0
        return np.linalg.norm(np.array([distN, distE]))


if __name__ == '__main__':
    rospy.init_node('autopilot_py', anonymous=True)
    ap = autopilot()
    rospy.spin()
