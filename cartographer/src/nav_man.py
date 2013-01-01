#!/usr/bin/python
import math #Because everyone needs some math
import rospy #Necessary for publishing
from ackermann_msgs.msg import AckermannDriveStamped #Controls steering
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
#TODO: remove this TODO and the TODOs below it
#TODO: test this code to make sure I did it right
#TODO: fix comments

class Nav_Man():
    def pathnav(self, data):
        self.pathnav_pushback=data.y
        self.pathnav_turn=data.x

    def wallnav(self, data):
        self.wallnav_pushback=data.y
        self.wallnav_turn=data.x

    def safe(self, data):
        pushback = self.pathnav_pushback + self.wallnav_pushback
        turn = self.pathnav_turn + self.wallnav_turn
    
        self.send=AckermannDriveStamped()
        
        self.send.drive.speed=self.drive_constant * pushback                  #sets net force equal to backforce - pushback
        if self.send.drive.speed > self.max_speed:
            self.send.drive.speed = self.max_speed
        elif self.send.drive.speed < -self.max_speed:
            self.send.drive.speed = -self.max_speed
        self.send.drive.steering_angle = self.turn_constant * turn                              #sets net force = turn* a constant that scales $
        if self.send.drive.speed < 0:                                                           #Checks if the speed is negative
            self.send.drive.steering_angle = -self.send.drive.steering_angle                    #reverses steering angle to turn around while i$
                                                                                                #positive because left is positive but cos(angl$
        
        #safety controller
        if data.data==False:
            self.send.drive.speed=-.2
            
        self.pub.publish(self.send)

    def __init__(self):
	self.drive_constant=1
        self.turn_constant=1
        self.max_speed=2
        self.pathnav_pushback=0
        self.pathnav_turn=0
        self.wallnav_pushback=0
        self.wallnav_turn=0
        rospy.init_node("nav_man", anonymous=False)
        rospy.Subscriber("/path_sum", Point, callback=self.pathnav)
        rospy.Subscriber("/wall_sum", Point, callback=self.wallnav)
        rospy.Subscriber("/safe", Bool, callback=self.safe)
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=6)
        rospy.on_shutdown(self.die)
        rospy.spin()

    def die(self):
        print "Navman down!"

Nav_Man()
