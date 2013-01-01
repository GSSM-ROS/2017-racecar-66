#!/usr/bin/python
import math		 #Because everyone needs some math
import rospy		 #Necessary for publishing
from sensor_msgs.msg import LaserScan #receives from the LIDAR
from ackermann_msgs.msg import AckermannDriveStamped #Controls steering
from std_msgs.msg import Bool

class Potentials():

	def drive(self, data):
		self.send=AckermannDriveStamped()
		self.slice=data.ranges[180:900]
		pushback=0				 #pushback is the force in the y(forward, backward) direction
		turn = 0				 #turn is the force in the x(right, left) direction
		for x in range(len(self.slice)):	 #This is a for loop. It lets us do one thing many times.
			angle=x*.25
			pushback += math.sin(math.radians(angle))*self.point_charge/(self.slice[x])**2
			turn += math.cos(math.radians(angle))*self.point_charge/(self.slice[x])**2 		

		speed=self.drive_constant * (self.back_force-pushback)
		self.send.drive.speed=max(-self.max_speed,min(self.max_speed,speed))
		
		self.send.drive.steering_angle = self.turn_constant * turn

		if self.send.drive.speed < 0:
			self.send.drive.steering_angle = -self.send.drive.steering_angle

		self.pub.publish(self.send)


	def __init__(self):
		''' 
		Constructor for Pot_Field to create ros node 'potential'
		'''
		rospy.init_node("potential", anonymous=False)
		rospy.Subscriber("/scan", LaserScan, callback=self.drive)
		self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=6)
		self.safe_pub = rospy.Publisher("/safe", Bool, queue_size=1)
		rospy.on_shutdown(self.die)		#calls back die() on death.

		self.back_force = 5
		self.point_charge = 0.012
		self.turn_constant = .2		#constant to scale turning
		self.drive_constant = .5	#contant to scale drive
		self.max_speed = 1			#maximum speed the robot will go
		rospy.spin()				#keeps the subscriber and publisher going until killed

	def die(self):
		print("Potentialman Down!")

Potentials()

