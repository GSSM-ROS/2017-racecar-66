#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray as PA
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid as OG
import aStar

class Pather():
	
	def getPose(self, data):
		print(data)
		self.posx=data.pose.pose.position.x
		self.posy=data.pose.pose.position.y
		
	def getGoal(self, data):
		print(data)
		self.goalx=data.x
		self.goaly=data.y

	def getMap(self, data):
		#check if the we know the goal and position
		if hasattr(self,"posx") and hasattr(self,"posy")  and hasattr(self,"goalx")  and hasattr(self,"goaly") :
			m = data.data #m is a 2d array to be passed to the A*
			print("MAP: " + m)
			#run the astar algo on the occupancy grid
			path=aStar.aStar((self.posy,self.posx), (goaly,goalx), occupancyGrid, free = 255)

			#convert the y,x tuples to poses
			poses=[]
			for i in path:
				p = Pose()
				Pose.position.x=i[0]
				Pose.position.y=i[1]
				poses.append(p)
			pa = PA()
			pa.poses=poses
			self.pub.publish(pa)
		else:
			print("Unknown start/stop")

	def __init__(self):
		rospy.init_node("pather", anonymous=False)
		self.pose=Pose()
		rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback=self.getPose)
		rospy.Subscriber("/map", OG, callback=self.getMap)
		rospy.Subscriber("/goal", Pose, callback=self.getGoal)
		self.pub = rospy.Publisher("/path", PA, queue_size=6)
		rospy.on_shutdown(self.die)
		rospy.spin()

	def die(self):
		print("Pather died!")

Pather()

