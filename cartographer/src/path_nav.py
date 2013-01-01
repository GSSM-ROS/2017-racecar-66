#! /usr/bin/env python

'''
Potential field attraction to detected path and endpoint
'''

import rospy
from geometry_msgs.msg import Pose, Point, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import MapMetaData
from ackermann_msgs.msg import AckermannDriveStamped
import math
import sys
from os.path import join as pathjoin, abspath, dirname, normpath

PROJECT_ROOT = normpath(dirname(abspath(sys.argv[0])))

class Potential_Path:
    def __init__(self):
        '''
        Creates Potential_Path object and begins spinning, waiting for pose arrays
        '''
        rospy.init_node('potential_path', anonymous=False)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback=self.pose_sub)
        rospy.Subscriber('/path', PoseArray, callback=self.update_path)
        rospy.Subscriber('/map_metadata', MapMetaData, callback=self.update_origin_offset)
        self.pub = rospy.Publisher('/path_sum', Point, queue_size=5) #x and y magnitudes of pull
        ackerpub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)

        try:
            self.path = self.get_path(pathjoin(PROJECT_ROOT, "path.csv"))
        except Exception as e:
            self.path = []
            print("Error: unable to read path file")
            print("Stacktrace: %s" % e)
        self.path_charge = rospy.get_param("path_point")
        self.endpoint_charge = rospy.get_param("goal_point")
        
        start_msg = AckermannDriveStamped()
        start_msg.drive.speed = 0.1
        ackerpub.publish(start_msg)

        rospy.spin()
    
    def update_origin_offset(self, msg):
        '''
        Updates origin offset data
        '''
        print("Updating origin offset")
        self.resolution = msg.resolution
        position = msg.origin.position
        self.offset_x = position.x
        self.offset_y = position.y

    def pose_sub(self, msg):
        '''
        Calculates "Coulomb Force" between robot and path + endpoint then publishes to /path_sum
        :param msg: Pose message from /amcl_pose
        '''
        robo_pos = msg.pose.pose.position
        robo_x = (robo_pos.x - self.offset_x) / self.resolution
        robo_y = (robo_pos.y - self.offset_y) / self.resolution
        robo_z = msg.pose.pose.orientation.z
        
        print("robo pos %s,%s" % (robo_x, robo_y)) #TODO remove

        sum_x = 0
        sum_y = 0
        
        #add force for each point in path
        for path_point in self.path[:-1]:
            x, y = path_point
            f_x, f_y = path_force(self.path_charge, x-robo_x, y-robo_y)
            sum_x += f_x
            sum_y += f_y

        print("path sum %s,%s" % (sum_x, sum_y)) #TODO remove
        
        #add force of endpoint
        if len(self.path) > 0:
            end_x, end_y = self.path[-1]
            f_x_end, f_y_end = endpoint_force(self.endpoint_charge, end_x-robo_x, end_y-robo_y)

            print("endpoint sum %s,%s\n" % (f_x_end, f_y_end)) #TODO remove

            sum_x += f_x_end
            sum_y += f_y_end
            print("endpoint vs robot %s, %s" % (end_x, end_y))

        hyp = math.sqrt(sum_x**2 + sum_y**2)
        robo_offset = robo_z * math.pi
        goal_offset = math.atan2(sum_y, sum_x)
        total_offset = robo_offset + goal_offset
        tilted_sum_x = math.sin(total_offset)
        tilted_sum_y = math.cos(total_offset)

        print("tilted axes %s, %s" % (tilted_sum_x, tilted_sum_y))
            
        #package force as Point message
        force_sum = Point()
        force_sum.x = tilted_sum_x
        force_sum.y = tilted_sum_y
        self.pub.publish(force_sum)
    
    def update_path(self, msg):
        new_path = []
        for pose in msg.poses:
            point = pose.position
            x = int(point.x)
            y = int(point.y)
            new_path.append((x,y))
        self.path = new_path

    def get_path(self, filename):
        '''
        Gets file directly from beside where it is
        '''
        path = []
        fp = open(filename, "r")
        for line in fp:
            print(line)
            y_str, x_str = line.split(',')
            x = int(int(x_str))
            y = int(int(y_str))
            path.append((x,y))
            print(x,y) #TODO remove
        fp.close()
        return path

def path_force(Q, dx, dy, q=1, k=1):
    r = math.sqrt(dx**2 + dy**2)
    total_force_asterisk = k * Q * q / r**2
    return total_force_asterisk * dx, total_force_asterisk * dy

def endpoint_force(Q, dx, dy, q=1, k=1):
    '''
    Calculates associated force
    :return: returns F_x, F_x
    '''
    r = math.sqrt(dx**2 + dy**2)
    total_force_asterisk = k * Q * q / r**(1/2) #total force but divided by r to make life easier later
    return total_force_asterisk * dx, total_force_asterisk * dy

Potential_Path()
