#!/usr/bin/python

#imports
import cv2
import imutils
import numpy
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Image


class Image_Echo:
    def __init__(self):
        '''
        initializes the image echoer node
        '''
        rospy.init_node("Image_Echo", anonymous=False)
        self.pub = rospy.Publisher("/contour", Image, queue_size=2)
        rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.callback
        
        self.bridge = CvBridge()
        
        
    #function to find a tennis ball
    def find_contour(self, img, lhsv=(35,110,175), uhsv=(95,255,255)):
        #im = cv2.imread(img)  #opens file
        im = self.bridge.imgmsg_to_cv2(img)
        
        
        
        
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)   #converts to hsv
    
        r1 = numpy.array(lhsv) #35,110,175             #numpy arrays for hsv detection
        r2 = numpy.array(uhsv)#95,255,255
    
        mask = cv2.inRange(hsv,r1,r2)               #applys bitmask for in range values

        mask = cv2.GaussianBlur(mask,(51,51),0)     #blurs image to smooth noise

        mask = cv2.erode(mask, (5,5), iterations=20) #erodes noise to focus on main contour

        Contours, h = cv2.findContours(mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                                                #creates a list of contours

        bigContour = Contours[0]                    #says biggest contour is first
        bigArea = cv2.contourArea(Contours[0])

        for i in Contours:                          #simple for loop algorith to check for largest contour area
            checkArea = cv2.contourArea(i)
            if checkArea > bigArea:
                bigArea = checkArea
                bigContour = i

        M = cv2.moments(bigContour)                 #creates moments of contour
        cX = int(M["m10"]/M["m00"])                 #uses moments for the circle
        cY = int(M["m01"]/M["m00"])

        cv2.drawContours(im, [bigContour], -1, (255,0,255), 2)#draws the contours
        cv2.circle(im, (cX, cY), 12 ,(50, 50, 255), -1)    #draws the circle
        
        
        pub_im = self.bridgecv2_to_imgmsg(im, "bgr8")
        
        self.pub.Publish(pub_im)
        
        #plt.imshow(mask)
        #plt.show()
        #cv2.namedWindow('', cv2.WINDOW_NORMAL)      #creates a named window so that we dont have to look at a huge pile of pixels, BRENNAN
        #cv2.resizeWindow('', 600, 600)
        #cv2.imshow("", im)                          #shows image, waits for a key
        #cv2.waitKey(0)


    #runs if the program is run, but can be accessed later
    #if __name__ == "__main__":                      #runs the program on each image
    #    find_contour("TennisBall1.jpg")
    #    find_contour("TennisBall2.jpg")
