#!/usr/bin/env python
import numpy as np
import cv2
import math
import gi
import os
import rospy
from std_msgs.msg import String
import imutils
from skimage import exposure
from geometry_msgs.msg import Twist
import math

font = cv2.FONT_HERSHEY_COMPLEX

i = 0

depth = 0

boundaries = [([17, 15, 100], [50, 56, 200])]

#lower = [160, 100, 40]
#upper = [179, 255, 255]

lower = [1, 100, 40]
upper = [30, 255, 255]

# Capture Video and set resolution
pipeline_string = "udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink "

video_capture = cv2.VideoCapture(pipeline_string, cv2.CAP_GSTREAMER)

bigline = 0

def callback(data):
    rospy.loginfo('%s', data.data)
    global depth
    #global i
    depth = data.data
    #depth = float(depth)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('pressure', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # in this case we will avoid this at all costs
    #rospy.spin()



if __name__ == '__main__':

    while video_capture.isOpened():

        # Where the feature detection goes
        ret, img = video_capture.read()
        imCopy = img.copy()
        img = cv2.GaussianBlur(img, (5, 5), 0)

        roi = img[100:380, 200:600]
		
        if not ret:
            print('empty frame')

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_red = np.array(lower)
        upper_red = np.array(upper) 


        mask = cv2.inRange(hsv, lower_red, upper_red) 
        output = cv2.bitwise_and(roi, roi, mask = mask)

        cv2.line(img,(400,600),(400,0),(0,255,0),2)
        cv2.line(img,(380,300),(420,300),(0,255,0),2)

        forward = 0
        turn = 0
        updown = 0

        edges = cv2.Canny(mask, 75, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50)
        if lines is not None:
            bigline = 0
            for line in lines:
                x1, y1, x2, y2 = line[0]
                x_diff = x2 - x1
                y_diff = y2 - y1
                length = abs(math.tan(x_diff/y_diff))
                if length >= bigline:
                    bigline = length
                    x1big, y1big, x2big, y2big = line[0]
                    angle = math.atan2(y2big - y1big, x2big - x1big) * 180
                    print angle
                    if angle < 245 and angle > 0:
                        print "LEFT"
                        turn = .2

                    elif angle > -245 and angle < 0:
                        print "RIGHT"
                        turn = -.2

                    else:
                        print "STRAIGHT"
                        forward = .2
        
            print "X1big = %s" % x1big
            cv2.line(roi, (x1big, y1big), (x2big, y2big), (255, 0, 0), 5)

        listener()

        if depth != 0 and i == 0:
            depth1 = float(depth)
            print depth1
            i = i + 1

        else:
            pass

        ## This is where we show everyone the magic

        depth_gui = "Depth: %s" % depth

        cv2.putText(roi, depth_gui, (0, 10), font, .5, (0,0,255))

        #Rcolor_filtered = np.hstack([img, output])
        cv2.imshow("Crop", roi)
        cv2.imshow("Normal View", img)
        #cv2.imshow("AUTOROV", Rcolor_filtered)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # When everything done, release the capture
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Create Twist message & add linear x and angular z from left joystick
        twist = Twist()
        twist.linear.x =  forward           # Move forward backward
        twist.linear.z = updown
        twist.angular.z = turn              # Turn left right

        # record values to log file and screen
        rospy.loginfo("Linear.x: %f :: Linear.z: %f :: Angular.z: %f ", twist.linear.x, twist.linear.z, twist.angular.z)

        # publish cmd_vel move command to ROV
        pub.publish(twist)


        # When everything done, release the capture
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # Create Twist message & add linear x and angular z from left joystick
    twist = Twist()
    twist.linear.x =  0              # Move forward backward
    twist.linear.z = 0
    twist.angular.z = 0              # Turn left right

    # record values to log file and screen
    rospy.loginfo("Linear.x: %f :: Linear.z: %f :: Angular.z: %f ", twist.linear.x, twist.linear.z, twist.angular.z)

    # publish cmd_vel move command to ROV
    pub.publish(twist)


    video_capture.release()
    cv2.destroyAllWindows()



'''

        lower_red = np.array([110,50,50]) 
        upper_red = np.array([130,255,255]) 

'''