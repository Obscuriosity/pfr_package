#!/usr/bin/python

''' Node to handle the darknet_ros topics and turn the information into
    physical actions, left and right initially, then back and forth, then...who knows?

'''

import rospy # ROS Python Support
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox

class image_handler:

    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.imgW = 0       # cv_image width
        self.imgH = 0       # cv_image height
        self.centrex = 0    # person centre x
        self.centrey = 0    # person centre y
        self.forward = 50   # no of pixels either side of centre where the robot
                            # will not turn/ face forward
        self.minTurnThreshold = 0
        self.maxTurnThreshold = 0
        self.deflection = 0
        self.bbs_callback_ran = False
        # Create subscribers
        # Image only required for debugging, comment in or out as required
        # self.sub_darknet_image = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.sub_bounding_boxes = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbs_callback)
        # create publishers
        # - publish centre(setpoint) and state to pid controller, that will return a value which will be used by motion_controller
        self._rotation_setpoint = rospy.Publisher('rotation_setpoint', Float64, queue_size=10)
        self._rotation_state = rospy.Publisher('rotation_state', Float64, queue_size=10)
        # - publish 'cmd_vel' Twist message for base controller to respond to, with PID controller.

    # subscription callbacks go here:

    def image_callback(self, image):
        # rospy.loginfo("Image received, sequence - %s", image.header.seq)
        self.imgW = image.width
        self.imgH = image.height
        self.minTurnThreshold = (self.imgW/2) - self.forward
        self.maxTurnThreshold = (self.imgW/2) + self.forward
        try:
            #self.cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
            self.cv_image = self.bridge.imgmsg_to_cv2(image, 'rgb8')
        except CvBridgeError as e:
            print(e)
        #cv2.imshow("Person Following Robot view", self.cv_image)
        #cv2.waitKey(100)

    def bbs_callback(self, boxes):
        self.bbs_callback_ran = True
        person = False
        #rospy.loginfo("Bounding Box data, received %s objects ", (len(boxes.bounding_boxes)))
        for bb in range(len(boxes.bounding_boxes)):
            if boxes.bounding_boxes[bb].Class == 'person':
                person = True
                #rospy.loginfo(boxes.bounding_boxes[bb].Class + " detected!")
                xmin = boxes.bounding_boxes[bb].xmin
                xmax = boxes.bounding_boxes[bb].xmax
                ymin = boxes.bounding_boxes[bb].ymin
                ymax = boxes.bounding_boxes[bb].ymax
                self.centrex = int(xmin + (xmax - xmin)/2)
                self.centrey = int(ymin + (ymax - ymin)/2)
                #rospy.loginfo("Person centre = " + str(self.centrex) + ", " + str(self.centrey))
        # self.show_image(person) # for debugging
        self.drive(person)
        # person = False
    
    # other methods go here:

    def personCheck(self):    # Method to check if people are still about?
        if (self.bbs_callback_ran == False):
            rospy.loginfo("BoundingBoxes has not run")
        else:
            rospy.loginfo("BBS Ran")
    
    def drive(self, pobl):    # Method to manage movement based on camera images, will publish CMD_VEL via PID controller
        if pobl:
            self.deflection = self.imgW/2 - self.centrex
            if self.deflection > 0 and self.deflection > self.forward:
                rospy.loginfo("Turn Right " + str(int(self.deflection))) #
            elif self.deflection < 0 and self.deflection < -self.forward: 
                rospy.loginfo("Turn Left " + str(int(self.deflection)))  #
            else:
                rospy.loginfo("Don't Turn ")
                self.deflection = 0.0
        else: # if there are no people in view
            self.deflection = 0.0
            self.speed = 0.0   
        # PID node runs every time a state is published 
        self._rotation_setpoint.publish(0.0)
        self._rotation_state.publish(self.deflection)
        

    def show_image(self, pobl):
        #rospy.loginfo("Showing Image")
        radius = 10
        cv2.line(self.cv_image, (int(self.minTurnThreshold), 0), (int(self.minTurnThreshold), self.imgH), (0, 255, 0), 1)
        cv2.line(self.cv_image, (int(self.maxTurnThreshold), 0), (int(self.maxTurnThreshold), self.imgH), (0, 0, 255), 1)
        if pobl: # pobl (welsh for people) 
            if self.centrex < self.minTurnThreshold:
                cv2.circle(self.cv_image, (self.centrex, self.centrey), radius, (0, 255, 0), 5)
            elif self.centrex > self.maxTurnThreshold:
                cv2.circle(self.cv_image, (self.centrex, self.centrey), radius, (0, 0, 255), 5)
            else:
                cv2.circle(self.cv_image, (self.centrex, self.centrey), radius, (255, 255, 255), 5)
        try:
            
            cv2.imshow("Person Following Robot view", self.cv_image) # Again useful for debugging
            cv2.waitKey(100)
        except:
            rospy.logwarn("Show Image failed")
                
# Main function        
def main():
    r = rospy.Rate(20) # 20Hz
    ih = image_handler()
    # initialise node
    rospy.init_node('PFR_image_analysis_node', anonymous=True)
    rospy.loginfo("Person Following Robot image analysis node started.")
    while not rospy.is_shutdown():
        rospy.loginfo("running while loop")
        try:
            ih.personCheck()
            ih.bbs_callback_ran = False
            r.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting Down...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    
