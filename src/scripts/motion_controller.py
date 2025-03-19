#! /usr/bin/env python

# Author: Tony Willett
# Date: 19th mar 2025
# Description: Node to receive cmd-vel messages from other nodes
# and combine them into one for the Base Controller node to control
# the wheels

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class motionController:

    def __init__(self):

        self.speed = 0.0
        self.spin = 0.0
        self.command_velocity = Twist()

        # Subscribers here:
        self.velocity_cmd_vel_Sub = rospy.Subscriber('user_velocity/control_effort', Float64, self.velocity_cmd_vel_CB)
        self.rotation_cmd_vel_Sub = rospy.Subscriber('user_rotation/control_effort', Float64, self.rotation_cmd_vel_CB)
        # Publisher here:
        self.combined_cmd_vel_Pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # Call Backs Here:
    def velocity_cmd_vel_CB(self, msg):
        self.speed = msg.data
        rospy.loginfo("Motion Controller: Velocity Received = %s", self.speed)
        self.update_cmd_vel()

    def rotation_cmd_vel_CB(self, msg):
        self.spin = msg.data
        rospy.loginfo("Motion controller: Rotation Received = %s", self.spin)
        self.update_cmd_vel()
    
    # Other methods here:
    def update_cmd_vel(self):
        self.command_velocity.linear.x = self.speed
        self.command_velocity.angular.z = self.spin
        self.combined_cmd_vel_Pub.publish(self.command_velocity)


def main():
    rospy.init_node('motion_controller')
    rospy.loginfo("Motion Controller starting")
    mc = motionController()
    rospy.spin()


if __name__ == '__main__':
    main()