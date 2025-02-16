#! /usr/bin/env python

# Author:   Tony Willett
# Date:     16 2 25
# Description: Node to turn control panel LED's on and off. For now it will simply
# subscribe to the yellow, green and blue buttons and toggle on or off as they are pressed.
# Red LED is connected directly to 3v3 pin and indicates the jetson nano is on.

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool

class led_controller:
    
    def __init__(self):
        
        # LED pins
        self.yel_led_pin = 19
        self.gre_led_pin = 21
        self.blu_led_pin = 23
        # Button state log
        self.yel_led_state = False
        self.gre_led_state = False
        self.blu_led_state = False
    
        def yel_but_CB(self, state):
            self.yel_led_state = state.data
            GPIO.output(self.yel_led_pin, self.yel_led_state)

        def gre_but_CB(self, state):
            self.gre_led_state = state.data
            GPIO.output(self.gre_led_pin, self.gre_led_state)

        def blu_but_CB(self, state):
            self.blu_led_state = state.data
            GPIO.output(self.blu_led_pin, self.blu_led_state)

        # Subscribers
        self.yel_but_sub = rospy.Subscriber('yelButtonState', Bool, yel_but_CB)
        self.gre_but_sub = rospy.Subscriber('greButtonState', Bool, gre_but_CB)
        self.blu_but_sub = rospy.Subscriber('bluButtonState', Bool, blu_but_CB)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.yel_led_pin, GPIO.OUT)
        GPIO.setup(self.gre_led_pin, GPIO.OUT)
        GPIO.setup(self.blu_led_pin, GPIO.OUT)
    
        def yel_but_CB(state):
            self.yel_led_state = state.data
            GPIO.output(self.yel_led_pin, self.yel_led_state)

        def gre_but_CB(state):
            self.gre_led_state = state.data
            GPIO.output(self.gre_led_pin, self.gre_led_state)

        def blu_but_CB(state):
            self.blu_led_state = state.data
            GPIO.output(self.blu_led_pin, self.blu_led_state)


def main():
    rospy.init_node('led_control_panel_node')
    rospy.loginfo("LED started")
    led = led_controller()
    rospy.spin()


if __name__ == '__main__':
    main()