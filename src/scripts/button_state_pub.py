#! /usr/bin/env python

# Author:   Tony Willett
# Date:     15 2 25
# Description: Node to monitor Dilyn's buttons and publish state

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool

class buttonStatePublisher:

    def __init__(self):
        # Button Pins
        self.red_but_pin = 18
        self.yel_but_pin = 16
        self.gre_but_pin = 22
        self.blu_but_pin = 24
        # Button state log
        self.red_but_state = False
        self.yel_but_state = False
        self.gre_but_state = False
        self.blu_but_state = False

        self.bounce = 250 # Button debounce time

        # Ros Publishers
        self.redButtonPub = rospy.Publisher('redButtonState', Bool, queue_size = 1)
        self.yelButtonPub = rospy.Publisher('yelButtonState', Bool, queue_size = 1)
        self.greButtonPub = rospy.Publisher('greButtonState', Bool, queue_size = 1)
        self.bluButtonPub = rospy.Publisher('bluButtonState', Bool, queue_size = 1)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.red_but_pin, GPIO.IN)
        GPIO.setup(self.yel_but_pin, GPIO.IN)
        GPIO.setup(self.gre_but_pin, GPIO.IN)
        GPIO.setup(self.blu_but_pin, GPIO.IN)

        def redButtonCB(channel):
            self.red_but_state = not self.red_but_state
            self.redButtonPub(self.red_but_state)

        def yelButtonCB(channel):
            self.yel_led_state = not self.yel_led_state
            self.yelButtonPub(self.yel_led_state)

        def greButtonCB(channel):
            self.gre_led_state = not self.gre_led_state
            self.greButtonPub(self.gre_led_state)

        def bluButtonCB(channel):
            self.blu_led_state = not self.blu_led_state
            self.bluButtonPub(self.blu_led_state)

        GPIO.add_event_detect(self.red_but_pin, GPIO.FALLING, callback = redButtonCB, bouncetime = self.bounce)
        GPIO.add_event_detect(self.yel_but_pin, GPIO.FALLING, callback = yelButtonCB, bouncetime = self.bounce)
        GPIO.add_event_detect(self.gre_but_pin, GPIO.FALLING, callback = greButtonCB, bouncetime = self.bounce)
        GPIO.add_event_detect(self.blu_but_pin, GPIO.FALLING, callback = bluButtonCB, bouncetime = self.bounce)


def main():
    rospy.init_node('button_state_node')
    rospy.loginfo("Button Monitor started")
    bsp = buttonStatePublisher()
    rospy.spin()


if __name__ == '__main__':
    main()