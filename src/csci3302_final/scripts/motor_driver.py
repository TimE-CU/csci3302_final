#!/usr/bin/env python2

import time

import rospy
import json
import RPi.GPIO as GPIO
from std_msgs.msg import String

# GPIO pin stuff
_PINS = {
    'A_PWM' : 13,
    'A_IN2' : 6,
    'A_IN1' : 5,
    'STDBY' : 22,
    'B_IN1' : 27,
    'B_IN2' : 17,
    'B_PWM' : 4 
}
_BOT_ID = 1

class Bot:
    def __init__(self):
        GPIO.setup(_PINS['A_IN2'], GPIO.OUT)
        GPIO.setup(_PINS['A_IN1'], GPIO.OUT)
        GPIO.setup(_PINS['STDBY'], GPIO.OUT)
        GPIO.setup(_PINS['B_IN1'], GPIO.OUT)
        GPIO.setup(_PINS['B_IN2'], GPIO.OUT)

        GPIO.setup(_PINS['A_PWM'], GPIO.OUT)
        GPIO.setup(_PINS['B_PWM'], GPIO.OUT)
        self.motor_A_PWM = GPIO.PWM(_PINS['A_PWM'], 50)
        self.motor_B_PWM = GPIO.PWM(_PINS['B_PWM'], 50)
        self.motor_A_PWM.start(0)
        self.motor_B_PWM.start(0)

        GPIO.output(_PINS['STDBY'], GPIO.HIGH)
        print("Started motor driver")

    def parse_msg(self, msg):
        msg = msg.data.split()
        left, right = msg
        self.set_A(int(left))
        self.set_B(int(right))

    def set_A(self, speed=0):
        print("Setting A to {}".format(speed))
        if speed > 0:
            GPIO.output(_PINS['A_IN1'], GPIO.HIGH)
            GPIO.output(_PINS['A_IN2'], GPIO.LOW)
            self.motor_A_PWM.ChangeDutyCycle(speed)
        elif speed < 0:
            GPIO.output(_PINS['A_IN1'], GPIO.LOW)
            GPIO.output(_PINS['A_IN2'], GPIO.HIGH)
            self.motor_A_PWM.ChangeDutyCycle(-speed)
        else:
            GPIO.output(_PINS['A_IN1'], GPIO.LOW)
            GPIO.output(_PINS['A_IN2'], GPIO.LOW)

    def set_B(self, speed=0):
        print("Setting B to {}".format(speed))
        if speed > 0:
            GPIO.output(_PINS['B_IN1'], GPIO.HIGH)
            GPIO.output(_PINS['B_IN2'], GPIO.LOW)
            self.motor_B_PWM.ChangeDutyCycle(speed)
        elif speed < 0:
            GPIO.output(_PINS['B_IN1'], GPIO.LOW)
            GPIO.output(_PINS['B_IN2'], GPIO.HIGH)
            self.motor_B_PWM.ChangeDutyCycle(-speed)
        else:
            GPIO.output(_PINS['B_IN1'], GPIO.LOW)
            GPIO.output(_PINS['B_IN2'], GPIO.LOW)

def main():
    # GPIO setup
    GPIO.setmode(GPIO.BCM)

    rospy.init_node('chat_sub', anonymous=True)

    robot = Bot()

    rospy.Subscriber('twitch_chat', String, robot.parse_msg)

    rospy.spin()

if __name__ == '__main__':
    main()