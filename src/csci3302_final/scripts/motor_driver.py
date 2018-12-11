#!/usr/bin/env python2

import rospy
import json
import RPi.GPIO as GPIO

# GPIO pin stuff
_PINS = {
    'A_PWM' : 18,
    'A_IN2' : 19,
    'A_IN1' : 20,
    'STDBY' : 21,
    'B_IN1' : 22,
    'B_IN2' : 23,
    'B_PWM' : 24
}

class Bot:
    def __init__(self):
        self.motor_A_PWM = GPIO.PWM(_PINS['A_PWM'], 50)
        self.motor_B_PWM = GPIO.PWM(_PINS['B_PWM'], 50)
        self.motor_A_PWM.start(0)
        self.motor_B_PWM.start(0)

        GPIO.setup(_PINS['A_IN2'], GPIO.OUT)
        GPIO.setup(_PINS['A_IN1'], GPIO.OUT)
        GPIO.setup(_PINS['STDBY'], GPIO.OUT)
        GPIO.setup(_PINS['B_IN1'], GPIO.OUT)
        GPIO.setup(_PINS['B_IN2'], GPIO.OUT)

        GPIO.output(_PINS['STDBY'], GPIO.HIGH)

    def parse_msg(self, msg):
        msg = json.loads(msg)
        self.set_A(msg['left'])
        self.set_B(msg['right'])

    def set_A(self, speed=0):
        if speed > 0:
            GPIO.output(_PINS['A_IN1'], GPIO.HIGH)
            GPIO.output(_PINS['A_IN2'], GPIO.LOW)
            self.motor_A.ChangeDutyCycle(left)
        elif speed < 0:
            GPIO.output(_PINS['A_IN1'], GPIO.LOW)
            GPIO.output(_PINS['A_IN2'], GPIO.HIGH)
            self.motor_A.ChangeDutyCycle(-left)
        else:
            GPIO.output(_PINS['A_IN1'], GPIO.LOW)
            GPIO.output(_PINS['A_IN2'], GPIO.LOW)

    def set_B(self, speed=0):
        if speed > 0:
            GPIO.output(_PINS['B_IN1'], GPIO.HIGH)
            GPIO.output(_PINS['B_IN2'], GPIO.LOW)
            self.motor_B.ChangeDutyCycle(left)
        elif speed < 0:
            GPIO.output(_PINS['B_IN1'], GPIO.LOW)
            GPIO.output(_PINS['B_IN2'], GPIO.HIGH)
            self.motor_B.ChangeDutyCycle(-left)
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