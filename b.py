#!/usr/bin/env python3
import RPi.GPIO as GPIO
from time import sleep
import rospy
from std_msgs.msg import Float32
import math
from gpiozero import Motor
motor = Motor(forward=20, backward=21)
servoPin          = 12
SERVO_MAX_DUTY    = 12
SERVO_MIN_DUTY    = 3
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servoPin, GPIO.OUT)

servo = GPIO.PWM(servoPin, 50)
servo.start(0)

def setServoPos(degree):
    
    if degree > 180:
        degree = 180
    
    duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
    servo.ChangeDutyCycle(duty)
def callback(data):

    yaho = round((data.data),3)
    rospy.loginfo(yaho)
    setServoPos(90+yaho)
    motor.forward(0.3)
def listener():
    rospy.init_node('sub_steers', anonymous=True)
    rospy.Subscriber("/steers", Float32, callback)
    rospy.spin()
if __name__ == '__main__':
    listener()
    servo.stop()
    GPIO.cleanup()
