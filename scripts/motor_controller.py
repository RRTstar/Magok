#!/usr/bin/env python3

import sys
import signal

import rospy
import numpy as np

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy


# export PYTHONPATH=$PYTHONPATH:/home/jetbot/jetbot
from jetbot import Robot

def signal_handler(signal,frame):
	print('pressed ctrl + c!!!')
	sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

class MotorController:
  def __init__(self):
    print("motorController init")
    rospy.init_node('motorController', anonymous=True)
    rospy.Subscriber('/rrt/cmd/motor', AckermannDrive , self.cmdMotorCB)
    rospy.Subscriber('/odom', Odometry , self.odomCB)
    rospy.Subscriber('/joy', Joy, self.joyCB)

    self.euler = [0.0, 0.0, 0.0]

    self.velF_mps = 0
    self.velL_mps = 0

    self.steeringAngle = 0
    self.speed_mps = 0
    self.speedRight_mps = 0
    self.speedLeft_mps = 0

    self.cmdMotorRight = 0
    self.cmdMotorLeft = 0

    self.KpSteering = 0.1
    self.KpSpeed = 0.1

    self.robot = Robot()
    
    self.is_joy = False 
    
  def joyCB(self, msg):
    self.is_joy = True 
    self.speedRight_mps = msg.axes[5] * 2
    self.speedLeft_mps = msg.axes[1] * 2

  def cmdMotorCB(self, msg):
    self.steeringAngle = msg.steering_angle
    self.speed_mps = msg.speed

  def odomCB(self, msg):
    q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,)

    self.euler = quaternion2yaw(q)
    self.velF_mps = msg.twist.twist.linear.x * np.cos(self.euler.yaw) + msg.twist.twist.linear.y * np.sin(self.euler.yaw)
    self.velL_mps = msg.twist.twist.linear.x * -np.sin(self.euler.yaw) +msg.twist.twist.linear.x * np.cos(self.euler.yaw)

  def speed2MotorCmdRight(self, speedRight):
    a = 1.0
    b = 0.0
    return a * speedRight + b

  def speed2MotorCmdLeft(self, speedLeft):
    a = 0.9444
    b = 0.0
    return a * speedLeft + b

  def quaternion2yaw(self, q):

    siny_cosp = 2 * (q(0) * q(1) + q(2) * q(3))
    cosy_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2))
    yaw = np.atan2(siny_cosp, cosy_cosp)
    return yaw

  def run(self):
    
    if self.is_joy == False:
        self.speedLeft_mps = self.KpSpeed * (self.speed_mps - self.velF_mps) + self.KpSteering * self.steeringAngle
        self.speedRight_mps = self.KpSpeed * (self.speed_mps - self.velF_mps) - self.KpSteering * self.steeringAngle

    self.cmdMotorLeft = self.speed2MotorCmdLeft(self.speedLeft_mps)
    self.cmdMotorRight = self.speed2MotorCmdRight(self.speedRight_mps)    
    self.robot.left_motor.value = self.cmdMotorLeft
    self.robot.right_motor.value = self.cmdMotorRight
    print("cmdMotorLeft, cmdMotorRight: [" + str(self.cmdMotorLeft) + ", " + str(self.cmdMotorRight) + "]")

if __name__ == "__main__":
  motorController = MotorController()
  rosRate = rospy.Rate(20)
  while not rospy.is_shutdown():
    motorController.run()
    rosRate.sleep()
