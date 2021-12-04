#!/usr/bin/env python3

import sys
import signal
from enum import Enum, auto
import numpy as np

import rospy

from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from ackermann_msgs.msg import AckermannDrive
from darknet_ros_msgs.msg import BoundingBoxes

def signal_handler(signal,frame):
	print('pressed ctrl + c!!!')
	sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

class States(Enum):
  IDLE = auto()
  GO_TO_PATIENT_BED = auto()
  DETECT_PATIENT = auto()
  WAIT_FOR_RESPONSE_FROM_PATIENT = auto()
  GO_TO_HOME = auto()

class MissionPlanner:
  def __init__(self):
    print("missionPlanner init")
    rospy.init_node('missionPlanner', anonymous=True)
    rospy.Subscriber('/rrt/goal_point', Float32MultiArray, self.goalCB)
    rospy.Subscriber('/rrt/is_taken', Bool, self.takenCB)
    rospy.Subscriber('/odom', Odometry, self.odomCB)
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , self.bBoxCB)
    rospy.Subscriber('/scan', PointCloud2 , self.lidarCB)
    rospy.Subscriber('/rrt/set_state', Int32, self.stateCB)
    self.motorPub = rospy.Publisher("/rrt/cmd/motor", AckermannDrive, queue_size=10)
    self.servoPub = rospy.Publisher("/rrt/cmd/servo", Float32MultiArray, queue_size=10)

    self.motorCmd = AckermannDrive()
    self.servoCmd = Float32MultiArray()

    self.state = States.IDLE # initial state

    self.goalType = -1 # [0] Go to patient, [1] Go to home
    self.goalPoint = [0.0, 0.0] # m

    self.curPos = [0.0, 0.0, 0.0] # m
    self.euler = [0.0, 0.0, 0.0] # rad

    self.targetServoPan = 0 # deg
    self.targetServoPanMax = 90 # deg
    self.targetServoTilt = 0 # deg

    self.targetClass = -1 # [0] person
    self.detectFlag = False

    self.front_data_thres = 0.5 # [m]
    self.isObstacles = False

    self.minSpeed = 0 # [m/s]
    self.maxSpeed = 2 # [m/s]
    self.kpSpeed = 0.5

    self.minSteeringAngle = -90 # deg
    self.maxSteeringAngle = 90 # deg
    self.kpSteeringAngle = 1.05

  def goalCB(self, msg):
    rospy.loginfo("[goalCB] Received goal data!")
    self.goalType = msg.data[0] # [0] Go to patient, [1] Go to home
    self.goalPoint[0] = msg.data[1]
    self.goalPoint[1] = msg.data[2]

  def takenCB(self, msg):
    rospy.loginfo("[takenCB] Received taken data!")
    self.isTaken = msg.data # [False] not yet, [True] yes!!!!!!

  def odomCB(self, msg):
    # rospy.loginfo("[odomCB] Received odom data!")
    self.curPos[0] = msg.pose.pose.position.x
    self.curPos[1] = msg.pose.pose.position.y
    self.curPos[2] = msg.pose.pose.position.z

    q = (
          msg.pose.pose.orientation.x,
          msg.pose.pose.orientation.y,
          msg.pose.pose.orientation.z,
          msg.pose.pose.orientation.w)
    self.euler = tf.transformations.euler_from_quaternion(q)

  def bBoxCB(self, msg):
    # rospy.loginfo("[bBoxCB] Received bbox data!")
    for box in msg.bounding_boxes:
      objectCenterArr = []
      if (box.id == self.targetClass):
        objectCenterX = (box.xmax + box.xmin)/2.
        objectCenterY = (box.ymax + box.ymin)/2.
        rospy.loginfo("[boxCB] x : %f, y : %f", objectCenterX , objectCenterY)
        objectCenterArr.append(objectCenterX, objectCenterY)
    if(len(objectCenterArr) != 0):
      self.detectFlag = True
    else:
      self.detectFlag = False

  def lidarCB(self, msg):
    # rospy.loginfo("[lidarCB] Received lidar data!")
    front_float = msg.angle_min/msg.angle_increment
    front = int(front_float)
    offset = 5

    front_data = msg.data[front-offset:front+offset]
    avg_front = np.mean(front_data)

    if avg_front <= self.front_data_thres:
      self.isObstacles = True
    else:
      self.isObstacles = False

  def stateCB(self, msg):
    self.state = msg.data

  def generateMotorCmd(self):
    motorCmd = AckermannDrive()
    distance = np.sqrt((self.curPos[0] - self.goalPoint[0]) **2 + (self.curPos[1] - self.goalPoint[1])**2)
    cmdSpeed = self.kpSpeed * distance
    limitCmdSpeed = max(min(self.maxSpeed, cmdSpeed), self.minSpeed)

    targetSteeringAngle = np.arctan2(self.goalPoint[1]-self.curPos[1], self.goalPoint[0]-self.curPos[0]) * 180.0/np.pi
    errSteeringAngle = targetSteeringAngle - self.euler[2] * 180.0/np.pi # deg
    cmdSteeringAngle = self.kpSteeringAngle * errSteeringAngle
    limitSteeringAngle = max(min(self.maxSteeringAngle, cmdSteeringAngle), self.minSteeringAngle)

    motorCmd.speed = limitCmdSpeed
    motorCmd.steering_angle = limitSteeringAngle

    return motorCmd

  def travel(self):
    if self.isObstacles is True:
      self.motorCmd.speed = 0
      self.motorCmd.steering_angle = 0
    else:
      self.motorCmd = self.generateMotorCmd()

    rospy.loginfo("[travel] speed : %f, steeringAngle : %f", self.motorCmd.speed , self.motorCmd.steering_angle)
    self.motorPub.publish(self.motorCmd)

    distance = np.sqrt((self.curPos[0] - self.goalPoint[0]) **2 + (self.curPos[1] - self.goalPoint[1])**2)
    if distance < 0.3:
      return True
    else:
      return False

  def run(self):
    # rospy.loginfo("run!")
    if self.state == States.IDLE:
      rospy.loginfo("Idle!")
      # waiting for request from medical team (goalCB)
      if self.goalType == 0:
        self.state = States.GO_TO_PATIENT_BED
      elif self.goalType == 1:
        self.state = States.GO_TO_HOME
      else:
        rospy.loginfo("Received goal type is wrong!!")

    elif self.state == States.GO_TO_PATIENT_BED:
      rospy.loginfo("Go to the patient!")
      self.arrived = self.travel()
      if self.arrived is True:
        self.state = States.DETECT_PATIENT
        self.targetServoPan = -self.targetServoPanMax

    elif self.state == States.DETECT_PATIENT:
      rospy.loginfo("Detect patient!")
      # waiting for bbox data (detectFlag)
      if self.detectFlag is False:
        rospy.loginfo("Cannot detect patient!")
        self.targetServoPan += 1
        if self.targetServoPan > self.targetServoPanMax:
          self.targetServoPan = -self.targetServoPanMax

        self.servoCmd.data[0] = self.targetServoPan
        self.servoCmd.data[1] = self.targetServoTilt
        self.servoPub.publish(self.servoCmd)
      else:
        rospy.loginfo("Detected patient!")
        self.state = States.WAIT_FOR_RESPONSE_FROM_PATIENT

    elif self.state == States.WAIT_FOR_RESPONSE_FROM_PATIENT:
      rospy.loginfo("Wait for response from the patient!")
      # waiting for response (takenCB)
      if self.isTaken is True:
        self.state = States.GO_TO_HOME

    elif self.state == States.GO_TO_HOME:
      rospy.loginfo("Go to home!")
      self.arrived = self.travel()
      if self.arrived is True:
        self.state = States.IDLE

    else:
      rospy.loginfo("Wrong mission state")



if __name__ == "__main__":
  missionPlanner = MissionPlanner()
  rosRate = rospy.Rate(20)
  while not rospy.is_shutdown():
    missionPlanner.run()
    rosRate.sleep()
