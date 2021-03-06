#!/usr/bin/env python3

import sys
import signal

import rospy

from std_msgs.msg import Float32MultiArray
from servoserial import ServoSerial
import math
from sensor_msgs.msg import Joy

def signal_handler(signal,frame):
	print('pressed ctrl + c!!!')
	sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

class ServoController:
  def __init__(self):
    print("servoController init")
    rospy.init_node('servoController', anonymous=True)
    rospy.Subscriber('/rrt/cmd/servo', Float32MultiArray , self.cmdServoCB)
    rospy.Subscriber('/joy', Joy, self.joyCB)

    self.cmdServoRight = 0
    self.cmdServoLeft = 0

    self.panCmdAngle = 0.0   # yaw (deg)
    self.tiltCmdAngle = 0.0  # pitch (deg)

    # TODO: Servo init(setup)
    self.servo_device = ServoSerial()
    
    self.is_joy = False 

  def cmdServoCB(self, msg):
   self.panCmdAngle = msg.data[0]
   self.tiltCmdAngle = msg.data[1]
    
  def joyCB(self, msg):
    self.is_joy = True 
    self.panCmdAngle = msg.axes[0] * 180 
    self.tiltCmdAngle = -msg.axes[2] * 90 if msg.axes[2]<0 else -msg.axes[2] * 30 

  def map_angle2raw(self, yaw, pitch):
    yaw_raw = int(yaw * 1400/90 + 2550)
    pitch_raw = int(pitch * 1345/90 + 2750)
    return (yaw_raw, pitch_raw)

  def run(self):
      yaw_cmd, pitch_cmd = self.map_angle2raw(self.panCmdAngle, self.tiltCmdAngle)
      self.servo_device.Servo_serial_double_control(1, yaw_cmd, 2, pitch_cmd)
      print("panCmdAngle, tiltCmdAngle: [" + str(self.panCmdAngle * 180/math.pi) + ", " + str(self.tiltCmdAngle*180/math.pi) + "]")

if __name__ == "__main__":
  servoController = ServoController()
  rosRate = rospy.Rate(20)
  while not rospy.is_shutdown():
    servoController.run()
    rosRate.sleep()
