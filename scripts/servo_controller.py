#!/usr/bin/env python3

import sys
import signal

import rospy

from std_msgs.msg import Float32MultiArray

def signal_handler(signal,frame):
	print('pressed ctrl + c!!!')
	sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

class ServoController:
  def __init__(self):
    print("servoController init")
    rospy.init_node('servoController', anonymous=True)
    rospy.Subscriber('/cmd/servo', Float32MultiArray , self.cmdServoCB)

    self.cmdServoRight = 0
    self.cmdServoLeft = 0

    self.panCmdAngle = 0   # yaw
    self.tiltCmdAngle = 0  # pitch

    # TODO: Servo init(setup)

  def cmdServoCB(self, msg):
    self.panCmdAngle = msg.data[0]
    self.tiltCmdAngle = msg.data[1]

  def run(self):
    rosRate = rospy.Rate(20)
    while not rospy.is_shutdown():
      # TODO: use API to servo motor
      print("panCmdAngle, tiltCmdAngle: [" + str(self.panCmdAngle) + ", " + str(self.tiltCmdAngle) + "]")
      rosRate.sleep()

# main
if __name__ == "__main__":
  servoController = ServoController()
  servoController.run()
