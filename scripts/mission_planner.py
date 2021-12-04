### Enum ###

# mission flow (scenario)
# 1. IDLE (waiting for connection)
  ## waiting for request from patient
  ## request: patient number, requested stuff
# 2. GO_TO_PATIENT (if #4 is True)
  ## DETECT_OBSTACLES
  ## CONTROL
  ## COMMUNICATION
# 3. DETECT_PATIENT
  ## detect a patient
  ## visual servoing
# 4. WAIT_FOR_RESPONSE_FROM_PATIENT
# 5. GO_BACK_HOME

### Nodes ###
  ## Mission Planner  - ??? all?
  ## Cartographer     - Joon
    ### rplidar       - done
    ### imu(myahrs+)  - done
  ## Darknet_ros      - done
    ### usb_cam       - done
  ## Motor_controller - Wee
  ## Servo_controller - Doyoon
# JS
  ## Comm             - Captain Kee

### Data ###
  ## Communication data
    ### std_msgs/Float32MultiArray request_msg
      #### data[0] = patient_info.ID
      #### data[1] = object_info.ID
      #### data[2] = object_info.counts
      #### data[3] = targetPoint.x
      #### data[4] = targetPoint.y
    ### std_msgs/Int32
      #### Go to patient [0]
      #### Go to home [1]
  ## Navigation data
    ### nav_msgs/Odometry
  ## BoundingBox data
    ### darknet_ros/BoundingBoxs "/darknet_ros/bounding_boxes"
      #### Header header
      #### Header image_header
      #### BoundingBox[] bounding_boxes
        ##### float64 probability
        ##### int64 xmin
        ##### int64 ymin
        ##### int64 xmax
        ##### int64 ymax
        ##### int16 id
        ##### string Class
  ## Motor cmd data
    ### ackermann_msgs/AckermannDrive
      #### float32 steering_angle
      #### float32 steering_angle_velocity
      #### float32 speed
      #### float32 acceleration
      #### float32 jerk
  ## Servo cmd data
    ### std_msgs/Float32MultiArray
      #### data[0] = panAngle
      #### data[1] = tiltAngle
  ## Imu data
    ### sensor_msgs/Imu "/imu/data_raw"
  ## Lidar data
    ### sensor_msgs/PointCloud2
  ## Image data
    ### sensor_msgs/Image "/usb_cam/image_raw"

### Pub & Sub ###
### Mission Planner ###
  # Subscriber
    ## Communication data
    ## Navigation data
    ## BoundingBox data
  # Publisher
    ## Communication data
    ## Motor cmd data
    ## Servo cmd data

### Cartographer ###
  # Subscriber
    ## Imu data
    ## Lidar data
  # Publisher
    ## Navigation data

### Darknet_ros ###
  # Subscriber
    ## Image data
  # Publisher
    ## BoundingBox data

### Motor_controller ###
  # Subscriber
    ## Motor cmd data
    ## Navigation data

### Servo_controller ###
  # Subscriber
    ## Servo cmd data


## FSM(Finite State Machine) state
# 1. IDLE
# 2. COMMUNICATION
#     - Goal point 받기
#     - 환자 정보 받기
#     - 물품 정보 받기
#     - 요청 결과 알리기
#     - 미션 수행 요청 받기
# 3. PATH_PLANNING
# 4. OPERATION
#     - MOVING (Controller)
#     - PAUSE
#     - DETECTION

#!/usr/bin/env python

import sys
import signal
from enum import Enum, auto
import numpy as np

import rospy

from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from darknet_ros_msgs.msg import BoundingBoxes

def signal_handler(signal,frame):
	print('pressed ctrl + c!!!')
	sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

class States(Enum):
  IDLE = auto()
  GO_TO_PATIENT = auto()
  DETECT_PATIENT = auto()
  WAIT_FOR_RESPONSE_FROM_PATIENT = auto()
  GO_BACK_HOME = auto()

class MissionPlanner:
  def __init__(self, comm):
    print("missionPlanner init")
    rospy.init_node('missionPlanner', anonymous=True)
    rospy.Subscriber('/comm/info', Float32, self.infoCB)
    rospy.Subscriber('/comm/state', Int32, self.stateCB)
    rospy.Subscriber('/odom', Odometry, self.odomCB)
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , self.boxCB)
    self.motorPub = rospy.Publisher("/cmd/motor", AckermannDrive, queue_size=1)
    self.servoPub = rospy.Publisher("/cmd/servo", Float32MultiArray, queue_size=1)

    self.odom = Odometry()
    self.servoCmd = Float32MultiArray()

    self.cur_pos_m = np.array([0.0, 0.0, 0.0])

    self.patientInfoID = 0
    self.objectInfoID = 0
    self.objectInfoCounts = 0
    self.targetPositionX = 0
    self.targetPositionY = 0


    self.hi='Hello!'
    self.a = 0
    self.targetClass = 0 # person
    self.detectFlag = False
    self.targetServoPan = 0
    self.targetY = 0
    self.detect_flag = False

    self.callbacks = {}


    # initial state
    self.state = States.IDLE

    #
    self.targetPoint = np.array([0.0, 0.0])

    # comm instance
    self.comm = comm


  def idle_transition(self):
    self.state = States.IDLE

  def go_to_patient_transition(self):
    self.state = States.GO_TO_PATIENT
    # calculate motor/servo cmd
    cmd = generate_cmd()
    # motorPub.publish


  def detect_patient_transition(self):
    self.state = States.DETECT_PATIENT

  def wait_for_response_from_patient(self):
    self.state = States.WAIT_FOR_RESPONSE_FROM_PATIENT

  def infoCB(self, msg):
    self.patientInfoID = msg.data[0]
    self.objectInfoID = msg.data[1]
    self.objectInfoCounts = msg.data[2]
    self.targetPositionX = msg.data[3]
    self.targetPositionY = msg.data[4]

  def stateCB(self, msg):
    pass

  def odomCB(self, msg):
    self.odom = msg
    self.cur_pos_m[0] = self.odom.pose.pose.position.x
    self.cur_pos_m[1] = self.odom.pose.pose.position.y
    self.cur_pos_m[2] = self.odom.pose.pose.position.z

  def boxCB(self, msg):
    for box in msg.bounding_boxes:
      centerXArr = []
      centerYArr = []
      if (box.id == self.targetClass):
        centerX = (box.xmax + box.xmin)/2.
        centerY = (box.ymax + box.ymin)/2.
        centerXArr.append(centerX)
        centerYArr.append(centerY)
    if(len(centerXArr) != 0):
      self.detectFlag = True
      # get detected object close to the center
      # obj_index = centerXArr.index(max(centerXArr))
      # self.centerX = max(centerXArr)
      # self.centerY = centerYArr[obj_index]
      targetX = np.average(centerXArr)
      targetY = np.average(centerYArr)
      rospy.loginfo("[boxCB] x : %f, y : %f", targetX , targetY)
    else:
      self.detectFlag = False

  def generate_cmd(self):
    #self.cur_pos_m
    #self.target_point
    motor_cmd = np.array([0.0, 0.0])
    servo_cmd = np.array([0.0, 0.0])
    return {"motor_cmd": motor_cmd, "servo_cmd": servo_cmd}


# 1. IDLE (waiting for connection)
  ## waiting for request from patient
  ## request: patient number, requested stuff
# 2. GO_TO_PATIENT (if #4 is True)
  ## DETECT_OBSTACLES
  ## CONTROL
  ## COMMUNICATION
# 3. DETECT_PATIENT
  ## detect a patient
  ## visual servoing
# 4. WAIT_FOR_RESPONSE_FROM_PATIENT
# 5. GO_BACK_HOME
  def run(self):
    print("run!")
    if (self.state == States.IDLE):

      pass

    elif (self.state == States.GO_TO_PATIENT):
      print("Go to the patient!")

      ## COMMUNICATION

      ## DETECT_OBSTACLES

      ## CONTROL

      pass

    elif (self.state == States.DETECT_PATIENT):
      print("Detect the patient!")

      if (self.detect_flag == True):
        self.servoCmd.data[0] = self.targetServoPan
        self.servoCmd.data[1] = self.targetY
        self.servoPub.publish(self.servoCmd)
      else:
        print("Cannot detect the patient!")

      pass

    elif (self.state == States.WAIT_FOR_RESPONSE_FROM_PATIENT):
      print("Wait for response from the patient!")

      pass

    elif (self.state == States.GO_BACK_HOME):
      print("Go back home!")

      pass

    else:
      print("wrong mission state")



# main
if __name__ == "__main__":
  missionPlanner = MissionPlanner()
  missionPlanner.run()
