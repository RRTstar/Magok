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
  ## Servo_controller - Wee
# JS
  ## Comm             - Captain Kee

### Data ###
  ## Communication data
    ### std_msgs/Float32MultiArray request_msg
      #### data[0] = patient_info.ID
      #### data[1] = object_info.ID
      #### data[2] = object_info.counts
      #### data[3] = target_point.x
      #### data[4] = target_point.y
    ### std_msgs/Int32
      #### Go to patient [0]
      #### Go to home [1]
    ### std_msgs/String
      #### Note msg for patient
  ## Navigation data
    ### navigation_msgs/Odometry
  ## BoundingBox data
    ### darknet_ros/BoundingBoxs
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
      #### data[0] = pan_angle
      #### data[1] = tilt_angle
  ## Imu data
    ### sensor_msgs/Imu "/imu/data_raw"
  ## Lidar data
    ### sensor_msgs/PointCloud2
  ## Image data
    ### sensor_msgs/Image "/usb_cam/image_raw"


### Mission Planner Pub & Sub ###
# Subscriber
  ## Communication data
  ## Navigation data
  ## BoundingBox data
# Publisher
  ## Communication data
  ## Motor cmd data
  ## Servo cmd data

### Cartographer Pub & Sub ###
# Subscriber
  ## Imu data
  ## Lidar data
# Publisher
  ## Navigation data

### Darknet_ros Pub & Sub ###
# Subscriber
  ## Image data
# Publisher
  ## BoundingBox data

### Motor_controller Pub & Sub ###
# Subscriber
  ## Motor cmd data

### Servo_controller Pub & Sub ###
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



### Objects ###

# 1. JETBOT
# 2. COMM



### 1. JETBOT Functions ###
# 1. STANDBY
# 2. WAYPOINT
## input: goal point
## output: check arrival
# 3. PAUSE
# 4. DETECTION
## input: image
## output: object (u, v [px])

### 2. COMM Functions ###



import rospy

from std_msgs.msg import Float32
from darknet_ros_msgs.msg import BoundingBoxes

import sys
import signal
from enum import Enum, auto

import numpy as np


def signal_handler(signal,frame):
	print('pressed ctrl + c!!!')
	sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

class States(Enum):
  IDLE = auto()
  COMMUNICATION = auto()
  PATH_PLANNING = auto()
  OPERATION = auto()



class MissionPlanner:
  def __init__(self, comm):
    print("missionPlanner init")
    rospy.init_node('missionPlanner', anonymous=True)
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , self.boxCB)
    self.pub = rospy.Publisher("mission/state", Float32, queue_size=1) # mission State
    self.hi='Hello!'
    self.a = 0
    self.target_class = 0 # person
    self.detect_flag = False

    # initial state
    self.state = States.IDLE

    #
    self.target_point = np.array([0.0, 0.0])

    # comm instance
    self.comm = comm


  def idle_transition():
    self.state = States.IDLE

  def boxCB(self, msg):
    for box in msg.bounding_boxes:
      center_x_arr = []
      center_y_arr = []
      if (box.id == self.target_class):
        center_x = (box.xmax + box.xmin)/2.
        center_y = (box.ymax + box.ymin)/2.
        center_x_arr.append(center_x)
        center_y_arr.append(center_y)
    if(len(center_x_arr) != 0):
      self.detect_flag = True
      # get detected object close to the center
      # obj_index = center_x_arr.index(max(center_x_arr))
      # self.center_X = max(center_x_arr)
      # self.center_Y = center_y_arr[obj_index]
      rospy.loginfo("[boxCB] x : %f, y : %f", self.center_X , self.center_Y)
    else:
      self.detect_flag = False
      rospy.loginfo("[boxCB] not detect time : %d", self.not_detect_time)

# 1. IDLE (waiting for connection)
  ## waiting for request from patient
  ## request: patient number, requested stuff
# 2. GO_TO_PATIENT (if #4 is True)
  ## DETECT_OBSTACLES
  ## CONTROL
  ## COMMUNICATION
# 3. DETECT_PATIENT
  ## detect a patient
# 4. WAIT_FOR_RESPONSE_FROM_PATIENT
# 5. GO_BACK_HOME
  def run(self):
    print("run!")
    mission_state = 1
    if (mission_state == 1): # IDLE
      pass
    elif (mission_state == 2): # GO_TO_PATIENT
      pass
    elif (mission_state == 3): # DETECT_PATIENT
      pass
    elif (mission_state == 4): # WAIT_FOR_RESPONSE_FROM_PATIENT
      pass
    elif (mission_state == 5): # GO_BACK_HOME
      pass
    else:
      print("wrong mission state")




# main
if __name__ == "__main__":
  missionPlanner = MissionPlanner()
  missionPlanner.run()
