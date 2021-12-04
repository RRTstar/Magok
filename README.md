# mission flow (scenario)
1. IDLE
    - waiting for request from medical team
    - request: goal position
      - goal type
      - x, y axis
2. GO_TO_PATIENT
    - DETECT_OBSTACLES
    - CONTROL
    - COMMUNICATION
3. DETECT_PATIENT
    - detect a patient
    - visual servoing
4. WAIT_FOR_RESPONSE_FROM_PATIENT
5. GO_BACK_HOME

---

# Modules
## Nodes
  1. Mission Planner
  2. Cartographer
      - rplidar
      - imu(myahrs+)
  3. Darknet_ros
      - usb_cam
  4. Motor_controller
  5. Servo_controller
## JS
  1. Comm

---

# Pub & Sub
1. Mission Planner
    - Subscriber
      - Communication data
      - Navigation data
      - BoundingBox data
      - Lidar data
    - Publisher
      - Motor cmd data
      - Servo cmd data

2. Cartographer
    - Subscriber
      - Imu data
      - Lidar data
    - Publisher
      - Navigation data

3. Darknet_ros
    - Subscriber
      - Image data
    - Publisher
      - BoundingBox data

4. Motor_controller
    - Subscriber
      - Motor cmd data
      - Navigation data

5. Servo_controller
    - Subscriber
      - Servo cmd data

---

# Data
## Communication data
  - std_msgs/Float32MultiArray ("/rrt/goal_point")
    - data[0] = [0] Go to patient, [1] Go to home
    - data[1] = targetPositionX
    - data[2] = targetPositionY
  - std_msgs/Bool ("/rrt/is_taken")
    - data = [False] not yet, [True] yes!!!!!!
## Navigation data
  - nav_msgs/Odometry
## BoundingBox data
  - darknet_ros/BoundingBoxs "/darknet_ros/bounding_boxes"
    - Header header
    - Header image_header
    - BoundingBox[] bounding_boxes
      - float64 probability
      - int64 xmin
      - int64 ymin
      - int64 xmax
      - int64 ymax
      - int16 id
      - string Class
## Motor cmd data
  - ackermann_msgs/AckermannDrive
    - float32 steering_angle
    - float32 steering_angle_velocity
    - float32 speed
    - float32 acceleration
    - float32 jerk
## Servo cmd data
  - std_msgs/Float32MultiArray
    - data[0] = panAngle
    - data[1] = tiltAngle
## Imu data
  - sensor_msgs/Imu "/imu/data_raw"
## Lidar data
  - sensor_msgs/PointCloud2 "/scan"
## Image data
  - sensor_msgs/Image "/usb_cam/image_raw"

---

# FSM(Finite State Machine) state
1. IDLE
2. COMMUNICATION
    - Goal point 받기
    - 환자 정보 받기
    - 물품 정보 받기
    - 요청 결과 알리기
    - 미션 수행 요청 받기
3. PATH_PLANNING
4. OPERATION
    - MOVING (Controller)
    - PAUSE
    - DETECTION
