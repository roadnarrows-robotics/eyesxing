cmake_minimum_required(VERSION 2.8.3)
project(eyesxing_faces)

find_package(catkin 
  REQUIRED COMPONENTS 
  eyesxing_msgs
  sensor_msgs
  std_msgs 
  genmsg 
  message_generation 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

set(rnr_DIR /prj/lib/rnr)
find_package(rnr)

#catkin_python_setup()

#add_message_files(
#  DIRECTORY msg
#  FILES 
#    OpState.msg 
#    ProductInfo.msg 
#    JointStateExtended.msg 
#    RobotStatusExtended.msg
#    ServoHealth.msg
#)

add_service_files(
  DIRECTORY srv
  FILES
    LoadClassifier.srv 
    ClearClassifiers.srv
)

## Generate added messages/services/actions with any dependencies listed here
generate_messages(
  DEPENDENCIES
  eyesxing_msgs
  sensor_msgs
  std_msgs
)

add_executable(faces_detector
  src/faces_detector/faces_detector.cpp
  src/faces_detector/faces_detector_main.cpp
)

include_directories(
  include 
  ../include
  ${rnr_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(faces_detector 
  ${rnr_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(faces_detector eyesxing_faces_gencpp)
