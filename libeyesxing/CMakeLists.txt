cmake_minimum_required(VERSION 2.8.3)
project(libeyesxing)

find_package(catkin 
  REQUIRED COMPONENTS 
  std_msgs 
  genmsg 
  message_generation 
  roscpp 
  rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

set(rnr_DIR /usr/local/lib/cmake/rnr)
find_package(rnr)

#message(STATUS, "${rnr_LIBRARIES}")

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ../include 
  ${rnr_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(RN_LOGGING_COMPILE_FLAGS "-DLOG -DLOGMOD=\\\"libeyesxing\\\"")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${RN_LOGGING_COMPILE_FLAGS}")
#message(STATUS, ${CMAKE_CXX_FLAGS})

add_library(eyesxing
  src/eyeClassifier.cpp
  src/eyeOOI.cpp
  src/eyeUtils.cpp
)

target_link_libraries(eyesxing 
  ${rnr_LIBRARIES}
)

message(STATUS, ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS eyesxing
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY ../include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
