cmake_minimum_required(VERSION 2.8.3)
project(eyesxing_apps)

find_package(catkin REQUIRED roscpp)

set(rnr_DIR /prj/lib/rnr)
find_package(rnr)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
  /opt/include/opencv2
  ${rnr_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(RN_LOGGING_COMPILE_FLAGS "-DLOG -DLOGMOD=\\\"eyesxing\\\"")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${RN_LOGGING_COMPILE_FLAGS}")

add_executable(eyesxing_object_detector src/eyesxing_object_detector.cpp)

target_link_libraries(eyesxing_object_detector 
  /opt/lib/libopencv_core.so
  /opt/lib/libopencv_highgui.so 
  /opt/lib/libopencv_imgproc.so
  /opt/lib/libopencv_objdetect.so
  ${rnr_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS
  eyesxing_object_detector
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
