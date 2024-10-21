cmake_minimum_required(VERSION 3.20)

project(realsense_reader LANGUAGES CXX)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)


# Set the default path for finding dependencies
#set(CMAKE_PREFIX_PATH /opt/ros/humble/share)

# find dependencies
find_package(ament_cmake REQUIRED)
#find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(realsense2)
find_package(rosbag2_cpp REQUIRED)


include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/stream
  ${PROJECT_SOURCE_DIR}/src/read
  ${PROJECT_SOURCE_DIR}/src/record
  ${PROJECT_SOURCE_DIR}/src/segment
  ${PROJECT_SOURCE_DIR}/src/segment/tools
)

set(YOLO_SOURCE
    ${PROJECT_SOURCE_DIR}/src/segment/yolosegment.cpp
    ${PROJECT_SOURCE_DIR}/src/segment/trtprocessor.cpp
    ${PROJECT_SOURCE_DIR}/src/segment/tools/basehelper.cpp
)

include_directories(${OPENCV_INCLUDE_DIRS}) # include headers for opencv
##------------------ Cuda ---------------------
# include_directories(/usr/local/cuda/include)
include_directories("/usr/local/cuda-11.8/include")

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  message_filters
  image_transport
  OpenCV
  rosbag2_cpp
)

# stream
add_executable(stream
    stream.cpp
    src/stream/stream_node.cpp
    ${YOLO_SOURCE}
)

ament_target_dependencies(stream PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  realsense2
)

target_link_libraries(stream PUBLIC
    ${realsense2_LIBRARY}
    -L/usr/local/cuda/lib64 -lcudart -lcuda -lnvinfer -lnvonnxparser
)

# record
add_executable(
    record record.cpp
    src/record/bag_record_node.cpp
    ${YOLO_SOURCE}
)

ament_target_dependencies(record PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(record PUBLIC
    ${OpenCV_LIBRARIES}
    ${Eigen3_LIBS}
    -L/usr/local/cuda/lib64 -lcudart -lcuda -lnvinfer -lnvonnxparser
)

# read from bag
add_executable(read
    read.cpp
    src/read/bag_read_node.cpp)

ament_target_dependencies(read PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  realsense2
)

install(TARGETS stream record read
    DESTINATION lib/${PROJECT_NAME})

ament_package()
