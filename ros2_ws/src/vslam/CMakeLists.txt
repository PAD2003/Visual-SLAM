cmake_minimum_required(VERSION 3.20)

project(vslam LANGUAGES CXX CUDA)

# TODO: You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS} -pthread")

set(CAFFE2_USE_CUDNN on)

# TODO: You should set path to your libtorch
set(Torch_DIR "/libtorch/share/cmake/Torch")

find_package(Torch REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(visualization_msgs)
find_package(PCL REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED) # Matched with Sophus
find_package(Photo_SLAM REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glm REQUIRED)
find_package(glfw3 REQUIRED)
find_package(realsense2)
find_package(rosbag2_cpp REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/rgbd
  ${PROJECT_SOURCE_DIR}/src/realsense
  ${PROJECT_SOURCE_DIR}/src/segment
  ${PROJECT_SOURCE_DIR}/src/segment/tools
  ${Photo_SLAM_ROOT_DIR}
  ${Photo_SLAM_ROOT_DIR}/include
  ${Photo_SLAM_ROOT_DIR}/viewer
  ${ORB_SLAM3_ROOT_DIR}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)

include_directories(${EIGEN3_INCLUDE_DIRS}) # Include headers for eigen3
include_directories(${OPENCV_INCLUDE_DIRS}) # include headers for pangolin
include_directories(${OPENGL_INCLUDE_DIRS}) # include headers for pangolin
include_directories(${GLFW_INCLUDE_DIRS})

set(YOLO_SOURCE
    ${PROJECT_SOURCE_DIR}/src/segment/yolosegment.cpp
    ${PROJECT_SOURCE_DIR}/src/segment/trtprocessor.cpp
    ${PROJECT_SOURCE_DIR}/src/segment/tools/basehelper.cpp
)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclpy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  message_filters
  image_transport
  rcl_interfaces
  rviz_common
  rviz_default_plugins
  OpenCV
  Eigen3
  rosbag2_cpp
  visualization_msgs
)

add_executable(rgbd main.cpp
    src/rgbd/rgbd_node.cpp
    ${YOLO_SOURCE}
)

ament_target_dependencies(rgbd PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  Photo_SLAM
  OpenGL
  glm
  glfw3
)

target_link_libraries(rgbd PUBLIC
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    "${TORCH_LIBRARIES}"
    "${Photo_SLAM_LIBRARIES}"
    ${Eigen3_LIBS}
    glm::glm
    glfw
    OpenGL::GL
    -L/usr/local/cuda/lib64 -lcudart -lcuda -lnvinfer -lnvonnxparser
)

add_executable(realsense415
    realsense415.cpp
    src/realsense/realsense_node.cpp)

ament_target_dependencies(realsense415 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  realsense2
)

target_link_libraries(realsense415 PUBLIC ${realsense2_LIBRARY})

install(TARGETS rgbd realsense415
    DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
