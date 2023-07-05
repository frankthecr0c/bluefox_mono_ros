cmake_minimum_required(VERSION 2.8.3)
project(bluefox_mono_ros)

SET(CMAKE_CXX_FLAGS "-std=c++1y")

SET( bluefox_mono_ros_LIBRARIES "" )
SET( bluefox_mono_ros_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include )

FIND_PACKAGE(cmake_modules REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem unit_test_framework REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  tf
  visualization_msgs
  dynamic_reconfigure
  image_transport
  camera_info_manager
)

generate_dynamic_reconfigure_options(
  cfg/bluefox_conf.cfg
)

INCLUDE_DIRECTORIES(
  /opt/mvIMPACT_Acquire
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


LIST(APPEND LINK_LIBS
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  /opt/mvIMPACT_Acquire/lib/arm64/libmvPropHandling.so
  /opt/mvIMPACT_Acquire/lib/arm64/libmvGenTLProducer.so
  /opt/mvIMPACT_Acquire/lib/arm64/libmvGenTLConsumer.so
  /opt/mvIMPACT_Acquire/lib/arm64/libmvDeviceManager.so
)

LIST(APPEND SOURCEFILES
)


add_executable(bluefox_mono_node src/bluefox_mono.cpp)     
target_link_libraries(bluefox_mono_node ${LINK_LIBS})

#add_executable(bluefox_mono_lifo_node src/bluefox_mono_lifo.cpp)
#target_link_libraries(bluefox_mono_lifo_node ${LINK_LIBS})

add_dependencies(bluefox_mono_node ${PROJECT_NAME}_gencfg)
#add_dependencies(bluefox_mono_lifo_node ${PROJECT_NAME}_gencfg)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)
