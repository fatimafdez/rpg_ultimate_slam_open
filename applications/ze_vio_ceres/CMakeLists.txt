project(ze_vio_ceres)
cmake_minimum_required(VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
catkin_simple(ALL_DEPS_REQUIRED)

include(ze_setup)

#############
# LIBRARIES #
#############
set(HEADERS
  include/ze/vio_ceres/vio_ceres_backend_interface.hpp
  )

set(SOURCES
  src/vio_ceres_backend_interface.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###############
# EXECUTABLES #
###############

cs_add_executable(ze_vio_ceres_node src/vio_ceres_node.cpp)
target_link_libraries(ze_vio_ceres_node ${PROJECT_NAME} ${OpenCV_LIBRARIES})

add_executable(txt_writing_parameters src/txt_writing_parameters.cpp)
target_link_libraries(txt_writing_parameters ${catkin_LIBRARIES})
# add_dependencies(txt_writing_parameters )

# add_executable(txt_writing_vicon src/txt_writing_vicon.cpp)
# target_link_libraries(txt_writing_vicon ${catkin_LIBRARIES})

# add_executable(txt_writing_odometry src/txt_writing_odometry.cpp)
# target_link_libraries(txt_writing_odometry ${catkin_LIBRARIES})

add_executable(event_msg_length src/event_msg_length.cpp)
target_link_libraries(event_msg_length ${catkin_LIBRARIES})

add_executable(vicon_path src/vicon_path.cpp)
target_link_libraries(vicon_path ${catkin_LIBRARIES})

##########
# GTESTS #
##########

##########
# EXPORT #
##########
cs_install()
cs_export()
