cmake_minimum_required(VERSION 3.4)
project(RGMC_In-Hand_Manipulation_2024)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  usb_cam
  apriltag_ros
  message_generation
  std_msgs
  actionlib_msgs
  visualization_msgs
  std_srvs
)

# catkin_python_setup()


# add_service_files(FILES
#   RecordWaypoint.srv
#   StartTask.srv
# )

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES webcam_utils
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)