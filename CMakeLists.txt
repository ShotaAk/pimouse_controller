cmake_minimum_required(VERSION 2.8.3)
project(pimouse_controller)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    std_srvs
    actionlib_msgs
)

catkin_package(
    CATKIN_DEPENDS rospy std_msgs std_srvs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
