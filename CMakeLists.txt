cmake_minimum_required(VERSION 3.0)
project(thermal2color)
set(ROS_VERSION $ENV{ROS_DISTRO})

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        rospy
        sensor_msgs
        std_msgs
        cv_bridge
        )

include_directories(
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS rospy sensor_msgs std_msgs cv_bridge
)
catkin_python_setup()

catkin_install_python(
  PROGRAMS
    script/thermal2color.py script/loadcpt.py script/blackbody2cpt.py script/cmaptools.py script/cmapy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY script/palettes
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

