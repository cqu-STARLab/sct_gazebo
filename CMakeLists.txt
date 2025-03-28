cmake_minimum_required(VERSION 3.0.2)
project(sct_gazebo)

# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint
        sct_common

        gazebo_ros
        gazebo_ros_control
        nav_msgs

        sensor_msgs
        )

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        sct_hw_sim
        CATKIN_DEPENDS
        roscpp
        sct_common

        gazebo_ros
        gazebo_ros_control
        DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)
#link_directories(${GAZEBO_LIBRARY_DIRS})

## Declare a cpp library
add_library(sct_robot_hw_sim src/sct_robot_hw_sim.cpp)
add_dependencies(sct_robot_hw_sim
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(sct_robot_hw_sim
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )


add_library(sct_odom_sim src/sct_odom_sim.cpp)
add_dependencies(sct_odom_sim
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(sct_odom_sim
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )

add_library(imu_sim src/imu_sim.cpp)
add_dependencies(imu_sim
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(imu_sim
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )

add_library(sct_chassis_sim src/sct_chassis_sim.cpp)
add_dependencies(sct_chassis_sim
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(sct_chassis_sim
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS sct_robot_hw_sim
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(FILES sct_robot_hw_sim_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

roslint_cpp()
