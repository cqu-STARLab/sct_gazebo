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
        controller_interface
        control_msgs
        dynamic_reconfigure
        geometry_msgs
        hardware_interface
        nav_msgs
        pluginlib
        realtime_tools
        tf
        urdf
        sensor_msgs
        )

find_package(Boost REQUIRED)
# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Declare a catkin package
generate_dynamic_reconfigure_options(config/DiffDriveController.cfg)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        LIBRARIES ${PROJECT_NAME}
        DEPENDS Boost
        sct_hw_sim
        CATKIN_DEPENDS
        roscpp
        sct_common

        controller_interface
        control_msgs
        dynamic_reconfigure
        geometry_msgs
        hardware_interface
        nav_msgs
        realtime_tools
        tf
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

add_library(sct_drive_controller
        src/sct_drive_controller.cpp
        src/odometry.cpp
        src/speed_limiter.cpp
        )
target_link_libraries(sct_drive_controller ${catkin_LIBRARIES})
add_dependencies(sct_drive_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS sct_robot_hw_sim
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(TARGETS sct_drive_controller
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
install(FILES sct_drive_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

roslint_cpp()
