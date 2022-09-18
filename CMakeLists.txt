cmake_minimum_required(VERSION 3.0.2)
project(nturt_torque_controller)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        can_msgs
        nturt_can_parser
        roscpp
        rospy
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS can_msgs nturt_can_parser roscpp rospy std_msgs
)

include_directories(include
        ${catkin_INCLUDE_DIRS}
)

add_library(torque_controller
        src/torque_controller.cpp
)

add_executable(torque_controller_test
        test/torque_controller_test.cpp
)

add_executable(${PROJECT_NAME}_node
        app/torque_controller_node.cpp
)

target_link_libraries(torque_controller
        ${catkin_LIBRARIES}
        nturt_can_parser
)

target_link_libraries(torque_controller_test
        ${catkin_LIBRARIES}
        torque_controller
)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        torque_controller
)
