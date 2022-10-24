cmake_minimum_required(VERSION 3.0.2)
project(nturt_torque_controller)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        can_msgs
        nturt_ros_interface
        roscpp
        std_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS can_msgs nturt_ros_interface roscpp std_msgs
)

include_directories(include
        ${catkin_INCLUDE_DIRS}
)

# libraries
# library: nturt_torque_controller
add_library(${PROJECT_NAME}
        src/torque_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

# build this package after "nturt_ros_interface" has already finished generating
add_dependencies(${PROJECT_NAME}
        nturt_ros_interface_generate_messages_cpp)

# application
# executable: nturt_torque_controller_node
add_executable(${PROJECT_NAME}_node
        app/torque_controller_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)

# tests
# executable: nturt_torque_controller_test
add_executable(${PROJECT_NAME}_test
        test/torque_controller_test.cpp
)

target_link_libraries(${PROJECT_NAME}_test
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)
