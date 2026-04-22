
cmake_minimum_required(VERSION 3.18)
project(send_info)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
# ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Kinematics project paths
set(ROBOT_CONTROL_DIR /home/tyler/projects/robot_control_code)
set(KINEMATICS_DIR ${ROBOT_CONTROL_DIR}/kinematic_calibration)

# Atracsys
set(ATRACSYS_INCLUDE_DIR /home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/fusionTrack_SDK-v4.10.1-linux64/include)
set(ATRACSYS_SDK_INSTALL_PATH ${ATRACSYS_INCLUDE_DIR}/..)
set(Atracsys_DIR /home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/fusionTrack_SDK-v4.10.1-linux64/cmake/Atracsys/)
find_package(Atracsys REQUIRED COMPONENTS SDK AdvancedAPI)
find_package(Ceres REQUIRED)

set(POST_LIBS gclib pthread gclibo)

set(SRC_SIMULATION
    ${KINEMATICS_DIR}/templated_classes/Templated_Point.h
    ${KINEMATICS_DIR}/templated_classes/Templated_Transform.h
    ${KINEMATICS_DIR}/jacobian.h
    ${KINEMATICS_DIR}/templated_classes/auto_diff_class/auto.h
    ${KINEMATICS_DIR}/atracsys_functions.h
    ${ROBOT_CONTROL_DIR}/inverse_kinematics.cpp
    ${ROBOT_CONTROL_DIR}/Matrix.cpp
    ${ROBOT_CONTROL_DIR}/Transform.cpp
)

add_executable(send_info
    send_info.cpp
    ${SRC_SIMULATION}
    ${ROBOT_CONTROL_DIR}/Pivot.cpp
)

target_include_directories(send_info PRIVATE
    #${ROBOT_CONTROL_DIR}
    ${KINEMATICS_DIR}
    ${KINEMATICS_DIR}/templated_classes
    ${KINEMATICS_DIR}/templated_classes/auto_diff_class
    ${ROBOT_CONTROL_DIR}/dependencies/eigen-3.4.0
    ${ROBOT_CONTROL_DIR}/dependencies/eigen-3.4.0/Eigen
    ${ROBOT_CONTROL_DIR}/dependencies
    ${ATRACSYS_INCLUDE_DIR}
    /home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/fusionTrack_SDK-v4.10.1-linux64/samples
)

ament_target_dependencies(send_info rclcpp geometry_msgs)

target_link_libraries(send_info 
    ceres
    Atracsys::AdvancedAPI
    ${POST_LIBS}
)

install(TARGETS send_info DESTINATION lib/${PROJECT_NAME})
ament_package()
