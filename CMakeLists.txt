cmake_minimum_required(VERSION 3.10)
project(ODETutorial)

# C++14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# Find ODE library
find_library(ODE_LIBRARY NAMES ode)
find_path(ODE_INCLUDE_DIR ode/ode.h)

if(NOT ODE_LIBRARY)
    message(FATAL_ERROR "ODE library not found. Please install libode-dev")
endif()

if(NOT ODE_INCLUDE_DIR)
    message(FATAL_ERROR "ODE headers not found. Please install libode-dev")
endif()

message(STATUS "ODE Library: ${ODE_LIBRARY}")
message(STATUS "ODE Include: ${ODE_INCLUDE_DIR}")

# Include directories
include_directories(
    ${ODE_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/common
)

# Common library
add_library(ode_common STATIC
    common/viewer.cpp
)

target_link_libraries(ode_common
    ${ODE_LIBRARY}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
    pthread
)

# Tutorial subdirectories
add_subdirectory(01_hello_world)
add_subdirectory(02_falling_sphere)
add_subdirectory(03_collision_detection)
add_subdirectory(04_multiple_objects)
add_subdirectory(05_different_shapes)
add_subdirectory(06_compound_objects)
add_subdirectory(07_hinge_joint)
add_subdirectory(08_slider_joint)
add_subdirectory(09_ball_joint)
add_subdirectory(10_motor)
add_subdirectory(11_ragdoll)
add_subdirectory(12_vehicle)
add_subdirectory(13_robot_arm)
add_subdirectory(14_physics_playground)
add_subdirectory(15_differential_drive)
add_subdirectory(16_odometry)
add_subdirectory(17_lidar_sensor)
add_subdirectory(18_camera_sensor)
add_subdirectory(19_robot_with_lidar)
add_subdirectory(20_walking_pedestrian)
add_subdirectory(21_crowd_simulation)
add_subdirectory(22_dynamic_obstacle_avoidance)
add_subdirectory(23_slam_basics)
add_subdirectory(24_social_navigation)
add_subdirectory(25_autonomous_navigation)

# Print build information
message(STATUS "")
message(STATUS "==================================")
message(STATUS "ODE Tutorial Build Configuration")
message(STATUS "==================================")
message(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "")
