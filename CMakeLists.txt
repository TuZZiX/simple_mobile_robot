cmake_minimum_required(VERSION 2.8.3)
project(simple_mobile_robot)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next line to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)

#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# use carefully;  can interfere with point-cloud library
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(my_lib src/my_lib.cpp)
cs_add_library(RobotCommander src/RobotCommander.cpp)

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(path_move src/path_move.cpp)
cs_add_executable(lidar_brake src/lidar_brake.cpp)
cs_add_executable(keyboard_controller src/keyboard_controller.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
target_link_libraries(lidar_brake RobotCommander)

cs_install()
cs_export()
    