cmake_minimum_required(VERSION 2.8.3)
project(final_lab)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Boost REQUIRED COMPONENTS system thread)
add_definitions(${EIGEN_DEFINITIONS})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})


catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(my_lib src/my_lib.cpp)   
cs_add_library(alpha_traj_builder src/alpha_traj_builder.cpp) 

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
# cs_add_executable(example src/example.cpp)
cs_add_executable(alpha_lidar_alarm src/alpha_lidar_alarm.cpp)
cs_add_executable(alpha_pub_des_state src/alpha_pub_des_state_main.cpp src/alpha_pub_des_state.cpp)
cs_add_executable(alpha_pub_des_state_path_client src/alpha_pub_des_state_path_client.cpp)
cs_add_executable(alpha_open_loop_controller src/alpha_open_loop_controller.cpp)
cs_add_executable(alpha_lin_steering src/alpha_lin_steering.cpp)
cs_add_executable(alpha_coordinator src/alpha_coordinator.cpp)
cs_add_executable(alpha_object_finder_as src/alpha_object_finder_as.cpp)
cs_add_executable(alpha_coordinator_no_arms src/alpha_coordinator_no_arms.cpp)
cs_add_executable(alpha_acquire_block_client src/alpha_acquire_block_client.cpp)
cs_add_executable(gripper_test src/gripper_test.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(example my_lib)
target_link_libraries(alpha_pub_des_state alpha_traj_builder)

cs_install()
cs_export()
    
