cmake_minimum_required(VERSION 3.0.2)
project(trash_collection_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++17 -fopenmp -O3) # -march=native)
#add_compile_options(-std=c++17 -fopenmp -Og -g)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  interbotix_xs_msgs
  roscpp
  std_msgs
  visualization_msgs
  tf
)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
   CATKIN_DEPENDS
     roscpp
     interbotix_xs_msgs
     std_msgs
     tf
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)

add_executable(color_detect_node src/color_detect.cpp)
add_dependencies(color_detect_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(color_detect_node ${catkin_LIBRARIES})

add_executable(controller_node src/controller_node.cpp src/arm_controller.cpp src/laser_scan_processor.cpp)
add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_node ${catkin_LIBRARIES})

