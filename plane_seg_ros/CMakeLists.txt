cmake_minimum_required(VERSION 3.0.2)

project(plane_seg_ros)

set(CMAKE_C_STANDARD 11)  # OpenCV4 is using _Atomic, a C11 extension
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  pcl_conversions
  tf_conversions
  tf2_eigen
  tf2_ros
  pcl_ros
  grid_map_core
  grid_map_ros
  grid_map_msgs
  plane_seg
  message_generation
)

find_package(OpenCV 3.0 QUIET)

add_message_files(
  FILES
  convex_hull.msg
  convex_hulls.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    #include
  CATKIN_DEPENDS eigen_conversions pcl_conversions tf_conversions pcl_ros plane_seg message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/Pass.cpp
 )

add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES})

# TODO: Install
