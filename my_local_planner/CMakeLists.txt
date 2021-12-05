cmake_minimum_required(VERSION 2.8.3)
project(my_local_planner)

set(CMAKE_CXX_COMPILER "/usr/bin/clang++-12")

## By default build with code optimization and no debug info
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release"
  	  CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED
        COMPONENTS
            base_local_planner
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            roscpp
            tf2_ros
        )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/MyLocalPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES my_local_planner
    CATKIN_DEPENDS
        dynamic_reconfigure
        nav_msgs
        pluginlib
        roscpp
        tf2_ros
)

add_library(my_local_planner src/MyLocalPlanner.cpp)

add_executable(operational_checker_node src/operational_checker.cpp)

add_dependencies(my_local_planner my_local_planner_gencfg)

target_link_libraries(my_local_planner ${catkin_LIBRARIES}) 
target_link_libraries(operational_checker_node ${catkin_LIBRARIES})

