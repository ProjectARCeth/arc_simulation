cmake_minimum_required(VERSION 2.8.3)
project(erod_control)

## Find catkin and any catkin packages
find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
  	roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    catkin_simple
    )


catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
src/eRodGZInterface.cpp
)

add_executable(eRodGZInterface_node src/eRodGZInterface_node.cpp)
target_link_libraries(eRodGZInterface_node ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_install()
cs_export()
