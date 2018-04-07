cmake_minimum_required(VERSION 2.8.3)
project(offb)

find_package(catkin REQUIRED COMPONENTS
  mavros
  mavros_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories( 
  ${catkin_INCLUDE_DIRS}
)

add_executable(offb_node src/offb_node.cpp)

add_dependencies(offb_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
