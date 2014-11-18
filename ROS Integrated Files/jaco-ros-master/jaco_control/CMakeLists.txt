cmake_minimum_required(VERSION 2.8.3)
project(jaco_control)

find_package(catkin REQUIRED COMPONENTS
roscpp rospy std_msgs geometry_msgs genmsg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})

