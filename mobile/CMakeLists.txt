cmake_minimum_required(VERSION 3.5)
project(stero_mobile_init)
set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf2_ros costmap_2d global_planner nav_msgs base_local_planner clear_costmap_recovery rotate_recovery)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp nodelet
#   DEPENDS elektron
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(listener scripts/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener stero_mobile_init_generate_messages_cpp)

add_executable(example scripts/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})
add_dependencies(example stero_mobile_init_generate_messages_cpp)

install (DIRECTORY worlds/ DESTINATION share/${CMAKE_PROJECT_NAME}/worlds 
         FILES_MATCHING PATTERN "*.world")
install (DIRECTORY launch/ DESTINATION share/${CMAKE_PROJECT_NAME}/launch 
         FILES_MATCHING PATTERN "*.launch*")
install (DIRECTORY config/ DESTINATION share/${CMAKE_PROJECT_NAME}/config 
         FILES_MATCHING PATTERN "*.yaml")
install (DIRECTORY param/ DESTINATION share/${CMAKE_PROJECT_NAME}/param 
         FILES_MATCHING PATTERN "*.yaml")