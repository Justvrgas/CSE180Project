cmake_minimum_required(VERSION 2.8.3)
project(project)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2 
  tf2_ros 
  actionlib 
  laser_geometry
)

catkin_package(

)


include_directories(

  ${catkin_INCLUDE_DIRS}
)




add_executable(lasertest src/lasertest.cpp) #laser
add_executable(moverobot src/moverobot.cpp) #moverobot



target_link_libraries(lasertest ${catkin_LIBRARIES} ) #laser
target_link_libraries(moverobot ${catkin_LIBRARIES} ) #moverobot








