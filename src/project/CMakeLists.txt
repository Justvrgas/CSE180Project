cmake_minimum_required(VERSION 2.8.3)
project(project)


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




add_executable(lasertest src/lasertest.cpp) #lab 1

target_link_libraries(lasertest ${catkin_LIBRARIES} ) #lab 1







