cmake_minimum_required(VERSION 3.1)
project(pointcloud_analysis)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

find_package (PCL REQUIRED)
#set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.9)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pointcloud_analysis src/pointcloud_analysis.cpp src/tic_toc.cpp)
target_link_libraries(pointcloud_analysis
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
