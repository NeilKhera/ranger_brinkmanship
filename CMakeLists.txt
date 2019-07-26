cmake_minimum_required(VERSION 3.1)
project(pointcloud_analysis)

find_package(catkin REQUIRED COMPONENTS pcl_conversions pcl_ros roscpp sensor_msgs)

find_package (PCL 1.8 REQUIRED)
#set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.9)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(pointcloud_analysis src/pointcloud_analysis.cpp)
add_executable(rs-pcl-color src/rs-pcl-color.cpp)
target_link_libraries(pointcloud_analysis
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
