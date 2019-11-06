cmake_minimum_required(VERSION 3.1)
project(ranger_brinkmanship)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  message_generation
)
find_package (PCL REQUIRED)

add_message_files(FILES Distances.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ranger_brinkmanship src/ranger_brinkmanship.cpp)
target_link_libraries(ranger_brinkmanship
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(ranger_brinkmanship ${ranger_brinkmanship_EXPORTED_TARGETS})
