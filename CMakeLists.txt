cmake_minimum_required(VERSION 2.8.3)
project(slam6d_exporter)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs pcl_ros tf)
find_package(PCL REQUIRED)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_ros tf
)

add_executable(slam6d_exporter src/slam6d_exporter.cpp)
target_link_libraries(slam6d_exporter ${catkin_LIBRARIES})
add_dependencies(slam6d_exporter ${catkin_EXPORTED_TARGETS})

install(TARGETS
  slam6d_exporter
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
