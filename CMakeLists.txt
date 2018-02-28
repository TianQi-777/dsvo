cmake_minimum_required(VERSION 2.8.3)
project(direct_stereo)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_filters
  dynamic_reconfigure
  roscpp
  pcl_ros
)

generate_dynamic_reconfigure_options(
  cfg/DirectStereoConfig.cfg
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

include_directories(include)

add_executable(manager_node src/manager_node.cpp src/manager.cpp src/state.cpp src/stereo_camera.cpp)

add_dependencies(manager_node ${PROJECT_NAME}_gencfg)

target_link_libraries(manager_node 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
)