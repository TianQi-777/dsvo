cmake_minimum_required(VERSION 2.8.3)
project(direct_stereo)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_filters
  roscpp
  pcl_ros
)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
# FIND_PACKAGE( G2O REQUIRED )
# CSparse
# FIND_PACKAGE( CSparse REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

include_directories(include)

add_executable(manager_node src/manager_node.cpp src/manager.cpp src/state.cpp src/imu.cpp src/stereo_camera.cpp src/imu_math.cpp src/directEdge.cpp src/directSolver.cpp)
target_link_libraries(manager_node 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
   ${G2O_SOLVER_CSPARSE_EXTENSION}
   g2o_core g2o_types_slam3d g2o_solver_csparse csparse g2o_stuff g2o_csparse_extension g2o_types_sba
)