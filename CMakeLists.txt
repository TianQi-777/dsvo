cmake_minimum_required(VERSION 2.8.3)
project(direct_stereo)

set( CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -DG2O_BUILD_EXAMPLES=OFF" )

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

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O REQUIRED )
# CSparse
FIND_PACKAGE( CSparse REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

include_directories(include)

add_executable(manager_node src/manager_node.cpp src/manager.cpp src/state.cpp src/comparer.cpp src/stereo_camera/stereo_camera.cpp src/stereo_camera/reconstructor.cpp src/stereo_camera/optimizer.cpp src/stereo_camera/scale_edge.cpp)

add_dependencies(manager_node ${PROJECT_NAME}_gencfg)

target_link_libraries(manager_node 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${G2O_SOLVER_CSPARSE_EXTENSION}
   ${CSPARSE_LIBRARY}
   g2o_core g2o_types_slam3d g2o_solver_csparse csparse g2o_stuff g2o_csparse_extension g2o_types_sba cholmod
)