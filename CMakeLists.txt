cmake_minimum_required(VERSION 2.8.3)
project(direct_stereo)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11" )

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
find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( G2O REQUIRED )
find_package( Cholmod REQUIRED)
find_package( CSparse REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} )

catkin_package()

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)

add_executable(manager_node src/manager_node.cpp src/manager.cpp src/state.cpp src/comparer.cpp src/stereo_camera/stereo_camera.cpp src/stereo_camera/reconstructor.cpp src/stereo_camera/scale_edge.cpp src/stereo_camera/scale_optimizer.cpp src/stereo_camera/local_KF_optimizer.cpp src/stereo_camera/pose_edge.cpp src/stereo_camera/pose_estimater.cpp)

add_dependencies(manager_node ${PROJECT_NAME}_gencfg)

target_link_libraries(manager_node 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
  ${CSPARSE_LIBRARY}
  ${CHOLMOD_LIBRARIES}
  g2o_core g2o_types_slam3d g2o_solver_csparse csparse g2o_stuff g2o_csparse_extension g2o_types_sba cholmod
)