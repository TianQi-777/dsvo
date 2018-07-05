cmake_minimum_required(VERSION 2.8.3)
project(dsvo)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  dynamic_reconfigure
  roscpp
  pcl_ros
)

generate_dynamic_reconfigure_options(
  cfg/dsvo.cfg
)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( G2O REQUIRED )
find_package( Cholmod REQUIRED)
find_package( CSparse REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} )

catkin_package()

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include test/include)

add_executable(stereo_rectifier_node
  src/stereo_rectifier_node.cpp
  src/stereo_rectifier.cpp
)

target_link_libraries(stereo_rectifier_node
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
)

add_executable(stereo_processor_node
  src/stereo_processor/g2o_edges/scale_edge.cpp
  src/stereo_processor/g2o_edges/pose_edge.cpp
  test/src/odom_comparer.cpp
  test/src/point_comparer.cpp
  test/src/trans_comparer.cpp
  test/src/comparer.cpp
  src/state.cpp
  src/stereo_processor/stereo_processor.cpp
  src/stereo_processor/stereo_processor_helpers.cpp
  src/stereo_processor/stereo_processor_node.cpp
  src/stereo_processor/reconstructor.cpp
  src/stereo_processor/scale_optimizer.cpp
  src/stereo_processor/pose_estimater.cpp)

add_dependencies(stereo_processor_node ${PROJECT_NAME}_gencfg)

target_link_libraries(stereo_processor_node
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
  ${CSPARSE_LIBRARY}
  ${CHOLMOD_LIBRARIES}
  g2o_core g2o_types_slam3d g2o_solver_csparse csparse g2o_stuff g2o_csparse_extension g2o_types_sba cholmod
)

add_executable(zed_split src/tools/zed_split.cpp)

target_link_libraries(zed_split
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(kitti_pub src/tools/kitti_pub.cpp)

target_link_libraries(kitti_pub
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
