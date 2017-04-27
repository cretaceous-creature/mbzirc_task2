cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_task2_perception)


OPTION(WITH_CUDA "turn this option on to enable cuda support" OFF)
IF(WITH_CUDA)
 MESSAGE("Compiling with CUDA support")
 add_definitions(-DUSE_CUDA)
ENDIF(WITH_CUDA)

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

find_package(catkin REQUIRED COMPONENTS std_msgs tf tf_conversions eigen_conversions)

find_package(catkin REQUIRED
  jsk_perception
  jsk_pcl_ros
  opencv_apps
  dynamic_reconfigure
  jsk_topic_tools
  sensor_msgs
  roscpp
  message_generation
  image_transport
  image_geometry
  compressed_image_transport
  nodelet
)

find_package(Boost REQUIRED COMPONENTS filesystem system signals)

find_package(OpenCV 3 REQUIRED)
message("OPENCV ${OpenCV_VERSION} FOUND")

add_message_files(
  FILES
  Area.msg
  AreaArray.msg
  AreaArrayStamped.msg
)

include(FindPkgConfig)

pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)
if(${yaml_cpp_VERSION} VERSION_LESS "0.5.0")
## indigo yaml-cpp : 0.5.0 /  hydro yaml-cpp : 0.3.0
  add_definitions("-DUSE_OLD_YAML")
endif()

pkg_check_modules(libcmt REQUIRED libcmt)

generate_dynamic_reconfigure_options(
  cfg/InsigmaNormalizer.cfg
  cfg/MeanshiftFiltering.cfg
  cfg/TemplateMatchDetector.cfg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  tf
  tf_conversions
  eigen_conversions
  opencv_apps
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

macro(jsk_add_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  if(NOT EXISTS "${PROJECT_SOURCE_DIR}/test/${_single_nodelet_exec_name}.test")
    list(APPEND ${PROJECT_NAME}_nodes_without_test ${_nodelet_cpp})
  endif()
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    ${PROJECT_NAME}_nodelet_sources ${PROJECT_NAME}_nodelet_executables)
endmacro()

jsk_add_nodelet(src/luminance_homogenizer.cpp "mbzirc_task2_perception/LuminanceHomogenizer" "luminance_homogenizer")
jsk_add_nodelet(src/meanshift_filtering.cpp "mbzirc_task2_perception/MeanshiftFiltering" "meanshift_filtering")
jsk_add_nodelet(src/roi_generator.cpp "mbzirc_task2_perception/ROIGenerator" "roi_generator")
jsk_add_nodelet(src/bounding_box_publisher.cpp "mbzirc_task2_perception/BoundingBoxPublisher" "bounding_box_publisher")
jsk_add_nodelet(src/extract_panel_cluster.cpp "mbzirc_task2_perception/ExtractPanelCluster" "extract_panel_cluster")
jsk_add_nodelet(src/image_clipper.cpp "mbzirc_task2_perception/ImageClipper" "image_clipper")
jsk_add_nodelet(src/image_manual_clipper.cpp "mbzirc_task2_perception/ImageManualClipper" "image_manual_clipper")
jsk_add_nodelet(src/insigma_normalizer.cpp "mbzirc_task2_perception/InsigmaNormalizer" "insigma_normalizer")
jsk_add_nodelet(src/panel_3d_projector.cpp "mbzirc_task2_perception/Panel3DProjector" "panel_3d_projector")
jsk_add_nodelet(src/template_match_detector.cpp "mbzirc_task2_perception/TemplateMatchDetector" "template_match_detector")
jsk_add_nodelet(src/wrench_3d_projector.cpp "mbzirc_task2_perception/Wrench3DProjector" "wrench_3d_projector")

add_library(${PROJECT_NAME} SHARED ${mbzirc_task2_perception_nodelet_sources}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
