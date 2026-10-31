cmake_minimum_required(VERSION 3.10)
project(vitulus_depth_obstacles)

# One package for ROS 1 (catkin) and ROS 2 (ament): the setup.bash of the distribution sets ROS_VERSION.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(PCL REQUIRED COMPONENTS common filters features search kdtree segmentation)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

if("$ENV{ROS_VERSION}" STREQUAL "1")
  # ------------------------------------------------------------------------------------------------- ROS 1
  find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs diagnostic_msgs std_srvs message_filters tf2
    tf2_ros tf2_eigen pcl_conversions)
  catkin_package()
  # ~set / ~config / the stored settings; libjsoncpp-dev comes with libpcl-dev (libvtk7-dev depends on it)
  find_path(JSONCPP_INCLUDE_DIR json/json.h PATH_SUFFIXES jsoncpp)
  find_library(JSONCPP_LIBRARY jsoncpp)
  if(NOT JSONCPP_INCLUDE_DIR OR NOT JSONCPP_LIBRARY)
    message(FATAL_ERROR "libjsoncpp-dev not found")
  endif()
  include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${JSONCPP_INCLUDE_DIR})

  add_library(depth_obstacles_core src/segmentation.cpp src/live_config.cpp)
  target_link_libraries(depth_obstacles_core ${PCL_LIBRARIES})
  if(OpenMP_CXX_FOUND)
    target_link_libraries(depth_obstacles_core OpenMP::OpenMP_CXX)
  endif()

  add_executable(depth_obstacles src/ros1_node.cpp)
  target_link_libraries(depth_obstacles depth_obstacles_core ${catkin_LIBRARIES} ${JSONCPP_LIBRARY})

  install(TARGETS depth_obstacles depth_obstacles_core
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(FILES launch/depth_obstacles.launch launch/rtabmap_reference.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(FILES config/d435_ros1.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
  catkin_install_python(PROGRAMS tools/ab_ros1.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_segmentation test/test_segmentation.cpp)
    target_link_libraries(test_segmentation depth_obstacles_core gtest_main)
    # the node on its own roscore (port 11820): pytest test/test_node.py with this workspace sourced
  endif()

else()
  # ------------------------------------------------------------------------------------------------- ROS 2
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_eigen REQUIRED)
  find_package(pcl_conversions REQUIRED)

  add_library(depth_obstacles_core SHARED src/segmentation.cpp src/live_config.cpp)
  target_include_directories(depth_obstacles_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include> ${PCL_INCLUDE_DIRS})
  target_link_libraries(depth_obstacles_core ${PCL_LIBRARIES} Eigen3::Eigen)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(depth_obstacles_core OpenMP::OpenMP_CXX)
  endif()

  add_library(depth_obstacles_component SHARED src/ros2_node.cpp)
  target_link_libraries(depth_obstacles_component depth_obstacles_core)
  ament_target_dependencies(depth_obstacles_component rclcpp rclcpp_components sensor_msgs diagnostic_msgs
    message_filters tf2 tf2_ros tf2_eigen pcl_conversions)
  rclcpp_components_register_node(depth_obstacles_component
    PLUGIN "vitulus_depth_obstacles::DepthObstaclesNode" EXECUTABLE depth_obstacles)

  install(TARGETS depth_obstacles_core depth_obstacles_component
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  install(DIRECTORY include/ DESTINATION include)
  install(FILES launch/depth_obstacles.launch.py launch/rtabmap_reference.launch.py
    DESTINATION share/${PROJECT_NAME}/launch)
  install(FILES config/d435.yaml config/depth_obstacles.rviz DESTINATION share/${PROJECT_NAME}/config)
  install(PROGRAMS tools/ab_rtabmap.py DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_gtest(test_segmentation test/test_segmentation.cpp)
    target_link_libraries(test_segmentation depth_obstacles_core)
    ament_add_pytest_test(test_node test/test_node.py TIMEOUT 180)
  endif()

  ament_package()
endif()
