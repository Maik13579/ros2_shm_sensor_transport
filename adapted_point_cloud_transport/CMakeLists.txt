cmake_minimum_required(VERSION 3.8)
project(adapted_point_cloud_transport)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/ros2_shm_target_dependencies.cmake")
find_package(point_cloud_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ros2_shm_target_dependencies(${PROJECT_NAME} INTERFACE
  point_cloud_transport
  rclcpp
  sensor_msgs
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_adapted_point_cloud_transport test/test_adapted_point_cloud_transport.cpp)
  if(TARGET test_adapted_point_cloud_transport)
    target_link_libraries(test_adapted_point_cloud_transport ${PROJECT_NAME})
    ros2_shm_target_dependencies(test_adapted_point_cloud_transport
      point_cloud_transport
      rclcpp
      sensor_msgs
    )
    set_tests_properties(test_adapted_point_cloud_transport PROPERTIES
      ENVIRONMENT "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_LOG_DIR=/tmp/adapted_point_cloud_transport_test_logs"
    )
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(point_cloud_transport rclcpp sensor_msgs)
ament_package()
