cmake_minimum_required(VERSION 3.8)
project(image_transport_shm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_target_dependencies REQUIRED)
include("${ament_cmake_target_dependencies_DIR}/ament_target_dependencies.cmake")
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shm_sensor_transport REQUIRED)
find_package(shm_sensor_transport_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/shm_plugins.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
  image_transport
  pluginlib
  rclcpp
  sensor_msgs
  shm_sensor_transport
  shm_sensor_transport_interfaces
)

pluginlib_export_plugin_description_file(image_transport plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_image_transport_shm test/test_image_transport_shm.cpp)
  if(TARGET test_image_transport_shm)
    target_link_libraries(test_image_transport_shm ${PROJECT_NAME})
    ament_target_dependencies(test_image_transport_shm
      image_transport
      pluginlib
      rclcpp
      sensor_msgs
      shm_sensor_transport
      shm_sensor_transport_interfaces
    )
    set_tests_properties(test_image_transport_shm PROPERTIES
      ENVIRONMENT "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_LOG_DIR=/tmp/image_transport_shm_test_logs"
    )
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  image_transport
  pluginlib
  rclcpp
  sensor_msgs
  shm_sensor_transport
  shm_sensor_transport_interfaces
)
ament_package()
