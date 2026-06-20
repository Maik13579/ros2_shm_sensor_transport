cmake_minimum_required(VERSION 3.8)
project(shm_sensor_transport_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_target_dependencies REQUIRED)
include("${ament_cmake_target_dependencies_DIR}/ament_target_dependencies.cmake")
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shm_sensor_transport REQUIRED)
find_package(shm_sensor_transport_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/shm_sensor_displays.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rclcpp
  rviz_common
  rviz_default_plugins
  sensor_msgs
  shm_sensor_transport
  shm_sensor_transport_interfaces
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_pluginlib_loads test/test_pluginlib_loads.cpp)
  if(TARGET test_pluginlib_loads)
    ament_target_dependencies(test_pluginlib_loads
      pluginlib
      rviz_common
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  pluginlib
  rclcpp
  rviz_common
  rviz_default_plugins
  sensor_msgs
  shm_sensor_transport
  shm_sensor_transport_interfaces
)

ament_package()
