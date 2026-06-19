# Copyright 2026 Maik Knof
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(shm_sensor_transport)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shm_sensor_transport_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/parameter_utils.cpp
  src/qos_utils.cpp
  src/shm_handle.cpp
  src/shm_name.cpp
  src/shm_ring_buffer.cpp
  src/shm_subscriber.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  shm_sensor_transport_interfaces
)

add_library(${PROJECT_NAME}_components SHARED
  src/shm_compressed_image_relay_component.cpp
  src/shm_image_relay_component.cpp
  src/shm_point_cloud2_relay_component.cpp
  src/register_components.cpp
)
target_compile_features(${PROJECT_NAME}_components PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_components ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_components
  sensor_msgs
  shm_sensor_transport_interfaces
)

rclcpp_components_register_nodes(${PROJECT_NAME}_components
  "shm_sensor_transport::ShmCompressedImageRelayComponent"
  "shm_sensor_transport::ShmImageRelayComponent"
  "shm_sensor_transport::ShmPointCloud2RelayComponent"
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_components
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_shm_name test/test_shm_name.cpp)
  target_link_libraries(test_shm_name ${PROJECT_NAME})

  ament_add_gtest(test_shm_ring_buffer test/test_shm_ring_buffer.cpp)
  target_link_libraries(test_shm_ring_buffer ${PROJECT_NAME})

  ament_add_gtest(test_shm_handle test/test_shm_handle.cpp)
  target_link_libraries(test_shm_handle ${PROJECT_NAME})

  ament_add_gtest(test_shm_subscriber test/test_shm_subscriber.cpp)
  target_link_libraries(test_shm_subscriber ${PROJECT_NAME})

  ament_add_gtest(test_shm_publisher test/test_shm_publisher.cpp)
  target_link_libraries(test_shm_publisher ${PROJECT_NAME} ${PROJECT_NAME}_components)
  set_tests_properties(test_shm_publisher PROPERTIES
    ENVIRONMENT "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_LOG_DIR=/tmp/shm_sensor_transport_test_logs"
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs shm_sensor_transport_interfaces)
ament_package()
