# Copyright 2026 Maik Knof
#
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
project(shm_sensor_transport_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/ros2_shm_target_dependencies.cmake")
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(benchmark_image_publisher_component SHARED
  src/benchmark_image_publisher_component.cpp
  src/register_components.cpp
)
target_compile_features(benchmark_image_publisher_component PUBLIC cxx_std_17)
target_include_directories(benchmark_image_publisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ros2_shm_target_dependencies(benchmark_image_publisher_component
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_nodes(benchmark_image_publisher_component
  "shm_sensor_transport_test::BenchmarkImagePublisherComponent"
)

add_executable(benchmark_image_publisher src/benchmark_image_publisher_main.cpp)
target_compile_features(benchmark_image_publisher PUBLIC cxx_std_17)
target_link_libraries(benchmark_image_publisher benchmark_image_publisher_component)
ros2_shm_target_dependencies(benchmark_image_publisher rclcpp)

install(TARGETS benchmark_image_publisher benchmark_image_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(PROGRAMS scripts/benchmark_image_transport
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_image_pipeline_launch test/test_image_pipeline_launch.py
    ENV
      RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      ROS_LOG_DIR=/tmp/shm_sensor_transport_test_logs
    TIMEOUT 60
  )
endif()

ament_package()
