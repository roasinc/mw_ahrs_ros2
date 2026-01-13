cmake_minimum_required(VERSION 3.8)

project(mw_ahrs_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(
  DEPENDENCIES
  rclcpp
  tf2
  realtime_tools
  std_srvs
  geometry_msgs
  sensor_msgs
)

find_package(PkgConfig)
pkg_check_modules(LIBSERIAL libserial)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(
  mw_ahrs_driver_node
  src/mw_ahrs_driver.cpp
)
target_include_directories(mw_ahrs_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS}
)
target_link_libraries(mw_ahrs_driver_node ${LIBSERIAL_LIBRARIES})
ament_target_dependencies(mw_ahrs_driver_node ${DEPENDENCIES})

install(
  DIRECTORY launch rules
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS mw_ahrs_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

ament_package()