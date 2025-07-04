cmake_minimum_required(VERSION 3.8)
project(bumperbot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

pkg_check_modules(SERIAL libserial)

add_library(${PROJECT_NAME} SHARED
  src/bumperbot_interface.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  "${SERIAL_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME} "${SERIAL_LDFLAGS}")
ament_target_dependencies(${PROJECT_NAME} 
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_executable(simple_serial_transmitter src/simple_serial_transmitter.cpp)
ament_target_dependencies(simple_serial_transmitter rclcpp std_msgs)
target_include_directories(simple_serial_transmitter PUBLIC "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(simple_serial_transmitter "${SERIAL_LDFLAGS}")

add_executable(simple_serial_reciever src/simple_serial_reciever.cpp)
ament_target_dependencies(simple_serial_reciever rclcpp std_msgs)
target_include_directories(simple_serial_reciever PUBLIC "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(simple_serial_reciever "${SERIAL_LDFLAGS}")

pluginlib_export_plugin_description_file(hardware_interface bumperbot_interface.xml)

install(TARGETS 
  simple_serial_transmitter
  simple_serial_reciever
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/mpu6050_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()
