cmake_minimum_required(VERSION 3.8)
project(serial_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(library_name1 serial_comm)
set(library_name2 imu_serial_comm)
set(library_name3 serial_logger)
set(matplotlibcpp_INCLUDE_DIRS /usr/include/matplotlib-cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hw_msgs REQUIRED)
find_package(imu_msgs REQUIRED)
find_package(PythonLibs REQUIRED)

# add_library(
#   ${PROJECT_NAME}
#   SHARED
#   src/serial/serial_comm.cpp
#   src/serial/serial_driver.cpp
#   src/serial/serial_port.cpp
#   src/io_context/io_context.cpp
# )

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${matplotlibcpp_INCLUDE_DIRS})

include_directories(
  ${CMAKE_SOURCE_DIR}/include
)

add_executable(serial_comm 
  src/serial/serial_comm.cpp
  src/serial/serial_driver.cpp
  src/serial/serial_port.cpp
  src/io_context/io_context.cpp
)

add_executable(imu_serial_comm 
  src/serial/serial_driver.cpp
  src/serial/serial_port.cpp
  src/io_context/io_context.cpp
  src/serial/imu_serial_comm.cpp
)

add_executable(serial_logger 
  src/serial/serial_logger.cpp
)


# ament_target_dependencies(serial_node rclcpp std_msgs)
ament_target_dependencies(serial_comm rclcpp std_msgs hw_msgs imu_msgs)
ament_target_dependencies(imu_serial_comm rclcpp std_msgs imu_msgs)
ament_target_dependencies(serial_logger rclcpp std_msgs hw_msgs imu_msgs)

target_link_libraries(serial_logger ${PYTHON_LIBRARIES} ${matplotlibcpp_LIBRARIES})

install(TARGETS ${library_name1} ${library_name2} ${library_name3} 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install
install(TARGETS ${library_name1} ${library_name2} ${library_name3} 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
