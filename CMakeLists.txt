cmake_minimum_required(VERSION 3.22)
project(pc_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(radar_interface REQUIRED)
find_package(Boost COMPONENTS date_time filesystem iostreams REQUIRED)

# pc_record_pub
add_executable(pc_record_pub)
ament_target_dependencies(pc_record_pub
  rclcpp
  std_msgs
  sensor_msgs
  radar_interface
  Boost
)

target_sources(pc_record_pub PRIVATE src/RecordPub.cpp)
target_include_directories(pc_record_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pc_record_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS pc_record_pub
  DESTINATION lib/${PROJECT_NAME})

# pc_lidar_pub
add_executable(pc_lidar_pub)
ament_target_dependencies(pc_lidar_pub
  rclcpp
  std_msgs
  sensor_msgs
  radar_interface
  Boost
)

target_sources(pc_lidar_pub PRIVATE src/LidarPub.cpp)
target_include_directories(pc_lidar_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pc_lidar_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS pc_lidar_pub
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# pc_publisher header-only library
add_library(pc_publisher INTERFACE)
target_include_directories(pc_publisher INTERFACE
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
"$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(pc_publisher HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs sensor_msgs radar_interface Boost)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
