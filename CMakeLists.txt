cmake_minimum_required(VERSION 3.16)
project(raydar)

# --- Find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gz-sim7 REQUIRED)
find_package(gz-utils2 REQUIRED)

set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

# --- Generate ROS 2 messages ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RadarTarget.msg"
  "msg/RadarTargetArray.msg"
  DEPENDENCIES std_msgs
)

# --- Build the Gazebo plugin library ---
add_library(RadarSystem SHARED 
  src/RadarSys.cc
  src/RadarRayCaster.cc
)

target_link_libraries(RadarSystem
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  rclcpp::rclcpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  ${visualization_msgs_TARGETS}   
  atomic
)

# Make the plugin's headers see the generated message headers
target_include_directories(RadarSystem PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

target_include_directories(RadarSystem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

target_link_libraries(RadarSystem
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-rendering7::gz-rendering7
  gz-utils2::gz-utils2     
  rclcpp::rclcpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  atomic
)

# Ensure the messages are generated before the plugin is compiled
add_dependencies(RadarSystem ${PROJECT_NAME}__rosidl_generator_cpp)

# --- Install ---
install(TARGETS RadarSystem
  DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
