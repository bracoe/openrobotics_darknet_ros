cmake_minimum_required(VERSION 3.10)
project(openrobotics_darknet_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_library(Darknet darknet REQUIRED)
find_library(Darkhelp darkhelp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_transport REQUIRED)

add_library(detector_node SHARED
  src/detector_node.cpp)
target_include_directories(detector_node PUBLIC include)
target_compile_definitions(detector_node PRIVATE "DARKNET_ROS_NODE_BUILDING_DLL")
ament_target_dependencies(detector_node PUBLIC
  "rclcpp"
  "rclcpp_components"
  cv_bridge
  sensor_msgs
  vision_msgs
  image_transport)
target_link_libraries(detector_node PUBLIC "${Darknet}" "${Darkhelp}")
rclcpp_components_register_nodes(detector_node "openrobotics::darknet_ros::DetectorNode")

add_executable(detector_node_main
  src/detector_node_main.cpp)
target_link_libraries(detector_node_main detector_node)
set_target_properties(detector_node_main PROPERTIES OUTPUT_NAME "detector_node")

ament_export_libraries(detector_node)

install(TARGETS detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS detector_node_main
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/ launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION include)

ament_package()
