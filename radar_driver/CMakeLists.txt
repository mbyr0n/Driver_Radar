cmake_minimum_required(VERSION 3.8)
project(radar_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorCfg.msg"
  "msg/EthCfg.msg"
  "msg/RadarPacket.msg"
  "msg/SensorStatus.msg"
  "msg/RadarDetection.msg"
  "msg/BoundingBoxList.msg"
  "msg/BoundingBox.msg"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include)

set(extra_LIBRARIES pcap pthread)
set(RADAR_UNFILTER_SOURCE src/UnfilteredMain.cpp src/parser.cpp src/sniffer.cpp)
set(RADAR_FILTER_SOURCE src/radarPublisher.cpp src/processPacket.cpp src/FilteredMain.cpp)
set(VIS_SOURCE src/PointCloudConverter.cpp)

add_executable(radar_publisher ${RADAR_UNFILTER_SOURCE})
ament_target_dependencies(radar_publisher rclcpp std_msgs sensor_msgs)
target_link_libraries(radar_publisher ${extra_LIBRARIES} "${cpp_typesupport_target}")

add_executable(radar_processor ${RADAR_FILTER_SOURCE})
ament_target_dependencies(radar_processor rclcpp std_msgs sensor_msgs)
target_link_libraries(radar_processor "${cpp_typesupport_target}")

add_executable(radar_visualizer ${VIS_SOURCE})
ament_target_dependencies(radar_visualizer rclcpp std_msgs sensor_msgs visualization_msgs geometry_msgs)
target_link_libraries(radar_visualizer "${cpp_typesupport_target}")

install(TARGETS
  radar_publisher
  radar_processor
  radar_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
