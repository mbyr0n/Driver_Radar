# ROS 2 Jazzy Phase 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the minimum functional radar pipeline (`radar_publisher` -> `radar_processor` -> `radar_visualizer`) from ROS 1/Noetic to ROS 2 Jazzy.

**Architecture:** Convert `radar_driver` into an `ament_cmake` package, generate the existing custom messages with ROS 2 interfaces, and migrate the three C++ nodes to `rclcpp`. Keep `libpcap` and the existing CLI behavior for Phase 1, while excluding clustering and multi-radar executables from the build.

**Tech Stack:** ROS 2 Jazzy, `ament_cmake`, `rclcpp`, `rosidl_default_generators`, `sensor_msgs`, `pcl_conversions`, `libpcap`, C++17, Python launch files, `pytest` static regression tests.

---

## File Structure

- Modify `radar_driver/CMakeLists.txt`: replace catkin build with ament build, generate messages, build/install Phase 1 targets only.
- Modify `radar_driver/package.xml`: replace ROS 1 dependencies with ROS 2 dependencies.
- Modify `radar_driver/msg/*.msg`: adjust message dependency syntax only if ROS 2 interface generation rejects current files.
- Modify `radar_driver/include/parser.h` and `radar_driver/src/parser.cpp`: switch ROS includes, message types, publisher type, logging, and published message field names.
- Modify `radar_driver/include/sniffer.h` and `radar_driver/src/sniffer.cpp`: keep libpcap capture logic, compile under ROS 2/C++17.
- Modify `radar_driver/include/processPacket.h` and `radar_driver/src/processPacket.cpp`: switch message types and ROS logging.
- Modify `radar_driver/include/radarPublisher.h` and `radar_driver/src/radarPublisher.cpp`: replace `ros::Publisher` with `rclcpp::Publisher`.
- Modify `radar_driver/src/UnfilteredMain.cpp`: convert `radar_publisher` node to `rclcpp`.
- Modify `radar_driver/src/FilteredMain.cpp`: convert `radar_processor` node to `rclcpp`.
- Modify `radar_driver/src/PointCloudConverter.cpp`: convert `radar_visualizer` node to `rclcpp` and ROS 2 PointCloud2 message names.
- Create `radar_driver/launch/single_radar.launch.py`: ROS 2 launch file for the three Phase 1 nodes.
- Modify `radar_driver/tests/test_critical_regressions.py`: keep static bug regression checks and add ROS 2 migration checks as needed.

---

### Task 1: Convert Build Metadata To ROS 2

**Files:**
- Modify: `radar_driver/CMakeLists.txt`
- Modify: `radar_driver/package.xml`
- Test: `radar_driver/tests/test_critical_regressions.py`

- [ ] **Step 1: Add failing static migration checks**

Add these tests to `radar_driver/tests/test_critical_regressions.py`:

```python
def test_build_metadata_uses_ros2_ament():
    cmake = read("CMakeLists.txt")
    package = read("package.xml")

    assert "find_package(ament_cmake REQUIRED)" in cmake
    assert "rosidl_generate_interfaces" in cmake
    assert "ament_package()" in cmake
    assert "catkin_package" not in cmake
    assert "<buildtool_depend>ament_cmake</buildtool_depend>" in package
    assert "<buildtool_depend>catkin</buildtool_depend>" not in package
    assert "<depend>rclcpp</depend>" in package
    assert "rosidl_default_generators" in package
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py::test_build_metadata_uses_ros2_ament -v
```

Expected: FAIL because `CMakeLists.txt` and `package.xml` are still ROS 1/catkin.

- [ ] **Step 3: Replace `radar_driver/CMakeLists.txt` with ROS 2 Phase 1 build**

Use this structure:

```cmake
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
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
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
ament_target_dependencies(radar_visualizer rclcpp std_msgs sensor_msgs pcl_conversions pcl_ros)
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
```

- [ ] **Step 4: Replace `radar_driver/package.xml` with ROS 2 metadata**

Use package format 3 and include these dependencies:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>radar_driver</name>
  <version>0.0.1</version>
  <description>The ROS 2 publisher package for the ARS430 Continental Radars</description>
  <maintainer email="watouser@todo.todo">watouser</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>pcl_conversions</depend>
  <depend>pcl_ros</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 5: Run static test**

Run:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py::test_build_metadata_uses_ros2_ament -v
```

Expected: PASS.

---

### Task 2: Migrate Message Includes And Field Names In Core Parser

**Files:**
- Modify: `radar_driver/include/parser.h`
- Modify: `radar_driver/src/parser.cpp`
- Test: `radar_driver/tests/test_critical_regressions.py`

- [ ] **Step 1: Add failing static parser migration test**

Add:

```python
def test_parser_uses_ros2_messages_and_rclcpp():
    parser_h = read("include/parser.h")
    parser_cpp = read("src/parser.cpp")

    assert "#include \"rclcpp/rclcpp.hpp\"" in parser_h
    assert "radar_driver/msg/radar_packet.hpp" in parser_h
    assert "radar_driver::msg::RadarPacket" in parser_cpp
    assert "rclcpp::Publisher<radar_driver::msg::RadarPacket>::SharedPtr" in parser_cpp
    assert "radar_driver/RadarPacket.h" not in parser_cpp
    assert "radar_driver::RadarPacket" not in parser_cpp
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py::test_parser_uses_ros2_messages_and_rclcpp -v
```

Expected: FAIL because parser still uses ROS 1 headers/types.

- [ ] **Step 3: Update parser includes/types**

In `parser.h`, replace ROS 1 includes with:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "radar_driver/msg/radar_detection.hpp"
#include "radar_driver/msg/radar_packet.hpp"
#include "radar_driver/msg/sensor_status.hpp"
```

Change declarations to:

```cpp
void initUnfilteredPublisher(rclcpp::Node::SharedPtr nh_new);
void loadPacketMsg(RDIPacket_t * packet, radar_driver::msg::RadarPacket * msg);
void loadSSMessage(SSPacket_t* packet, radar_driver::msg::SensorStatus* msg);
```

In `parser.cpp`, use:

```cpp
static rclcpp::Publisher<radar_driver::msg::RadarPacket>::SharedPtr unfilteredPublisher;

void initUnfilteredPublisher(rclcpp::Node::SharedPtr nh) {
    unfilteredPublisher = nh->create_publisher<radar_driver::msg::RadarPacket>("unfiltered_radar_packet", 50);
}
```

Update message variables:

```cpp
radar_driver::msg::RadarPacket msg;
radar_driver::msg::RadarDetection data;
radar_driver::msg::SensorStatus
```

Update fields to ROS 2 generated snake_case:

```cpp
msg->event_id
msg->time_stamp
msg->measurement_counter
msg->vambig
msg->center_frequency
msg->detections
data.pdh0
data.az_ang0
data.rcs0
data.az_ang_var0
data.prob0
data.az_ang1
data.rcs1
data.az_ang_var1
data.prob1
data.vrel_rad
data.el_ang
data.range_var
data.vrel_rad_var
data.el_ang_var
data.snr
data.range
data.pos_x
data.pos_y
data.pos_z
```

- [ ] **Step 4: Run parser static test**

Run:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py::test_parser_uses_ros2_messages_and_rclcpp -v
```

Expected: PASS.

---

### Task 3: Migrate Processor And Filtered Publisher

**Files:**
- Modify: `radar_driver/include/processPacket.h`
- Modify: `radar_driver/src/processPacket.cpp`
- Modify: `radar_driver/include/radarPublisher.h`
- Modify: `radar_driver/src/radarPublisher.cpp`
- Modify: `radar_driver/src/FilteredMain.cpp`
- Test: `radar_driver/tests/test_critical_regressions.py`

- [ ] **Step 1: Add failing static processor migration test**

Add:

```python
def test_processor_uses_ros2_rclcpp_types():
    filtered = read("src/FilteredMain.cpp")
    publisher_h = read("include/radarPublisher.h")
    processor_h = read("include/processPacket.h")

    assert "#include <rclcpp/rclcpp.hpp>" in filtered
    assert "rclcpp::Subscription<radar_driver::msg::RadarPacket>::SharedPtr" in filtered
    assert "rclcpp::Publisher<radar_driver::msg::RadarPacket>::SharedPtr" in publisher_h
    assert "radar_driver::msg::RadarPacket::SharedPtr" in processor_h
    assert "ros::" not in filtered
    assert "ros::" not in publisher_h
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py::test_processor_uses_ros2_rclcpp_types -v
```

Expected: FAIL because processor still uses ROS 1.

- [ ] **Step 3: Update processor message signatures**

Change `processPacket.h`:

```cpp
uint8_t processRDIMsg(const radar_driver::msg::RadarPacket::SharedPtr packet);
```

Change `processPacket.cpp` helper signature:

```cpp
bool loadRDIMessageFromPacket(radar_driver::msg::RadarPacket* newMsg, const radar_driver::msg::RadarPacket::SharedPtr oldMsg);
```

Update message field names from ROS 1 style to ROS 2 snake_case throughout `processPacket.cpp`, including:

```cpp
packet->detections
packet->event_id
packet->time_stamp
oldMsg->detections[i].snr
oldMsg->detections[i].vrel_rad
oldMsg->detections[i].pos_x
```

- [ ] **Step 4: Update `RadarPublisher` to ROS 2**

In `radarPublisher.h`, use:

```cpp
#include <rclcpp/rclcpp.hpp>
#include "radar_driver/msg/radar_packet.hpp"
#include "radar_driver/msg/radar_detection.hpp"

rclcpp::Node::SharedPtr nh_;
rclcpp::Publisher<radar_driver::msg::RadarPacket>::SharedPtr packet_pub_;
rclcpp::Publisher<radar_driver::msg::RadarDetection>::SharedPtr detection_pub_;
RadarPublisher(rclcpp::Node::SharedPtr new_nh_);
```

In `radarPublisher.cpp`, create publisher with:

```cpp
packet_pub_ = nh_->create_publisher<radar_driver::msg::RadarPacket>("filtered_radar_packet", 50);
```

- [ ] **Step 5: Update `FilteredMain.cpp` to ROS 2**

Use:

```cpp
rclcpp::init(argc, argv);
auto node = std::make_shared<rclcpp::Node>("radar_processor");
RadarPublisher rp(node);
auto radarUnfilteredSub = node->create_subscription<radar_driver::msg::RadarPacket>(
  unfiltered_topic,
  100,
  radarCallback
);
rclcpp::spin(node);
rclcpp::shutdown();
```

Update logging to `RCLCPP_*`.

- [ ] **Step 6: Run processor static test**

Run:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py::test_processor_uses_ros2_rclcpp_types -v
```

Expected: PASS.

---

### Task 4: Migrate Unfiltered Publisher Node And Sniffer

**Files:**
- Modify: `radar_driver/src/UnfilteredMain.cpp`
- Modify: `radar_driver/src/sniffer.cpp`
- Modify: `radar_driver/include/sniffer.h`
- Test: `radar_driver/tests/test_critical_regressions.py`

- [ ] **Step 1: Add failing static publisher-node migration test**

Add:

```python
def test_unfiltered_main_uses_rclcpp():
    main = read("src/UnfilteredMain.cpp")

    assert "#include <rclcpp/rclcpp.hpp>" in main
    assert "rclcpp::init(argc, argv);" in main
    assert "std::make_shared<rclcpp::Node>(\"radar_publisher\")" in main
    assert "rclcpp::shutdown();" in main
    assert "ros::" not in main
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py::test_unfiltered_main_uses_rclcpp -v
```

Expected: FAIL because `UnfilteredMain.cpp` still uses ROS 1.

- [ ] **Step 3: Update `UnfilteredMain.cpp`**

Use `rclcpp::init`, create a node, pass it to `initUnfilteredPublisher(node)`, run `run(...)`, then call `rclcpp::shutdown()`.

Keep `getopt()` CLI parsing unchanged except for ROS 2 logging.

- [ ] **Step 4: Keep sniffer ROS-neutral**

Ensure `sniffer.cpp` does not use ROS 1 APIs. Its includes can remain C/libpcap plus `processPacket.h` if required by constants, but it must compile with C++17.

- [ ] **Step 5: Run publisher-node static test**

Run:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py::test_unfiltered_main_uses_rclcpp -v
```

Expected: PASS.

---

### Task 5: Migrate Visualizer Node

**Files:**
- Modify: `radar_driver/src/PointCloudConverter.cpp`
- Test: `radar_driver/tests/test_critical_regressions.py`

- [ ] **Step 1: Add failing static visualizer migration test**

Add:

```python
def test_visualizer_uses_ros2_messages():
    visualizer = read("src/PointCloudConverter.cpp")

    assert "#include <rclcpp/rclcpp.hpp>" in visualizer
    assert "sensor_msgs::msg::PointCloud2" in visualizer
    assert "radar_driver::msg::RadarPacket" in visualizer
    assert "rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr" in visualizer
    assert "ros::" not in visualizer
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py::test_visualizer_uses_ros2_messages -v
```

Expected: FAIL because visualizer still uses ROS 1.

- [ ] **Step 3: Update `PointCloudConverter.cpp`**

Convert includes and types:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "radar_driver/msg/radar_packet.hpp"
```

Use:

```cpp
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
rclcpp::Subscription<radar_driver::msg::RadarPacket>::SharedPtr
```

Update field names to snake_case:

```cpp
packet->detections[i].pos_x
packet->detections[i].pos_y
packet->detections[i].pos_z
packet->header.stamp
```

- [ ] **Step 4: Run visualizer static test**

Run:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py::test_visualizer_uses_ros2_messages -v
```

Expected: PASS.

---

### Task 6: Add ROS 2 Single Radar Launch File

**Files:**
- Create: `radar_driver/launch/single_radar.launch.py`
- Test: `radar_driver/tests/test_critical_regressions.py`

- [ ] **Step 1: Add failing launch-file test**

Add:

```python
def test_ros2_single_radar_launch_exists():
    launch = read("launch/single_radar.launch.py")

    assert "from launch import LaunchDescription" in launch
    assert "from launch_ros.actions import Node" in launch
    assert "executable='radar_publisher'" in launch
    assert "executable='radar_processor'" in launch
    assert "executable='radar_visualizer'" in launch
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py::test_ros2_single_radar_launch_exists -v
```

Expected: FAIL because `single_radar.launch.py` does not exist.

- [ ] **Step 3: Create `single_radar.launch.py`**

Create a ROS 2 launch file with `DeclareLaunchArgument`, `LaunchConfiguration`, and three `Node` actions. Preserve launch arguments `name`, `iface`, `port`, `filter`, `capture`, `cpath`, `scanMode`, and `uTopic`.

- [ ] **Step 4: Run launch-file static test**

Run:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py::test_ros2_single_radar_launch_exists -v
```

Expected: PASS.

---

### Task 7: Run ROS 2 Build And Fix Compile Errors

**Files:**
- Modify files from previous tasks as required by compiler errors.
- Test: full package build/test.

- [ ] **Step 1: Run static regression tests**

Run:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py
```

Expected: PASS.

- [ ] **Step 2: Run ROS 2 build**

Run:

```bash
colcon build --packages-select radar_driver
```

Expected: build starts. If it fails, read the first compiler/interface error and fix only that root cause before re-running.

- [ ] **Step 3: Run ROS 2 tests**

Run:

```bash
colcon test --packages-select radar_driver
colcon test-result --verbose
```

Expected: no package-level failures.

- [ ] **Step 4: Verify interfaces**

Run:

```bash
source install/setup.bash
ros2 interface show radar_driver/msg/RadarPacket
ros2 interface show radar_driver/msg/RadarDetection
```

Expected: both commands print interface definitions.

- [ ] **Step 5: Verify executable discovery**

Run:

```bash
source install/setup.bash
ros2 pkg executables radar_driver
```

Expected: output includes `radar_publisher`, `radar_processor`, and `radar_visualizer`.

---

## Self-Review

- Spec coverage: The plan covers build metadata, message generation, publisher, processor, visualizer, launch, and ROS 2 verification from the approved Phase 1 spec.
- Placeholder scan: No `TBD`, incomplete task, or unspecified file path is intentionally left.
- Type consistency: ROS 2 message types consistently use `radar_driver::msg::*`; publisher/subscription types consistently use `rclcpp::*::SharedPtr`.
- Scope check: Clustering, multi-radar, ROS bag utilities, scripts, and RViz config migration remain out of scope except for build installation of existing directories.
