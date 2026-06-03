# ROS 2 Jazzy Phase 1 Migration Design

**Goal:** Migrate the minimum functional radar pipeline from ROS Noetic/ROS 1 to ROS 2 Jazzy without using `ros1_bridge`.

**Scope:** Phase 1 includes only `radar_publisher`, `radar_processor`, `radar_visualizer`, custom messages, and a single-radar launch file.

**Out of Scope:** `radar_clustering`, `multi_radar`, `radar_points`, shell scripts, RViz configs, rosbag utilities, and Python helper scripts are not migrated in Phase 1 unless they block the package build.

---

## Current State

The repository contains a ROS 1 catkin package at `radar_driver/`. It currently builds nodes around these responsibilities:

- `radar_publisher`: captures UDP radar packets with `libpcap`, parses them, and publishes `unfiltered_radar_packet`.
- `radar_processor`: subscribes to `unfiltered_radar_packet`, filters/groups detections, and publishes `filtered_radar_packet`.
- `radar_visualizer`: converts radar packets into `sensor_msgs/PointCloud2` for visualization.
- Clustering and multi-radar code exists but is incomplete or depends on packages that are not part of the minimum pipeline.

Critical parser and processor bugs were fixed before migration so the logic being ported is safer: publication is active, packet sizes are checked, timestamps update, packet group writes are bounded, bit masks use bitwise checks, and cluster math avoids uninitialized values.

---

## Migration Strategy

Use a clean ROS 2 `ament_cmake` package while preserving the existing source layout as much as possible. The migration should make the smallest practical changes needed to compile and run the Phase 1 pipeline in Jazzy.

The package remains named `radar_driver`.

The message definitions remain in `radar_driver/msg/` and are generated with `rosidl_generate_interfaces()`.

The C++ nodes use `rclcpp` and ROS 2 message namespaces such as `radar_driver::msg::RadarPacket`.

The packet capture code continues to use `libpcap`.

---

## Build System

Replace the ROS 1 catkin build with ROS 2 `ament_cmake`.

`radar_driver/CMakeLists.txt` should:

- Require a ROS 2 compatible CMake version.
- Find `ament_cmake`, `rclcpp`, `std_msgs`, `sensor_msgs`, `visualization_msgs`, `geometry_msgs`, `pcl_conversions`, `pcl_ros`, and `rosidl_default_generators` as needed by Phase 1.
- Generate interfaces from all existing `.msg` files.
- Build only Phase 1 executables:
  - `radar_publisher`
  - `radar_processor`
  - `radar_visualizer`
- Link `radar_publisher` with `pcap` and `pthread`.
- Install targets and launch files.
- Call `ament_package()`.

`radar_driver/package.xml` should:

- Use package format 3.
- Replace `catkin`, `roscpp`, `rospy`, `message_generation`, and `message_runtime` with ROS 2 dependencies.
- Declare message generation/runtime dependencies.
- Declare runtime/build dependencies required by Phase 1.
- Include `member_of_group` for ROS interface packages.

---

## Message Migration

All current message files stay in place:

- `msg/BoundingBox.msg`
- `msg/BoundingBoxList.msg`
- `msg/EthCfg.msg`
- `msg/RadarDetection.msg`
- `msg/RadarPacket.msg`
- `msg/SensorCfg.msg`
- `msg/SensorStatus.msg`

Phase 1 only uses radar packet/detection messages directly, but generating all package messages avoids partial interface surprises.

ROS 2 generated C++ includes use snake_case filenames, for example:

- `radar_driver/msg/radar_packet.hpp`
- `radar_driver/msg/radar_detection.hpp`
- `radar_driver/msg/sensor_status.hpp`

Generated C++ message types use the `radar_driver::msg` namespace.

---

## Node Migration

### radar_publisher

Files:

- `src/UnfilteredMain.cpp`
- `src/parser.cpp`
- `src/sniffer.cpp`
- `include/parser.h`
- `include/sniffer.h`

Responsibilities:

- Initialize a ROS 2 node named `radar_publisher`.
- Parse existing CLI arguments for interface, port, capture mode, capture path, and optional packet filter.
- Initialize the unfiltered publisher on topic `unfiltered_radar_packet`.
- Run `libpcap` capture and publish parsed radar packets.

Design notes:

- Keep the existing CLI behavior for compatibility with launch arguments.
- Use a shared/global ROS 2 publisher only as a minimal migration step if it avoids a larger parser refactor.
- Do not introduce ROS 2 parameters in Phase 1 unless required for build or launch.

### radar_processor

Files:

- `src/FilteredMain.cpp`
- `src/processPacket.cpp`
- `src/radarPublisher.cpp`
- `include/processPacket.h`
- `include/radarPublisher.h`

Responsibilities:

- Initialize a ROS 2 node named `radar_processor`.
- Subscribe to `unfiltered_radar_packet` or a CLI-provided topic.
- Filter and group radar detections using existing `PacketProcessor` logic.
- Publish filtered packets on `filtered_radar_packet`.

Design notes:

- Preserve current scan mode CLI behavior.
- Use `rclcpp::Subscription<radar_driver::msg::RadarPacket>::SharedPtr`.
- Use `rclcpp::Publisher<radar_driver::msg::RadarPacket>::SharedPtr`.

### radar_visualizer

Files:

- `src/PointCloudConverter.cpp`

Responsibilities:

- Initialize a ROS 2 node named `radar_visualizer`.
- Subscribe to a radar packet topic.
- Publish `sensor_msgs::msg::PointCloud2` on the existing point cloud topic naming pattern.

Design notes:

- Preserve CLI topic selection if practical.
- Keep point cloud conversion behavior unchanged unless ROS 2 API differences require small adjustments.

---

## Launch Migration

Create `radar_driver/launch/single_radar.launch.py`.

The launch file should start:

- `radar_publisher`
- `radar_processor`
- `radar_visualizer`

Launch arguments:

- `name`, default `radar`
- `iface`, default existing interface value if still useful
- `port`, default `31122`
- `filter`, default empty string
- `capture`, default `1`
- `cpath`, default `radarCap5.pcap`
- `scanMode`, default `0`
- `uTopic`, default `unfiltered_radar_packet`

The nodes should run under the `name` namespace to preserve the ROS 1 launch behavior.

---

## Error Handling

Keep current integer status codes for parser/processor logic during Phase 1.

ROS 2 logging should replace ROS 1 logging macros:

- `ROS_INFO` becomes `RCLCPP_INFO`.
- `ROS_WARN_STREAM` becomes `RCLCPP_WARN` or `RCLCPP_WARN_STREAM` if available.
- `ROS_ERROR_STREAM` becomes `RCLCPP_ERROR` or equivalent.

Packet parsing errors should continue to return status codes and skip malformed packets rather than throwing exceptions.

---

## Testing And Verification

Keep the existing static regression tests added for critical bug fixes:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py
```

After migration, the primary verification is:

```bash
colcon build --packages-select radar_driver
colcon test --packages-select radar_driver
colcon test-result --verbose
```

Interface verification:

```bash
source install/setup.bash
ros2 interface show radar_driver/msg/RadarPacket
ros2 interface show radar_driver/msg/RadarDetection
```

Runtime smoke checks:

```bash
ros2 run radar_driver radar_publisher --help
ros2 run radar_driver radar_processor --help
ros2 launch radar_driver single_radar.launch.py
```

If live radar hardware is unavailable, successful build plus CLI/help/launch startup checks are acceptable for Phase 1. Packet-level functional validation can be done later with real pcap data or synthetic packet tests.

---

## Risks And Constraints

- Message field naming may require broad C++ edits because ROS 2 generated C++ fields are lower snake_case.
- `std_msgs/Header` in `.msg` may need to be changed to `std_msgs/Header header` syntax if ROS 2 interface generation rejects the current form.
- Existing code uses global/static publishers in parser logic. This is acceptable for Phase 1 but can be refactored later.
- `PointCloudConverter.cpp` may need ROS 2-specific PointCloud2 iterator changes.
- The current environment has ROS 2 but not ROS 1, so Noetic/catkin build verification is intentionally skipped.
- Clustering remains out of scope because it depends on `jsk_recognition_msgs` and is not needed for the minimum pipeline.

---

## Acceptance Criteria

Phase 1 is complete when:

- `colcon build --packages-select radar_driver` succeeds.
- `colcon test --packages-select radar_driver` runs without package-level failures.
- ROS 2 interfaces for `RadarPacket` and `RadarDetection` are available.
- `radar_publisher`, `radar_processor`, and `radar_visualizer` executables are installed.
- `single_radar.launch.py` starts the three Phase 1 nodes.
- Static regression tests for the critical bug fixes pass.
