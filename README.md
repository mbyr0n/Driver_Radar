# Continental ARS430 Radar Driver for ROS 2 Jazzy

ROS 2 driver for Continental ARS430 automotive radars. This branch targets Ubuntu 24.04 and ROS 2 Jazzy.

The current supported pipeline is the single-radar path:

```text
radar_publisher -> radar_processor -> radar_visualizer
```

Multi-radar fusion, clustering executables, ROS 1 launch files, and older rosbag utilities are not part of the supported ROS 2 path yet.

## How The Driver Works

The radar sends UDP packets over Ethernet. The driver captures those packets with `libpcap`, parses the radar payload, publishes custom ROS 2 messages, filters/groups detections, and optionally converts the result to a `PointCloud2` for visualization.

Main nodes:

| Node | Input | Output | Purpose |
| --- | --- | --- | --- |
| `radar_publisher` | UDP packets from radar or `.pcap` file | `unfiltered_radar_packet` | Captures raw radar packets and converts them to `radar_driver/msg/RadarPacket`. |
| `radar_processor` | `unfiltered_radar_packet` | `filtered_radar_packet` | Groups packets by timestamp and applies basic filtering. |
| `radar_visualizer` | `filtered_radar_packet` | `radar_pointcloud`, `visualization_marker` | Converts radar detections to `sensor_msgs/msg/PointCloud2` and publishes FOV markers. |

Generated custom messages include:

- `radar_driver/msg/RadarPacket`
- `radar_driver/msg/RadarDetection`
- `radar_driver/msg/SensorStatus`
- `radar_driver/msg/BoundingBox`
- `radar_driver/msg/BoundingBoxList`
- `radar_driver/msg/EthCfg`
- `radar_driver/msg/SensorCfg`

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- `colcon`
- `libpcap0.8-dev`
- Ethernet connection to the radar
- Permission to capture packets on the selected network interface

Install the common dependencies:

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions libpcap0.8-dev tcpdump net-tools
```

Source ROS 2 before building or running:

```bash
source /opt/ros/jazzy/setup.bash
```

## Installation

Clone the repository into a ROS 2 workspace:

```bash
mkdir -p ~/radar_ws/src
cd ~/radar_ws/src
git clone https://github.com/mbyr0n/Driver_Radar.git
cd ~/radar_ws
```

Build only this package:

```bash
colcon build --packages-select radar_driver
```

Source the workspace overlay:

```bash
source install/setup.bash
```

## Verify The Build

Check that the package exports the expected executables:

```bash
ros2 pkg executables radar_driver
```

Expected executables:

```text
radar_driver radar_processor
radar_driver radar_publisher
radar_driver radar_visualizer
```

Check that the custom interfaces were generated:

```bash
ros2 interface show radar_driver/msg/RadarPacket
ros2 interface show radar_driver/msg/RadarDetection
```

Run the static regression tests included with this port:

```bash
python3 -m pytest src/Driver_Radar/radar_driver/tests/test_critical_regressions.py -v
```

If you are already inside the repository root, use:

```bash
python3 -m pytest radar_driver/tests/test_critical_regressions.py -v
```

## Network Setup Before Connecting The Radar

Find the Ethernet interface connected to the radar:

```bash
ip link
```

Typical interface names look like `enp3s0`, `enx...`, or `eth0`. Bring the interface up:

```bash
sudo ip link set <iface> up
```

If the radar expects the host to be on a static subnet, assign an address on that subnet. Example:

```bash
sudo ip addr add 192.168.1.10/24 dev <iface>
```

The default radar UDP port used by this driver is `31122`.

Check whether UDP traffic is arriving before starting ROS:

```bash
sudo tcpdump -i <iface> udp port 31122
```

If no packets appear, check power, cabling, radar IP/port configuration, and whether the host interface is on the same network.

## Capture Permissions

`radar_publisher` uses `libpcap`, so it needs packet capture permission.

For quick testing, run the launch command with `sudo -E` only if needed:

```bash
sudo -E bash -c 'source /opt/ros/jazzy/setup.bash && source ~/radar_ws/install/setup.bash && ros2 launch radar_driver single_radar.launch.py iface:=<iface>'
```

For normal use, prefer granting capture capability to the installed node:

```bash
sudo setcap 'cap_net_raw,cap_net_admin=eip' ~/radar_ws/install/radar_driver/lib/radar_driver/radar_publisher
```

Verify the capability:

```bash
getcap ~/radar_ws/install/radar_driver/lib/radar_driver/radar_publisher
```

## Run The Full Single-Radar Pipeline

Source ROS 2 and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/radar_ws/install/setup.bash
```

Launch the three-node pipeline:

```bash
ros2 launch radar_driver single_radar.launch.py iface:=<iface> port:=31122 name:=radar
```

Useful launch arguments:

| Argument | Default | Meaning |
| --- | --- | --- |
| `name` | `radar` | ROS namespace for all nodes. |
| `iface` | `enx5c28868515e9` | Ethernet interface used by `radar_publisher`. Override this for your machine. |
| `port` | `31122` | UDP port used by the radar. |
| `filter` | empty | Optional extra Berkeley Packet Filter expression. |
| `capture` | `1` | `1` for live Ethernet capture, `0` for offline `.pcap` capture. |
| `cpath` | `radarCap5.pcap` | Path to `.pcap` file when `capture:=0`. |
| `scanMode` | `0` | `0` near and far, `1` near only, `2` far only. |
| `uTopic` | `unfiltered_radar_packet` | Topic consumed by `radar_processor`. |

Offline smoke test with a pcap file:

```bash
ros2 launch radar_driver single_radar.launch.py capture:=0 cpath:=/path/to/radar.pcap iface:=lo
```

## Run Nodes Manually

Manual mode is useful when debugging one stage at a time.

Terminal 1, publish unfiltered packets:

```bash
source /opt/ros/jazzy/setup.bash
source ~/radar_ws/install/setup.bash
ros2 run radar_driver radar_publisher -e <iface> -p 31122 -c 1
```

Terminal 2, process packets:

```bash
source /opt/ros/jazzy/setup.bash
source ~/radar_ws/install/setup.bash
ros2 run radar_driver radar_processor -t /unfiltered_radar_packet -s 0
```

Terminal 3, publish point cloud:

```bash
source /opt/ros/jazzy/setup.bash
source ~/radar_ws/install/setup.bash
ros2 run radar_driver radar_visualizer -t /filtered_radar_packet -n radar
```

## What To Verify When The Radar Is Connected

1. Physical link is up:

```bash
ip link show <iface>
```

Look for `state UP` and a connected Ethernet link light.

2. UDP packets are arriving:

```bash
sudo tcpdump -i <iface> udp port 31122
```

3. ROS nodes are running:

```bash
ros2 node list
```

Expected names with the default namespace:

```text
/radar/radar_publisher
/radar/radar_processor
/radar/radar_visualizer
```

4. ROS topics exist:

```bash
ros2 topic list
```

Expected topics include:

```text
/radar/unfiltered_radar_packet
/radar/filtered_radar_packet
/radar/radar_pointcloud
/radar/visualization_marker
```

5. Messages are being published:

```bash
ros2 topic hz /radar/unfiltered_radar_packet
ros2 topic echo /radar/unfiltered_radar_packet --once
```

6. Processed packets are produced:

```bash
ros2 topic hz /radar/filtered_radar_packet
```

7. Point cloud is available:

```bash
ros2 topic info /radar/radar_pointcloud
```

Open RViz 2 if you want to visualize the cloud:

```bash
rviz2
```

Add a `PointCloud2` display and set the topic to `/radar/radar_pointcloud`.

## Troubleshooting

### No UDP packets in tcpdump

- Confirm radar power and Ethernet cable.
- Confirm the host interface is the one physically connected to the radar.
- Confirm the radar is configured to transmit to port `31122`, or override `port:=...`.
- Confirm the host IP is on the radar network.

### `radar_publisher` permission error

- Install `libpcap0.8-dev`.
- Grant capture capabilities with `setcap`.
- For temporary debugging, run with `sudo -E` and source both ROS environments inside the command.

### Topics exist but no detections appear

- Check that the radar sees a target.
- Check `tcpdump` packet rate.
- Try `scanMode:=0` to accept both near and far packets.
- Echo `/radar/unfiltered_radar_packet` before debugging `radar_processor`.

### `filtered_radar_packet` is empty or slow

- Confirm `radar_processor` subscribes to the same namespace/topic that `radar_publisher` publishes.
- With launch defaults, both run under `/radar`.
- If running manually, use fully qualified topic names when needed, for example `-t /radar/unfiltered_radar_packet`.

### RViz shows no cloud

- Confirm `/radar/radar_pointcloud` exists.
- Set the RViz fixed frame to the frame used by the point cloud.
- Confirm `radar_visualizer` receives `filtered_radar_packet`.

## Development Checks

From the repository root or workspace root after sourcing ROS 2:

```bash
colcon build --packages-select radar_driver --cmake-clean-first
python3 -m pytest radar_driver/tests/test_critical_regressions.py -v
colcon test --packages-select radar_driver
colcon test-result --verbose
```

## Known Limitations

- The supported ROS 2 path is currently the single-radar pipeline.
- Multi-radar fusion and clustering code still exists in the repository but is not built as part of the ROS 2 phase 1 package.
- A functional live test requires radar hardware or a valid `.pcap` recorded from the radar.
- Existing RViz config files are from the older ROS 1 workflow and may need updates for ROS 2 displays.
