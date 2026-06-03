from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return (ROOT / path).read_text()


def test_unfiltered_packet_is_published():
    parser = read("src/parser.cpp")

    assert "unfilteredPublisher->publish(msg);" in parser
    assert "//unfilteredPublisher.publish(msg);" not in parser


def test_probability_flags_use_bit_masks():
    parser = read("src/parser.cpp")

    assert "ui_Pdh0 && NEAR_PROB_MASK" not in parser
    assert "ui_Pdh0 && INFERENCE_PROB_MASK" not in parser
    assert "ui_Pdh0 && SIDELOBE_PROB_MASK" not in parser
    assert "ui_Pdh0 & NEAR_PROB_MASK" in parser
    assert "ui_Pdh0 & INFERENCE_PROB_MASK" in parser
    assert "ui_Pdh0 & SIDELOBE_PROB_MASK" in parser


def test_64_bit_packet_fields_cast_before_shift():
    parser = read("src/parser.cpp")

    assert "static_cast<uint64_t>(packetptr[30]) << 56" in parser
    assert "static_cast<uint64_t>(packetptr[93]) << 56" in parser
    assert "packetptr[30] << 56" not in parser
    assert "packetptr[93] << 56" not in parser


def test_parser_base_header_length_matches_accesses():
    parser = read("src/parser.cpp")

    assert "packetLen < 24" in parser
    assert "packetLen < UDP_HEADER_LEN + 24" not in parser


def test_sniffer_rejects_malformed_ip_headers():
    sniffer = read("src/sniffer.cpp")

    assert "ipHeaderLen < sizeof(struct ip)" in sniffer
    assert "ntohs(iphdr->ip_len)" in sniffer


def test_timestamp_changes_update_current_timestamp():
    processor = read("src/processPacket.cpp")

    assert "curFarTimeStamp = packet->time_stamp;" in processor
    assert "curNearTimeStamp = packet->time_stamp;" in processor


def test_packet_group_writes_are_bounds_checked():
    processor = read("src/processPacket.cpp")

    assert "curGroup->numFarPackets >= NUM_FAR" in processor
    assert "curGroup->numNearPackets >= NUM_NEAR" in processor


def test_print_packet_group_unlocks_mutex():
    processor = read("src/processPacket.cpp")

    assert "pthread_mutex_unlock(&Mutex);" in processor
    assert "pthread_mutex_lock(&Mutex);\n}" not in processor


def test_cluster_math_initializes_and_guards_empty_inputs():
    cluster = read("src/cluster.cpp")

    assert "double x = 0;" in cluster
    assert "double y = 0;" in cluster
    assert "points.empty()" in cluster
    assert "std::numeric_limits<double>::lowest()" in cluster


def test_pipeline_publish_does_not_preallocate_then_push():
    clustering = read("src/radar_clustering.cpp")

    assert "msg.boxes.resize(rawBoundingBoxes.size());" not in clustering
    assert "msg.boxes.reserve(rawBoundingBoxes.size());" in clustering


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


def test_parser_uses_ros2_messages_and_rclcpp():
    parser_h = read("include/parser.h")
    parser_cpp = read("src/parser.cpp")

    assert "#include \"rclcpp/rclcpp.hpp\"" in parser_h
    assert "radar_driver/msg/radar_packet.hpp" in parser_h
    assert "radar_driver::msg::RadarPacket" in parser_cpp
    assert "rclcpp::Publisher<radar_driver::msg::RadarPacket>::SharedPtr" in parser_cpp
    assert "radar_driver/RadarPacket.h" not in parser_cpp
    assert "radar_driver::RadarPacket" not in parser_cpp


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


def test_unfiltered_main_uses_rclcpp():
    main = read("src/UnfilteredMain.cpp")

    assert "#include <rclcpp/rclcpp.hpp>" in main
    assert "rclcpp::init(argc, argv);" in main
    assert "std::make_shared<rclcpp::Node>(\"radar_publisher\")" in main
    assert "rclcpp::shutdown();" in main
    assert "ros::" not in main


def test_visualizer_uses_ros2_messages():
    visualizer = read("src/PointCloudConverter.cpp")

    assert "#include <rclcpp/rclcpp.hpp>" in visualizer
    assert "sensor_msgs::msg::PointCloud2" in visualizer
    assert "radar_driver::msg::RadarPacket" in visualizer
    assert "rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr" in visualizer
    assert "ros::" not in visualizer


def test_ros2_single_radar_launch_exists():
    launch = read("launch/single_radar.launch.py")

    assert "from launch import LaunchDescription" in launch
    assert "from launch_ros.actions import Node" in launch
    assert "executable='radar_publisher'" in launch
    assert "executable='radar_processor'" in launch
    assert "executable='radar_visualizer'" in launch


def test_parser_header_does_not_expose_libpcap():
    parser_h = read("include/parser.h")

    assert "#include <pcap.h>" not in parser_h


def test_detection_list_parsing_avoids_unsequenced_index_increments():
    parser = read("src/parser.cpp")

    assert "packetptr[listStartIndex++] << 8) | (packetptr[listStartIndex++])" not in parser


def test_package_declares_libpcap_dependency():
    package = read("package.xml")

    assert "<depend>libpcap</depend>" in package


def test_uint64_printf_uses_portable_format_macro():
    parser = read("src/parser.cpp")

    assert "PRIu64" in parser
    assert "UTCTimeStamp: %llu" not in parser


def test_processor_parses_help_before_ros_initialization():
    filtered = read("src/FilteredMain.cpp")

    assert filtered.index("while ((c = getopt") < filtered.index("rclcpp::init(argc, argv);")


def test_nodes_ignore_ros_args_when_parsing_custom_cli_options():
    for path in ["src/UnfilteredMain.cpp", "src/FilteredMain.cpp", "src/PointCloudConverter.cpp"]:
        source = read(path)

        assert "getNonRosArgc" in source
        assert "--ros-args" in source
        assert "getopt(customArgc" in source


def test_launch_does_not_pass_empty_filter_as_missing_argument():
    launch = read("launch/single_radar.launch.py")

    assert "'-f', packet_filter" not in launch
    assert "packet_filter.perform(context)" in launch


def test_unfiltered_publisher_resets_before_ros_shutdown():
    parser_h = read("include/parser.h")
    parser_cpp = read("src/parser.cpp")
    main = read("src/UnfilteredMain.cpp")

    assert "void    resetUnfilteredPublisher();" in parser_h
    assert "void resetUnfilteredPublisher()" in parser_cpp
    assert "resetUnfilteredPublisher();\n  rclcpp::shutdown();" in main


def test_build_warning_patterns_are_removed():
    processor = read("src/processPacket.cpp")
    sniffer = read("src/sniffer.cpp")

    assert "PacketsBuffer[2];" not in processor
    assert "pcap_lookupdev" not in sniffer
    assert "uint32_t  srcip, netmask;" not in sniffer
    assert "struct bpf_program  bpf;" not in sniffer
    assert "unsigned short id, seq;" not in sniffer
    assert "(void)user;" in sniffer
    assert "(void)signo;" in sniffer
