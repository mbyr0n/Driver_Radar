#include "radarPublisher.h"
#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <stdlib.h>

//msgs
#include "std_msgs/msg/string.hpp"
#include "radar_driver/msg/eth_cfg.hpp"
#include "radar_driver/msg/radar_detection.hpp"
#include "radar_driver/msg/radar_packet.hpp"
#include "radar_driver/msg/sensor_cfg.hpp"
#include "radar_driver/msg/sensor_status.hpp"

//#define PRINT_PUBLISHER

RadarPublisher::RadarPublisher (rclcpp::Node::SharedPtr nh_)
  : nh_(nh_)
{
  packet_pub_ = nh_->create_publisher<radar_driver::msg::RadarPacket>("filtered_radar_packet", 50);
}

void RadarPublisher::publishRadarPacketMsg(radar_driver::msg::RadarPacket& radar_packet_msg)
{
  packet_pub_->publish(radar_packet_msg);
}

void RadarPublisher::publishRadarDetectionMsg(radar_driver::msg::RadarDetection& radar_detection_msg)
{
  detection_pub_->publish(radar_detection_msg);
}

uint8_t RadarPublisher::pubCallback(PacketGroup_t* Packets) {   //call upon the appropriate publish function
  for (uint8_t i = 0; i < Packets->numFarPackets; i++) {
    this->publishRadarPacketMsg(Packets->farPackets[i]);
    RCLCPP_INFO(nh_->get_logger(), "Far packet timestamp: %u", (Packets->farPackets[i]).time_stamp);
  }

  for (uint8_t i = 0; i < Packets->numNearPackets; i++) {
    this->publishRadarPacketMsg(Packets->nearPackets[i]);
    RCLCPP_INFO(nh_->get_logger(), "Near packet timestamp: %u", (Packets->nearPackets[i]).time_stamp);
  }

  return SUCCESS;
}
