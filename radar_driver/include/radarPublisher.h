#ifndef RADAR_PUBLISHER_H
#define RADAR_PUBLISHER_H

/* File: radarPublisher.h
 *
 * Publishes the radar data
 */

#include <rclcpp/rclcpp.hpp>
#include <sys/types.h>
#include <sys/prctl.h>
#include "processPacket.h"

 //msgs
 #include "radar_driver/msg/radar_packet.hpp"
 #include "radar_driver/msg/radar_detection.hpp"
 #include "radar_driver/msg/eth_cfg.hpp"
 #include "radar_driver/msg/sensor_cfg.hpp"
 #include "radar_driver/msg/sensor_status.hpp"

class RadarPublisher {
	private:
		rclcpp::Node::SharedPtr nh_; //Node Handle

		// Publisher Objects
		rclcpp::Publisher<radar_driver::msg::RadarPacket>::SharedPtr packet_pub_;
		rclcpp::Publisher<radar_driver::msg::RadarDetection>::SharedPtr detection_pub_; //Not used now, kept if useful

		//Publisher Functions
		void publishRadarPacketMsg(radar_driver::msg::RadarPacket& radar_packet_msg);
		void publishRadarDetectionMsg(radar_driver::msg::RadarDetection& radar_detection_msg);

	public:
		RadarPublisher(rclcpp::Node::SharedPtr new_nh_);
		uint8_t pubCallback(PacketGroup_t* Packets);
};

#endif
