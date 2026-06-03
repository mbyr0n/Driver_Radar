#include <rclcpp/rclcpp.hpp>
#include <getopt.h>
#include <cstring>
#include "radarPublisher.h"
#include "processPacket.h"
#include "radar_driver/msg/radar_packet.hpp"

extern int opterr;

PacketProcessor packetProcessor;
static rclcpp::Node::SharedPtr processorNode;

void printUsage(char buff[]) {
  printf("\nUsage: %s [-h] [-n RADARNAME][-y UNFILTEREDTOPIC]\r\n", buff);
  printf("\tn: radar namespace (defaults: radar), should use launch script for this\r\n");
  printf("\tt: unfiltered topic name if reading (defaults: unfiltered_radar_packet\r\n");
}

static int getNonRosArgc(int argc, char** argv) {
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--ros-args") == 0) {
      return i;
    }
  }
  return argc;
}

void radarCallback(const radar_driver::msg::RadarPacket::SharedPtr msg) {
  uint8_t ret_status = packetProcessor.processRDIMsg(msg);
  std::string status_info = "";

  uint8_t log_verbosity = LOG_ERROR;

  switch(ret_status) {
    case NO_DETECTIONS:
      status_info = "No Detections";
      log_verbosity = LOG_WARNING;
      break;
    case BAD_EVENT_ID:
      status_info = "Bad Event ID";
      break;
    case NO_PUB_CLR_FAIL:
      status_info = "No Publisher and Unable to Clear Packets";
      break;
    case NO_PUBLISHER:
      status_info = "No Publisher";
      break;
    case PUBLISH_FAIL:
      status_info = "Publishing failed";
      break;
    case CLEAR_FAIL:
      status_info = "Failed to clear packets";
      break;
    case PACKET_GROUP_FULL:
      status_info = "Packet group full";
      break;
    default:
      return;
  }

  if (!processorNode) {
    return;
  }

  if (log_verbosity == LOG_WARNING) {
    RCLCPP_WARN(processorNode->get_logger(), "radarCallback warning, unable to execute processRDIMsg. Cause: %s", status_info.c_str());
  } else if (log_verbosity == LOG_ERROR) {
    RCLCPP_ERROR(processorNode->get_logger(), "radarCallback ERROR, unable to execute processRDIMsg. Cause: %s", status_info.c_str());
  }
}

int main(int argc, char** argv)
{
  opterr = 0;

  int c;
  uint8_t scanMode = 0;
  std::string unfiltered_topic = "unfiltered_radar_packet";
  std::string radar_name = "";
  int customArgc = getNonRosArgc(argc, argv);

  optind = 1;
  while ((c = getopt(customArgc, argv, "+ht:s:n:")) != -1)
  {
    switch (c)
    {
      case 'h':
        printUsage(argv[0]);
        return 0;
      case 't':
        unfiltered_topic = std::string(optarg);
        printf("Unfiltered topic: %s\r\n", unfiltered_topic.c_str());
        break;
      case 's':
        scanMode = atoi(optarg);
        printf("scanMode: %d\r\n", scanMode);
        break;
      case 'n':
        radar_name = std::string(optarg);
        break;
    }
  }

  if (customArgc <= 1) {
      printf("Insufficient args\r\n");
      printUsage(argv[0]);
      return 0;
  }

  if (scanMode > 2) {
      printf("Scan Mode must be 0,1,2\r\n");
      printUsage(argv[0]);
      return 0;
  }

  rclcpp::init(argc, argv);
  processorNode = std::make_shared<rclcpp::Node>("radar_processor");
  RCLCPP_INFO(processorNode->get_logger(), "Initialize radar processor");

  RadarPublisher rp(processorNode);
  rclcpp::Subscription<radar_driver::msg::RadarPacket>::SharedPtr radarUnfilteredSub;

  if (!packetProcessor.initializePacketProcessor(&rp, scanMode)) {
    radarUnfilteredSub = processorNode->create_subscription<radar_driver::msg::RadarPacket>(
      unfiltered_topic,
      100,
      radarCallback
    );
    rclcpp::spin(processorNode);
  } else {
    RCLCPP_INFO(processorNode->get_logger(), "Failed to setup processor");
  }

  rclcpp::shutdown();
  return 0;
}
