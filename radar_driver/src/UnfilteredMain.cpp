#include <rclcpp/rclcpp.hpp>
#include <getopt.h>
#include <cstring>
#include "parser.h"

extern int opterr;

void printUsage(char buff[]) {
  printf("\nUsage: %s [-h] [-e interface] [-n packets] [-p port] [-f filter] [-c capture] [-l cpath] \r\n", buff);
  printf("\te: ethernet interface (string)\n");
  printf("\tn: number of packets (int)\n");
  printf("\tp: port (int)\n");
  printf("\tf: extra filter string (optional), enclose string in \" \" (string)\n");
  printf("\tc: capture method (int), default 1, LIVE\n");
  printf("\tl: file path to captured data (string), required for OFFLINE capture\n");
}

static int getNonRosArgc(int argc, char** argv) {
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--ros-args") == 0) {
      return i;
    }
  }
  return argc;
}

int main(int argc, char** argv)
{
  opterr = 0;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("radar_publisher");
  RCLCPP_INFO(node->get_logger(), "Initialize radar parser");

  char interface[16] = "";
  char filter[256] = "";
  int packets = 0, c, port = 31122;
  int capture_live = 1;
  char capture_path[256] = "";
  int customArgc = getNonRosArgc(argc, argv);

  optind = 1;
  while ((c = getopt(customArgc, argv, "+hp:e:n:c:l:f:")) != -1)
  {
    switch (c)
    {
      case 'h':
        printUsage(argv[0]);
        rclcpp::shutdown();
        return 0;
      case 'e':
        strncpy(interface, optarg, sizeof(interface) - 1);
        interface[sizeof(interface) - 1] = '\0';
        printf("INTERFACE: %s\r\n", optarg);
        break;
      case 'n':
        packets = atoi(optarg);
        break;
      case 'p':
        port = atoi(optarg);
        printf("PORT: %d\r\n", port);
        break;
      case 'c':
        capture_live = atoi(optarg);
        printf("CAPTURE: %d\r\n", capture_live);
        break;
      case 'l':
        strncpy(capture_path, optarg, sizeof(capture_path) - 1);
        capture_path[sizeof(capture_path) - 1] = '\0';
        printf("CAPTURE_PATH: %s\r\n", capture_path);
        break;
      case 'f':
        strncpy(filter, optarg, sizeof(filter) - 1);
        filter[sizeof(filter) - 1] = '\0';
        printf("FILTER: %s\r\n", filter);
        break;
    }
  }

  if (customArgc <= 1) {
      RCLCPP_INFO(node->get_logger(), "Insufficient args");
      printUsage(argv[0]);
      rclcpp::shutdown();
      return 0;
  }

  if (capture_live == LIVE) {
    printf("RUNNING LIVE\r\n");
  } else if (capture_live == OFFLINE) {
    printf("RUNNING OFFLINE\r\n");
  } else {
    printf("Bad capture option, Definition in parser.h\r\n");
    rclcpp::shutdown();
    return 0;
  }

  initUnfilteredPublisher(node);

  run(port, packets, interface, filter, capture_live, capture_path);
  resetUnfilteredPublisher();
  rclcpp::shutdown();

  return 0;
}
