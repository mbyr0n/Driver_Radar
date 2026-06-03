#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <cstring>
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "processPacket.h"
#include "radar_driver/msg/radar_detection.hpp"
#include "radar_driver/msg/radar_packet.hpp"

#define MAX_DIST    16 //metres, arbitrary

std::string name = "";
std::string radarFrame = "radar_fixed";
std::vector<radar_driver::msg::RadarPacket> packets;
rclcpp::Node::SharedPtr visualizerNode;
rclcpp::Subscription<radar_driver::msg::RadarPacket>::SharedPtr rawData;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcData;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
visualization_msgs::msg::Marker leftFOVLine, rightFOVline;

uint32_t curTimeStamp = 0;

static int getNonRosArgc(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--ros-args") == 0) {
            return i;
        }
    }
    return argc;
}

void packetCloudGenerator(std::vector<radar_driver::msg::RadarPacket> group) {

    if (group.size() == 0) {
        printf("Skipping empty group\r\n");
        return;
    }
    sensor_msgs::msg::PointCloud2 pc;
    pc.header = group[0].header;
    pc.header.frame_id = radarFrame;
    pc.height = 1;
    pc.width = 0;

    sensor_msgs::msg::PointField pf;
    pf.name = "x";
    pf.offset = 0;
    pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
    pf.count = 1;
    pc.fields.push_back(pf);

    pf.name = "y";
    pf.offset = 4;
    pc.fields.push_back(pf);

    pf.name = "z";
    pf.offset = 8;
    pc.fields.push_back(pf);

    pf.name = "intensity";
    pf.offset = 12;
    pc.fields.push_back(pf);

    pc.is_bigendian = false;
    pc.point_step = 16;
    pc.is_dense = true;

    uint8_t* tmp;
    for (size_t i = 0; i < group.size(); i++) {
        for (size_t j = 0; j < group[i].detections.size(); j++) {
            pc.width++;
            tmp = reinterpret_cast<uint8_t*>(&(group[i].detections[j].pos_x));
            for (uint8_t k = 0; k < 4; k++) {
                pc.data.push_back(tmp[k]);
            }

            tmp = reinterpret_cast<uint8_t*>(&(group[i].detections[j].pos_y));
            for (uint8_t k = 0; k < 4; k++) {
                pc.data.push_back(tmp[k]);
            }

            tmp = reinterpret_cast<uint8_t*>(&(group[i].detections[j].pos_z));
            for (uint8_t k = 0; k < 4; k++) {
                pc.data.push_back(tmp[k]);
            }

            float intensity = (group[i].detections[j].rcs0 / 2.0) + 50.0;
            tmp = reinterpret_cast<uint8_t*>(&intensity);
            for (uint8_t k = 0; k < 4; k++) {
                pc.data.push_back(tmp[k]);
            }
        }
    }

    pc.row_step = pc.point_step * pc.width;
    leftFOVLine.header.stamp = rightFOVline.header.stamp = pc.header.stamp;

    pcData->publish(pc);
    marker_pub->publish(leftFOVLine);
    marker_pub->publish(rightFOVline);
}

void radarCallback(const radar_driver::msg::RadarPacket::SharedPtr msg) {

    if (curTimeStamp == 0) {
        curTimeStamp = msg->time_stamp;
        packets.push_back(*msg);
    } else if (curTimeStamp != msg->time_stamp) {
        packetCloudGenerator(packets);
        curTimeStamp = msg->time_stamp;
        packets.clear();
        packets.push_back(*msg);
    }
    else {
        packets.push_back(*msg);
    }
}

void configureFOVlines(void) {
    leftFOVLine.header.frame_id = rightFOVline.header.frame_id = radarFrame;
    leftFOVLine.ns = rightFOVline.ns = "points_and_lines";
    leftFOVLine.action = rightFOVline.action = visualization_msgs::msg::Marker::ADD;
    leftFOVLine.pose.orientation.w = rightFOVline.pose.orientation.w = 1.0;

    leftFOVLine.id = 1;
    rightFOVline.id = 2;

    leftFOVLine.type = rightFOVline.type = visualization_msgs::msg::Marker::LINE_STRIP;

    leftFOVLine.scale.x = rightFOVline.scale.x = 0.05;
    leftFOVLine.color.b = rightFOVline.color.b = 1.0;
    leftFOVLine.color.r = rightFOVline.color.r = 1.0;
    leftFOVLine.color.g = rightFOVline.color.g = 1.0;
    leftFOVLine.color.a = rightFOVline.color.a = 0.5;

    geometry_msgs::msg::Point p;
    p.x = p.y = p.z = 0;
    rightFOVline.points.push_back(p);
    leftFOVLine.points.push_back(p);

    p.x = MAX_DIST;
    p.y = p.x * tan(AZI_ANGLE_1_THRESHOLD);
    rightFOVline.points.push_back(p);
    p.y *= -1;
    leftFOVLine.points.push_back(p);
}

int main(int argc, char **argv)
{
    std::string packetTopic = "";
    int c = 0;
    int customArgc = getNonRosArgc(argc, argv);

    optind = 1;
    while ((c = getopt(customArgc, argv, "+hn:t:")) != -1) {
        switch (c) {
            case 'h':
                printf("\nUsage: %s [-h] [-t Topic Name]\r\n", argv[0]);
                printf("\tt: Input radar Topic\r\n");
                return 0;
            case 't':
                packetTopic = std::string(optarg);
                printf("Converting Data From Topic: %s\r\n", packetTopic.c_str());
                break;
            case 'n':
                name = std::string(optarg);
                printf("Using namespace:: %s\r\n", name.c_str());
                break;
        }
    }

    rclcpp::init(argc, argv);
    visualizerNode = std::make_shared<rclcpp::Node>("radar_visualizer");

    if (boost::iequals(name,"radar_left")) {
        radarFrame = "radar_left_frame";
    }
    else if (boost::iequals(name,"radar_right")) {
        radarFrame = "radar_right_frame";
    }

    pcData = visualizerNode->create_publisher<sensor_msgs::msg::PointCloud2>("radar_pointcloud", 100);
    rawData = visualizerNode->create_subscription<radar_driver::msg::RadarPacket>(packetTopic, 100, radarCallback);
    marker_pub = visualizerNode->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    packets.clear();
    configureFOVlines();

    rclcpp::spin(visualizerNode);
    rclcpp::shutdown();

    return 0;
}
