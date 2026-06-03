#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.node import Node

from radar_driver.msg import RadarDetection, RadarPacket


def make_packet(name):
    packet = RadarPacket()
    packet.header.frame_id = 'radar_fixed'
    packet.event_id = 0
    packet.time_stamp = 1
    packet.measurement_counter = 1
    packet.vambig = 0.0
    packet.center_frequency = 0.0
    packet.name = name

    first = RadarDetection()
    first.pos_x = 4.0
    first.pos_y = 0.5
    first.pos_z = 0.0
    first.range = 4.0
    first.vrel_rad = 9.0
    first.snr = 15.0

    second = RadarDetection()
    second.pos_x = 7.0
    second.pos_y = -0.5
    second.pos_z = 0.0
    second.range = 7.0
    second.vrel_rad = 1.0
    second.snr = 10.0

    packet.detections = [first, second]
    return packet


class RadarPacketTestPublisher(Node):
    def __init__(self, topic, name, rate_hz):
        super().__init__('radar_packet_test_publisher')
        self.publisher = self.create_publisher(RadarPacket, topic, 10)
        self.packet_name = name
        self.counter = 0
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_packet)

    def publish_packet(self):
        packet = make_packet(self.packet_name)
        packet.header.stamp = self.get_clock().now().to_msg()
        packet.measurement_counter = self.counter
        packet.time_stamp = self.counter
        self.publisher.publish(packet)
        self.counter += 1


def main():
    parser = argparse.ArgumentParser(description='Publish sample ROS 2 RadarPacket messages for pipeline testing.')
    parser.add_argument('--topic', default='/radar/unfiltered_radar_packet', help='Topic to publish sample RadarPacket messages on.')
    parser.add_argument('--name', default='radar', help='Radar name field in the sample packets.')
    parser.add_argument('--rate', type=float, default=5.0, help='Publish rate in Hz.')
    args = parser.parse_args()

    rclpy.init()
    node = RadarPacketTestPublisher(args.topic, args.name, args.rate)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
