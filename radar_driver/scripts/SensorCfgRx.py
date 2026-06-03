#!/usr/bin/env python3

import argparse
import socket
import struct


def build_message(radome_damping, far_center_freq, near_center_freq):
    message = struct.pack('!I', radome_damping)
    message += struct.pack('!B', far_center_freq)
    message += struct.pack('!B', near_center_freq)
    return message


def main():
    parser = argparse.ArgumentParser(description="Send one ARS430 SensorCfg UDP packet.")
    parser.add_argument('--ip', default='192.168.1.2', help='Radar destination IP address.')
    parser.add_argument('--port', type=int, default=31123, help='Radar configuration UDP port.')
    parser.add_argument('--radome-damping', type=int, default=100, help='Radome damping value.')
    parser.add_argument('--far-center-freq', type=int, default=0, help='Far center frequency byte.')
    parser.add_argument('--near-center-freq', type=int, default=0, help='Near center frequency byte.')
    args = parser.parse_args()

    message = build_message(args.radome_damping, args.far_center_freq, args.near_center_freq)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sent = sock.sendto(message, (args.ip, args.port))

    print(f"Sent {sent} bytes to {args.ip}:{args.port}")


if __name__ == '__main__':
    main()
