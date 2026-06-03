#!/usr/bin/env python3

import argparse

import matplotlib.pyplot as plt


def read_values(path):
    ranges = []
    vel_rad = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if "f_Range " in line:
                parts = line.split()
                ranges.append(float(parts[1]))
            if "f_VrelRad " in line:
                parts = line.split()
                vel_rad.append(float(parts[1]))
    return ranges, vel_rad


def plot_values(ranges, vel_rad, output=None):
    fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)

    ax1.plot(ranges[50:160], color='r')
    ax1.set_title("Ranges")
    ax2.plot(vel_rad[13500:15000], color='b')
    ax2.set_title("VelRad")

    if output:
        fig.savefig(output)
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Plot selected range and radial velocity samples.')
    parser.add_argument('--input', default='test7.txt', help='Input text file containing f_Range and f_VrelRad lines.')
    parser.add_argument('--output', default=None, help='Optional output image path. If omitted, opens a plot window.')
    args = parser.parse_args()

    ranges, vel_rad = read_values(args.input)
    plot_values(ranges, vel_rad, args.output)


if __name__ == '__main__':
    main()
