#!/usr/bin/env python3

import argparse
from collections import defaultdict
from itertools import cycle
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

cycol = cycle('bgrcmykw')

def read_output(filename):
    x = defaultdict(list)
    y = defaultdict(list)
    z = defaultdict(list)
    with open(filename, encoding='utf-8') as f:
        for line in f.readlines():
            point = line.split(',')
            x[point[3][:1]].append(float(point[0]))
            y[point[3][:1]].append(float(point[1]))
            z[point[3][:1]].append(float(point[2]))
    return x, y, z

def plot_clusters(x, y, z, output):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlim3d([-5, 5])
    ax.set_ylim3d([-5, 5])
    ax.set_zlim3d([-5, 5])

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    for cluster in x:
        ax.scatter(x[cluster], y[cluster], z[cluster], c=next(cycol))
    
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot radar clustering output text as a 3D scatter plot.')
    parser.add_argument('--input', default='data/output.txt', help='Input CSV-like text file.')
    parser.add_argument('--output', default='data/test.png', help='Output image path.')
    args = parser.parse_args()

    x, y, z = read_output(args.input)
    plot_clusters(x, y, z, Path(args.output))
