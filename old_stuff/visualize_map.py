#!/usr/bin/env python3

import yaml
import numpy as np
import matplotlib.pyplot as plt

def load_map_from_yaml(file_path):
    with open(file_path, 'r') as file:
        map_info = yaml.safe_load(file)

    resolution = map_info['resolution']
    origin = map_info['origin']
    width = map_info['width']
    height = map_info['height']
    data = np.array(map_info['data']).reshape(height, width)

    return resolution, origin, width, height, data

def visualize_map(resolution, origin, width, height, data):
    fig, ax = plt.subplots()
    ax.imshow(data, cmap='gray', origin='lower')

    ax.set_title('Occupancy Grid Map')
    ax.set_xlabel('X (cells)')
    ax.set_ylabel('Y (cells)')

    plt.show()

if __name__ == "__main__":
    map_file_path = 'square_room_map.yaml'
    resolution, origin, width, height, data = load_map_from_yaml(map_file_path)
    visualize_map(resolution, origin, width, height, data)
