#!/usr/bin/env python3

import yaml
import numpy as np

def generate_square_room_map_yaml(file_path):
    width = 100
    height = 100
    resolution = 0.1  # Each cell represents 0.1m x 0.1m

    # Initialize map with all free space
    map_data = np.ones((height, width), dtype=np.int8) * 50

    # Define walls
    wall_thickness = 5  # 5 cells thick
    map_data[0:wall_thickness, :] = 100  # Top wall
    map_data[:, 0:wall_thickness] = 100  # Left wall
    map_data[height-wall_thickness:height, :] = 100  # Bottom wall
    map_data[:, width-wall_thickness:width] = 100  # Right wall

    # Add pillars to the walls
    pillar_size = 5  # 5 cells = 0.5 meters

    # Top wall (1 pillar)
    map_data[5:5+pillar_size, 45:45+pillar_size] = 100

    # Right wall (2 pillars)
    map_data[35:35+pillar_size, 90:90+pillar_size] = 100
    map_data[65:65+pillar_size, 90:90+pillar_size] = 100

    # Bottom wall (3 pillars)
    map_data[90:90+pillar_size, 25:25+pillar_size] = 100
    map_data[90:90+pillar_size, 45:45+pillar_size] = 100
    map_data[90:90+pillar_size, 75:75+pillar_size] = 100

    # Left wall (4 pillars)
    map_data[20:20+pillar_size, 5:5+pillar_size] = 100
    map_data[40:40+pillar_size, 5:5+pillar_size] = 100
    map_data[60:60+pillar_size, 5:5+pillar_size] = 100
    map_data[80:80+pillar_size, 5:5+pillar_size] = 100

    # Create the YAML dictionary
    map_yaml = {
        'resolution': resolution,
        'width': width,
        'height': height,
        'origin': [0.0, 0.0, 0.0],
        'data': map_data.flatten().tolist()
    }

    # Write the dictionary to a YAML file
    with open(file_path, 'w') as file:
        yaml.dump(map_yaml, file, default_flow_style=False)

if __name__ == '__main__':
    generate_square_room_map_yaml('square_room_map.yaml')
