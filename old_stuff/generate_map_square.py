#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np

def generate_square_room_map():
    rospy.init_node('generate_square_room_map')

    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

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

    # Convert the map to a 1D array
    map_data_1d = map_data.flatten()

    while not rospy.is_shutdown():
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = resolution
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.info.origin = Pose()
        map_msg.data = map_data_1d.tolist()

        map_pub.publish(map_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        generate_square_room_map()
    except rospy.ROSInterruptException:
        pass
