import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import cv2

def pgm_to_point_cloud(pgm_file, resolution=0.1):
    # Load PGM file using OpenCV
    image = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)

    # Determine the size of the map
    height, width = image.shape

    # Initialize empty point cloud
    point_cloud = []

    # Convert map to point cloud
    for y in range(height):
        for x in range(width):
            if image[y, x] < 200:  # Threshold for occupied space (adjust as needed)
                # Convert pixel coordinates to world coordinates
                world_x = x * resolution
                world_y = y * resolution

                # Add point to point cloud
                point_cloud.append([world_x, world_y, 0])  # Z-coordinate is set to 0

    return np.array(point_cloud)

def create_point_cloud_msg(point_cloud):
    # Create a PointCloud2 message
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]

    point_cloud_msg = pc2.create_cloud(header, fields, point_cloud)
    return point_cloud_msg

def publish_point_cloud(pgm_file, resolution=0.1):
    rospy.init_node('pgm_to_point_cloud_publisher')
    point_cloud_pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        point_cloud = pgm_to_point_cloud(pgm_file, resolution)
        point_cloud_msg = create_point_cloud_msg(point_cloud)
        point_cloud_pub.publish(point_cloud_msg)
        rate.sleep()

# Example usage
pgm_file = "path/to/your/map.pgm"  # Path to your PGM file
publish_point_cloud(pgm_file)
