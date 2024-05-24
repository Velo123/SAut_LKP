#!/usr/bin/env python3

import rospy
import numpy as np
import tf
import tf2_ros
from sensor_msgs.msg import PointField, PointCloud2
import std_msgs.msg
from std_msgs.msg import Header
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2

def points_to_pointcloud2(points, frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]

    point_cloud = np.array(points, dtype=np.float32)
    return pc2.create_cloud(header, fields, point_cloud)

        
def icp (source_points, target_points):
    try:

        # Initialize the tf buffer and listener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
     
        if source_points is None:
            print("Failed to transform source points.")
            return 0, 0, 0

        # Publish transformed source points to /icp topic
        publish_pointcloud(source_points, "map", '/icp_laser')
        publish_pointcloud(target_points, "map", '/icp_map')

        # Convert source and target points to Open3D PointCloud
        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(source_points)
        
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(target_points)

        # Perform ICP registration
        max_correspondence_distance = 1  # Adjust based on your data and needs
        icp_result = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        transformation = icp_result.transformation
        #print("Transformation:", transformation)
        
        # Log ICP result details
        fitness = icp_result.fitness
        inlier_rmse = icp_result.inlier_rmse
        num_correspondences = len(icp_result.correspondence_set)

        rospy.loginfo(f"ICP fitness: {fitness}")
        rospy.loginfo(f"ICP inlier RMSE: {inlier_rmse}")
        rospy.loginfo(f"Number of correspondences: {num_correspondences}")

        delta_x = transformation[0, 3]
        delta_y = transformation[1, 3]
        delta_theta = np.arctan2(transformation[1, 0], transformation[0, 0])

        # Check for large or unrealistic transformations
        if abs(delta_x) > 10 or abs(delta_y) > 10 or abs(delta_theta) > np.pi:
            rospy.logwarn(f"Unrealistic transformation detected: delta_x={delta_x}, delta_y={delta_y}, delta_theta={delta_theta}")
            return 0, 0, 0

        # Check if the transformation is (0, 0, 0)
        if np.isclose([delta_x, delta_y, delta_theta], [0, 0, 0], atol=1e-3).all():
            rospy.loginfo("ICP returned expected transformation (0, 0, 0).")
        else:
            rospy.logwarn(f"Unexpected transformation: delta_x={delta_x}, delta_y={delta_y}, delta_theta={delta_theta}")

        return delta_x, delta_y, delta_theta
    except Exception as e:
        rospy.logerr("ICP failed: {}".format(e))
        return 0, 0, 0  # Return default values or handle the error as needed

def publish_pointcloud(points, frame_id, topic_name):
    icp_pub = rospy.Publisher(topic_name, PointCloud2, queue_size=10)
    pointcloud_msg = points_to_pointcloud2(points, frame_id)
    icp_pub.publish(pointcloud_msg)
    print("Published point cloud.") 
