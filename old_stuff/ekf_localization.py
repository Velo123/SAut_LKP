#!/usr/bin/env python

import rospy
import numpy as np
import tf
from sensor_msgs.msg import PoseStamped, LaserScan
from nav_msgs.msg import OccupancyGrid
import open3d as o3d

# Covariance for EKF simulation
Q = np.diag([0.1, 0.1, np.deg2rad(1.0)]) ** 2  # Process noise covariance
R = np.diag([1.0, 1.0, np.deg2rad(5.0)]) ** 2  # Measurement noise covariance

DT = 0.1  # time tick [s]

class EKF_Localization:
    def __init__(self, initial_state, initial_covariance):
        self.state = initial_state
        self.covariance = initial_covariance
        self.map_data = None
        self.map_point_cloud = None

    def predict(self, u):
        theta = self.state[2]
        delta_x = u[0] * np.cos(theta) * DT
        delta_y = u[0] * np.sin(theta) * DT
        delta_theta = u[1] * DT

        self.state[0] += delta_x
        self.state[1] += delta_y
        self.state[2] += delta_theta

        F = np.array([[1.0, 0, -delta_x * np.sin(theta)],
                      [0, 1.0, delta_y * np.cos(theta)],
                      [0, 0, 1.0]])

        self.covariance = F @ self.covariance @ F.T + Q

    def correct(self, z):
        if self.map_point_cloud is None:
            return

        laser_point_cloud = self.laser_to_point_cloud(z)

        icp_result = self.icp(laser_point_cloud, self.map_point_cloud)
        delta_x, delta_y, delta_theta = icp_result

        self.state[0] += delta_x
        self.state[1] += delta_y
        self.state[2] += delta_theta

        H = np.eye(3)
        S = H @ self.covariance @ H.T + R
        K = self.covariance @ H.T @ np.linalg.inv(S)

        innovation = np.array([delta_x, delta_y, delta_theta])
        self.state += K @ innovation
        self.covariance = (np.eye(len(self.state)) - K @ H) @ self.covariance

    def icp(self, source_points, target_points):
        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(source_points)
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(target_points)

        icp_result = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance=0.5,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        transformation = icp_result.transformation

        delta_x = transformation[0, 3]
        delta_y = transformation[1, 3]
        delta_theta = np.arctan2(transformation[1, 0], transformation[0, 0])

        return delta_x, delta_y, delta_theta

    def laser_to_point_cloud(self, laser_data):
        angle = laser_data.angle_min
        point_cloud = []

        for r in laser_data.ranges:
            if laser_data.range_min < r < laser_data.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                point_cloud.append([x, y, 0])
            angle += laser_data.angle_increment

        return np.array(point_cloud)

    def map_to_point_cloud(self, map_data):
        width = map_data.info.width
        height = map_data.info.height
        resolution = map_data.info.resolution
        origin = map_data.info.origin

        map_array = np.array(map_data.data).reshape((height, width))
        point_cloud = []

        for y in range(height):
            for x in range(width):
                if map_array[y, x] < 50:  # Adjust threshold as needed
                    wx = x * resolution + origin.position.x
                    wy = y * resolution + origin.position.y
                    point_cloud.append([wx, wy, 0])

        return np.array(point_cloud)

def pose_callback(msg):
    global ekf
    u = [msg.pose.position.x, tf.transformations.euler_from_quaternion([
        msg.pose.orientation.x, msg.pose.orientation.y,
        msg.pose.orientation.z, msg.pose.orientation.w
    ])[2]]
    ekf.predict(u)

def laser_callback(msg):
    global ekf
    ekf.correct(msg)

def map_callback(msg):
    global ekf
    ekf.map_data = msg
    ekf.map_point_cloud = ekf.map_to_point_cloud(msg)

if __name__ == "__main__":
    rospy.init_node("ekf_localization")

    initial_state = np.zeros(3)
    initial_covariance = np.eye(3) * 0.1

    ekf = EKF_Localization(initial_state, initial_covariance)

    rospy.Subscriber("/pose", PoseStamped, pose_callback)
    rospy.Subscriber("/laser", LaserScan, laser_callback)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)

    rospy.spin()
