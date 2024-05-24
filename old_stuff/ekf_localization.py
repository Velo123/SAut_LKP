#!/usr/bin/env python3

import rospy
import numpy as np
import tf
from scipy.ndimage import sobel
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker
from icp_algorithm import icp

# Covariance for EKF
Q = np.diag([0.1, 0.1, np.deg2rad(1.0)]) ** 2  # Process noise covariance
R = np.diag([0.1, 0.1, np.deg2rad(1.0)]) ** 2  # Measurement noise covariance

class EKF_Localization:
    def __init__(self, initial_state, initial_covariance):
        self.state = initial_state
        self.covariance = initial_covariance
        self.map_data = None
        self.map_point_cloud = None
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.prev_pose = initial_state

    def odom_2_command(self, pose):
        dist = np.sqrt((pose[0] - self.prev_pose[0])**2 + (pose[1] - self.prev_pose[1])**2)
        delta_theta = pose[2] - self.prev_pose[2]
        self.prev_pose = pose
        return [dist, delta_theta]

    def predict(self, u):
        delta_x = u[0] * np.cos(self.state[2])
        delta_y = u[0] * np.sin(self.state[2])
        delta_theta = u[1]

        self.state[0] += delta_x
        self.state[1] += delta_y
        self.state[2] += delta_theta

        F = np.array([[1, 0, -delta_y],
                      [0, 1, delta_x],
                      [0, 0, 1]])

        # Update covariance with process noise
        self.covariance = F @ self.covariance @ F.T + Q
        print("Predicted state: ", self.state)

    def correct(self, z):
        if self.map_point_cloud is None:
            return

        laser_point_cloud = self.laser_to_point_cloud(z)
        
        icp_result = icp(laser_point_cloud, self.map_point_cloud)
        #print("ICP result: ", icp_result)
        delta_x, delta_y, delta_theta = icp_result    #icp_result[0], icp_result[1], icp_result[2]

        H = np.eye(3)
        innovation = np.array([delta_x, delta_y, delta_theta])

        # Calculate the Kalman Gain
        S = H @ self.covariance @ H.T + R
        K = self.covariance @ H.T @ np.linalg.inv(S)
        print("Kalman gain: ", K)

        # Update the state and covariance
        self.state += K @ innovation
        self.covariance = (np.eye(len(self.state)) - K @ H) @ self.covariance
        print("Innovation: ", innovation)
        print("Corrected state: ", self.state)

        self.publish_marker(self.state[0], self.state[1], self.state[2])

    def laser_to_point_cloud(self, laser_data):
        angle = laser_data.angle_min
        point_cloud = []

        for r in laser_data.ranges:
            if laser_data.range_min < r < laser_data.range_max:
                x_laser = r * np.cos(angle)
                y_laser = r * np.sin(angle)

                # Transform from laser frame to global frame
                x_global = self.state[0] + x_laser * np.cos(self.state[2]) - y_laser * np.sin(self.state[2])
                y_global = self.state[1] + x_laser * np.sin(self.state[2]) + y_laser * np.cos(self.state[2])

                point_cloud.append([x_global, y_global, 0])
            angle += laser_data.angle_increment

        return np.array(point_cloud)

    def map_to_point_cloud(self, map_data):
        width = map_data.info.width
        height = map_data.info.height
        resolution = map_data.info.resolution
        origin = map_data.info.origin

        # Reshape the map data into a 2D array
        map_array = np.array(map_data.data).reshape((height, width))

        # Apply Sobel filter to detect edges
        sobel_x = sobel(map_array, axis=1)
        sobel_y = sobel(map_array, axis=0)
        edges = np.hypot(sobel_x, sobel_y)

        # Threshold the edges to get a binary edge map
        edge_threshold = 100  # Adjust threshold as needed
        edge_points = edges > edge_threshold

        point_cloud = []

        for y in range(height):
            for x in range(width):
                if edge_points[y, x]:
                    wx = x * resolution + origin.position.x
                    wy = y * resolution + origin.position.y
                    point_cloud.append([wx, wy, 0])

        point_cloud_np = np.array(point_cloud)
        print("Map point cloud sample (edges only):", point_cloud_np[:5])  # Print first 5 points
        return point_cloud_np

    def publish_marker(self, x, y, theta):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ekf"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        
        # Set orientation of the arrow based on theta
        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]

        marker.scale.x = 1.0  # Length of the arrow
        marker.scale.y = 0.1  # Width of the arrow
        marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)
        print("Published marker")

def pose_callback(msg):
    global ekf
    # Extract pose from Odometry message
    pose = msg.pose.pose
    u = [
        pose.position.x,
        pose.position.y,
        tf.transformations.euler_from_quaternion([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        ])[2]
    ]
    command = ekf.odom_2_command(u)
    ekf.predict(command)

def laser_callback(msg):
    global ekf
    ekf.correct(msg)

def map_callback(msg):
    global ekf
    ekf.map_data = msg
    ekf.map_point_cloud = ekf.map_to_point_cloud(msg)
    print("Map received")

if __name__ == "__main__":
    rospy.init_node("ekf_localization")
    counter = 1

    initial_state = np.array([1.9, 2.0, 0.0])
    initial_covariance = np.eye(3) * 0.1

    ekf = EKF_Localization(initial_state, initial_covariance)

    if counter == 1:
        rospy.Subscriber("/map", OccupancyGrid, map_callback)
        counter = 0
    rospy.Subscriber("/pose", Odometry, pose_callback)
    rospy.Subscriber("/laser", LaserScan, laser_callback)
    
    rospy.spin()
