import numpy as np
import rospy
from sensor_msgs.msg import PoseStamped, LaserScan, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import tf
from nav_msgs.msg import OccupancyGrid
import open3d as o3d
import cv2

class EKF_Localization:
    def __init__(self, initial_state, initial_covariance, process_noise_cov, measurement_noise_cov):
        self.state = initial_state
        self.covariance = initial_covariance
        self.Q = process_noise_cov
        self.R = measurement_noise_cov
        self.map_data = None
        self.resolution = 0.1  # Adjust based on your map resolution

    def predict(self, u):
        # Predict the next state based on odometry data
        theta = self.state[2]
        delta_x = u[0]
        delta_y = u[1]
        delta_theta = u[2]

        self.state[0] += delta_x * np.cos(theta) - delta_y * np.sin(theta)
        self.state[1] += delta_x * np.sin(theta) + delta_y * np.cos(theta)
        self.state[2] += delta_theta

        # Update the state transition matrix (F)
        F = np.array([[1, 0, -delta_x * np.sin(theta) - delta_y * np.cos(theta)],
                      [0, 1, delta_x * np.cos(theta) - delta_y * np.sin(theta)],
                      [0, 0, 1]])

        # Update the covariance matrix
        self.covariance = F @ self.covariance @ F.T + self.Q

    def correct(self, z, map_point_cloud):
        # Apply ICP to get the transformation
        transformation = self.icp(z, map_point_cloud)

        # Correct the predicted state based on ICP result
        self.state = self.apply_transformation(self.state, transformation)

        # Compute the measurement Jacobian (H)
        H = np.eye(3)

        # Compute the Kalman Gain
        S = H @ self.covariance @ H.T + self.R
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Correct the predicted state and covariance
        innovation = np.array(transformation)  # Assuming transformation is the difference between predicted and observed state
        self.state = self.state + K @ innovation
        self.covariance = (np.eye(len(self.state)) - K @ H) @ self.covariance


    def map_to_point_cloud(self, map_data):
        # Convert OccupancyGrid map to point cloud
        map_array = np.array(map_data.data).reshape((map_data.info.height, map_data.info.width))
        point_cloud = []

        for y in range(map_array.shape[0]):
            for x in range(map_array.shape[1]):
                if map_array[y, x] < 50:  # Threshold for occupied space (adjust as needed)
                    world_x = x * map_data.info.resolution + map_data.info.origin.position.x
                    world_y = y * map_data.info.resolution + map_data.info.origin.position.y
                    point_cloud.append([world_x, world_y, 0])

        return np.array(point_cloud)

def pose_callback(msg):
    global ekf
    # Convert PoseStamped to a simple odometry update
    u = [msg.pose.position.x, msg.pose.position.y, tf.transformations.euler_from_quaternion(
        [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]]
    ekf.predict(u)

def laser_callback(msg):
    global ekf, map_point_cloud
    ekf.correct(msg, map_point_cloud)

def map_callback(msg):
    global ekf, map_point_cloud
    ekf.map_data = msg
    map_point_cloud = ekf.map_to_point_cloud(msg)

if __name__ == "__main__":
    rospy.init_node("ekf_localization")

    initial_state = [0, 0, 0]
    initial_covariance = np.eye(3) * 0.1
    process_noise_cov = np.eye(3) * 0.1
    measurement_noise_cov = np.eye(3) * 0.1

    ekf = EKF_Localization(initial_state, initial_covariance, process_noise_cov, measurement_noise_cov)
    map_point_cloud = None

    rospy.Subscriber("/pose", PoseStamped, pose_callback)
    rospy.Subscriber("/laser", LaserScan, laser_callback)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)

    rospy.spin()
