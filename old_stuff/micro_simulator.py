#!/usr/bin/env python3

import rospy
import numpy as np
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import tf.transformations

class MicroSimulator:
    get_map = True
    def __init__(self):
        rospy.init_node('micro_simulator')

        self.pose_pub = rospy.Publisher('/pose', Odometry, queue_size=10)
        self.laser_pub = rospy.Publisher('/laser', LaserScan, queue_size=10)
        self.tf_broadcaster = TransformBroadcaster()
        self.rate = rospy.Rate(10)

        self.position = np.array([1.9, 2.0])
        self.orientation = - np.pi/2
        self.move_index = 0
        self.rotating = False
        self.rotation_steps = 0
        self.total_rotation_steps = 10

        self.corners = [
            np.array([2.0, 2.0]),
            np.array([8.0, 2.0]),
            np.array([8.0, 8.0]),
            np.array([2.0, 8.0]),
        ]
        

        self.map_data = None
        self.map_resolution = None
        self.map_origin = None

        if (self.get_map):
            rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
            self.get_map = False

    def map_callback(self, data):
        self.map_data = np.array(data.data).reshape(data.info.height, data.info.width)
        self.map_resolution = data.info.resolution
        self.map_origin = np.array([data.info.origin.position.x, data.info.origin.position.y])

    def simulate_pose(self):
        if self.move_index == 0:
            self.corners[0] = np.array([2.0, 2.0])
        elif self.move_index == 1:
            self.corners[1] = np.array([8.0, 2.0])
        elif self.move_index == 2:
            self.corners[2] = np.array([8.0, 8.0])
        elif self.move_index == 3:
            self.corners[3] = np.array([2.0, 8.0])
        if not self.rotating:
            target_position = self.corners[self.move_index]
            direction_vector = target_position - self.position
            norm = np.linalg.norm(direction_vector)


            if norm > 0.1:  # Only move if the distance to the target is greater than the step size
                step_vector = direction_vector / norm * 0.1
                self.position += step_vector
            else:
                self.position = target_position
                self.rotating = True
                self.rotation_steps = 0

        if self.rotating:
            rotation_increment = np.pi / 2 / self.total_rotation_steps  # 90 degrees in radians
            self.orientation += rotation_increment
            self.rotation_steps += 1

            if self.rotation_steps >= self.total_rotation_steps:
                self.move_index = (self.move_index + 1) % 4
                self.rotating = False

        # Ground truth pose
        ground_truth_position = np.copy(self.position)
        ground_truth_orientation = self.orientation

        # Add noise to ground truth
        noisy_position = ground_truth_position + np.random.normal(0, 0, 2)
        noisy_orientation = ground_truth_orientation + np.random.normal(0, 0)

        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = noisy_position[0]
        msg.pose.pose.position.y = noisy_position[1]
        msg.pose.pose.position.z = 0  # Ensure z position is set to 0 for a 2D plane

        quaternion = tf.transformations.quaternion_from_euler(0, 0, noisy_orientation)
        msg.pose.pose.orientation = Quaternion(*quaternion)

        self.pose_pub.publish(msg)

        # Broadcast the transform from odom to base_link
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = noisy_position[0]
        t.transform.translation.y = noisy_position[1]
        t.transform.translation.z = 0
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def simulate_laser(self):
        if self.map_data is None:
            return

        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "laser"
        msg.angle_min = -(100/180) * np.pi
        msg.angle_max = (100/180) * np.pi
        msg.angle_increment = (0.35 * np.pi) / 180
        msg.range_min = 0.01
        msg.range_max = 5.0

        ranges = []
        for i in range(int((msg.angle_max - msg.angle_min) / msg.angle_increment)):
            angle = msg.angle_min + i * msg.angle_increment
            r = msg.range_max + 1

            for dist in np.arange(0, msg.range_max, self.map_resolution):
                x = self.position[0] + dist * np.cos(self.orientation + angle)
                y = self.position[1] + dist * np.sin(self.orientation + angle)

                xi = int((x - self.map_origin[0]) / self.map_resolution)
                yi = int((y - self.map_origin[1]) / self.map_resolution)

                if 0 <= xi < self.map_data.shape[1] and 0 <= yi < self.map_data.shape[0]:
                    if self.map_data[yi, xi] == 100:  # Assuming 100 represents an obstacle
                        r = dist + np.random.normal(0, 0.01)  # Add noise to the distance measurement
                        break
            

            if r > msg.range_max:
                ranges.append(float('NaN'))  # Append NaN if distance exceeds range_max
            else:
                ranges.append(r)

        msg.ranges = ranges
        self.laser_pub.publish(msg)
        # Broadcast the transform from base_link to laser
        t_laser = TransformStamped()
        t_laser.header.stamp = msg.header.stamp
        t_laser.header.frame_id = "map"
        t_laser.child_frame_id = "laser"
        t_laser.transform.translation.x = self.position[0]  # Example offset from base_link to laser
        t_laser.transform.translation.y = self.position[1]
        t_laser.transform.translation.z = 0.0
        t_laser.transform.rotation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, self.orientation))

        self.tf_broadcaster.sendTransform(t_laser)

    def run(self):
        while not rospy.is_shutdown():
            self.simulate_pose()
            self.simulate_laser()
            self.rate.sleep()

if __name__ == "__main__":
    simulator = MicroSimulator()
    simulator.run()
