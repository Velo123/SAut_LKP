#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PoseStamped, LaserScan
import tf

class MicroSimulator:
    def __init__(self):
        rospy.init_node("micro_simulator")

        self.pose_pub = rospy.Publisher("/pose", PoseStamped, queue_size=10)
        self.laser_pub = rospy.Publisher("/laser", LaserScan, queue_size=10)

        self.position = np.array([1.0, 1.0])
        self.orientation = 0.0
        self.side_length = 1.0
        self.move_index = 0
        self.move_directions = [
            np.array([0, 1]),  # up
            np.array([1, 0]),  # right
            np.array([0, -1]), # down
            np.array([-1, 0])  # left
        ]
        self.rate = rospy.Rate(10)  # 10 Hz

    def simulate_pose(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        # Update position and orientation
        direction = self.move_directions[self.move_index]
        self.position += direction * 0.1
        if np.linalg.norm(self.position - np.round(self.position)) < 0.1:
            self.move_index = (self.move_index + 1) % 4

        if direction[0] == 1:
            self.orientation = 0
        elif direction[0] == -1:
            self.orientation = np.pi
        elif direction[1] == 1:
            self.orientation = np.pi / 2
        elif direction[1] == -np.pi / 2:
            self.orientation = -np.pi / 2

        msg.pose.position.x = self.position[0]
        msg.pose.position.y = self.position[1]
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.orientation)
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]

        self.pose_pub.publish(msg)

    def simulate_laser(self):
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "laser_frame"

        # Simulated laser data for a 10x10 room
        msg.angle_min = -np.pi / 2
        msg.angle_max = np.pi / 2
        msg.angle_increment = np.pi / 180
        msg.range_min = 0.2
        msg.range_max = 10.0

        ranges = []
        for angle in np.arange(msg.angle_min, msg.angle_max, msg.angle_increment):
            r = 10.0
            if -np.pi / 4 <= angle < np.pi / 4:
                r = 10.0 - self.position[0]
            elif np.pi / 4 <= angle < 3 * np.pi / 4:
                r = 10.0 - self.position[1]
            elif -3 * np.pi / 4 <= angle < -np.pi / 4:
                r = self.position[1]
            else:
                r = self.position[0]

            ranges.append(r + np.random.uniform(-0.05, 0.05))  # Add some noise

        msg.ranges = ranges
        self.laser_pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            self.simulate_pose()
            self.simulate_laser()
            self.rate.sleep()

if __name__ == "__main__":
    simulator = MicroSimulator()
    simulator.run()
