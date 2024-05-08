import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import tf
import numpy as np


def laser_callback(msg):
    """
    Callback function for the /pose topic.

    Parameters:
        msg (Path): Message containing the pose data.
    """
    # Run ICP algorithm to match the current laser with the map
    