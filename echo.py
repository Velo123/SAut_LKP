"""
Simple script that can be run in a system that has ros
Template for echo code that can be adapted to any topic

to be used when you want to save a certain published messaged to be replayed later

can also be used if you dont want to create a full node but still want to interact with other nodes
"""




import matplotlib.pyplot as plt
import numpy as np
import rospy
import time
from common_msgs.msg import ConeDetections, Cone
from nav_msgs.msg import Path



    def __init__(self, listen_topic, publish_topic, message_type):
        self.listen_topic = listen_topic
        self.message_type = message_type

        self.listen_mode = False
        self.loaded_message = False
        
        self.subTopic()
        self.loop()

    def subTopic(self):
        # name of the topic to listen to
        self.my_subscriber = rospy.Subscriber(self.listen_topic, self.message_type, self.messageCallback)
        print("Listening to topic: ", self.listen_topic)
        print("with type: ", self.message_type)
        print()

    def messageCallback(self, msg):

        if self.listen_mode:
            print("have received a new message")
            self.loaded_message = msg

    def loop(self):
        # start ui loop
        while True:

            user_input = input(f"""
        L -> toggle listen mode ({self.listen_mode})
        P -> publish current message
        Q -> exit
    """)

            # received key to toggle listen mode
            if user_input == "L" or user_input == "l":
                self.listen_mode = not self.listen_mode
                print("Toggling listen mode to: ", self.listen_mode)

            # received key to publish current message
            if user_input == "P" or user_input == "p" or user_input == "" or user_input == " ":
                print("Publishing message")
                self.publishMessage()


            # received key to quit
            if user_input == "q" or user_input == "Q":
                print("exiting the program")
                break


#Function that does a x and y plot of the pose with a arrow that points to the direction of the robot
def plot_robot_pose(Sub_topic prediction):
    """
    Plot the pose of a robot with an arrow indicating its direction.

    Parameters:
        x (float): x-coordinate of the robot's pose.
        y (float): y-coordinate of the robot's pose.
        theta (float): orientation of the robot in radians (0 is pointing along the x-axis).
    """
    x = prediction.loaded_message.poses[0].pose.position.x
    y = prediction.loaded_message.poses[0].pose.position.y
    theta = prediction.loaded_message.poses[0].pose.orientation.z
    # Set up the figure and axis
    fig, ax = plt.subplots()

    # Plot the robot position
    ax.plot(x, y, 'ro')

    # Calculate the end point of the arrow
    arrow_length = 0.5
    dx = arrow_length * np.cos(theta)
    dy = arrow_length * np.sin(theta)

    # Plot the arrow
    ax.arrow(x, y, dx, dy, head_width=0.1, head_length=0.1, fc='k', ec='k')

    # Set equal aspect ratio
    ax.set_aspect('equal', adjustable='box')

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Robot Pose')

    # Show the plot
    plt.show()
    


def main():

    # Variables to CHANGE according to your need
    listen_topic1 = "/scan"
    listen_topic2 = "/pose"
    publish_topic = "/localization"
    message_type = Path


    rospy.init_node('localization_node')

    # creat the main class
    prediction = Sub_topic(listen_topic1, publish_topic, message_type)
    plot_robot_pose(prediction)
    observation = Sub_topic(listen_topic2, publish_topic, message_type)
    


if __name__ == "__main__":
    main()


