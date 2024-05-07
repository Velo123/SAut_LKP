"""
Simple script that can be run in a system that has ros
Template for echo code that can be adapted to any topic

to be used when you want to save a certain published messaged to be replayed later

can also be used if you dont want to create a full node but still want to interact with other nodes
"""



import rospy
import time
from common_msgs.msg import ConeDetections, Cone
from nav_msgs.msg import Path

class Echo :

    def __init__(self, listen_topic, publish_topic, message_type):
        self.listen_topic = listen_topic
        self.publish_topic = publish_topic
        self.message_type = message_type

        self.listen_mode = False
        self.loaded_message = False
        
        self.subTopic()
        self.advertTopic()
        self.loop()

    def subTopic(self):
        # name of the topic to listen to
        self.my_subscriber = rospy.Subscriber(self.listen_topic, self.message_type, self.messageCallback)
        print("Listening to topic: ", self.listen_topic)
        print("with type: ", self.message_type)
        print()

    def advertTopic(self):
        # name of the topic that you will want to publish
        self.my_publsiher = rospy.Publisher(self.publish_topic, self.message_type, queue_size=1)
        print("Publishing to topic: ", self.publish_topic)
        print("with type: ", self.message_type)
        print()

    def messageCallback(self, msg):

        if self.listen_mode:
            print("have saved a new message")
            self.loaded_message = msg

    def publishMessage(self):
        self.my_publsiher.publish(self.loaded_message)
        
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



def main():

    # Variables to CHANGE according to your need
    listen_topic = "/control/path_planner/centerline"
    publish_topic = "/control/path_planner/centerline"
    message_type = Path


    rospy.init_node('echo_node')

    # creat the main class
    my_echo = Echo(listen_topic, publish_topic, message_type)


if __name__ == "__main__":
    main()


