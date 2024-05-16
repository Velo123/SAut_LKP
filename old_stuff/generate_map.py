import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose

def generate_square_room_map():
    rospy.init_node('map_generator')

    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    map_msg = OccupancyGrid()
    
    # Define map properties
    map_msg.info = MapMetaData()
    map_msg.info.resolution = 0.1
    map_msg.info.width = 100
    map_msg.info.height = 100
    map_msg.info.origin = Pose()
    map_msg.info.origin.position.x = 0.0
    map_msg.info.origin.position.y = 0.0
    map_msg.info.origin.position.z = 0.0
    map_msg.info.origin.orientation.w = 1.0

    # Create an empty room with walls
    room = np.zeros((map_msg.info.width, map_msg.info.height), dtype=int)
    room[:, 0] = 100
    room[:, -1] = 100
    room[0, :] = 100
    room[-1, :] = 100

    # Flatten the array in row-major order
    map_msg.data = room.flatten().tolist()
    
    rospy.sleep(1.0)  # Give time for initialization

    map_pub.publish(map_msg)
    rospy.loginfo("Map published.")

    rospy.spin()

if __name__ == "__main__":
    generate_square_room_map()
