import rospy
import csv
import time

from your_ros_pkg.msg import YourROSMsg  # Replace with your actual ROS message type

# Callback function for the ROS topic
def callback(data):
    with open('ros_data.csv', mode='a') as file:
        writer = csv.writer(file)
        writer.writerow([data.field1, data.field2, data.field3])  # Replace with the fields in your ROS message

def listener():
    # rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('your_ros_topic', YourROSMsg, callback)  # Replace with your actual ROS topic and message type
    rospy.spin()

if __name__ == '__main__':
    with open('ros_data.csv', mode='w') as file:  # Create a new file or overwrite if it already exists
        writer = csv.writer(file)
        writer.writerow(['Field1', 'Field2', 'Field3'])  # Replace with the fields in your ROS message

    while not rospy.is_shutdown():
        listener()
        time.sleep(1)
