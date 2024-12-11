#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32 


# Node initialization
rospy.init_node('init_pose')
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 1)

# Construct message
init_msg = PoseWithCovarianceStamped()
init_msg.header.frame_id = "map"

sense_info = ""

def process_info(data):
    global sense_info 

    sense_info = data

pop_subscriber = rospy.Subscriber('light_pub_one', Float32, process_info)


# Get initial pose from Gazebo
# odom_msg = rospy.wait_for_message('/odom', Odometry)
# init_msg.pose.pose.position.x = odom_msg.pose.pose.position.x
# init_msg.pose.pose.position.y = odom_msg.pose.pose.position.y
# init_msg.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
# init_msg.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
# init_msg.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
# init_msg.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

# Delay
# rospy.sleep(1)

# Publish message
rospy.loginfo("setting initial pose")
# pub.publish(init_msg)
rospy.loginfo("initial pose is set")

while not rospy.is_shutdown():
    # print('initalized')
    pass
    # pub.publish(sense_info)
