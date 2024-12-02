#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class HomingBehavior:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('homing_behavior', anonymous=True)

        # Create an action client for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Wait for the action server to start
        rospy.loginfo("Waiting for move_base action server to start...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        # Define the home position (set to the initial start location of the robot)
        # In a real scenario, this would be dynamic based on your robot's initial position
        self.home_position = PoseStamped()
        self.home_position.header.frame_id = "map"
        self.home_position.pose.position.x = 0.0  # X coordinate of home position
        self.home_position.pose.position.y = 0.0  # Y coordinate of home position
        self.home_position.pose.orientation.w = 1.0  # No rotation

        # Define a flag to indicate when the object has been collected
        self.object_collected = False

        # Simulate object collection trigger (in reality, this could come from a sensor callback)
        rospy.Timer(rospy.Duration(5), self.collect_object)  # Simulate object collection after 5 seconds

    def collect_object(self, event):
        """Simulate the object collection process"""
        rospy.loginfo("Object collected. Initiating homing behavior.")
        self.object_collected = True

        # Once the object is collected, trigger homing
        self.initiate_homing()

    def initiate_homing(self):
        """Move the robot back to the home position."""
        if self.object_collected:
            rospy.loginfo("Starting to return to home position.")

            # Create a MoveBase goal for the home position
            goal = MoveBaseGoal()
            goal.target_pose = self.home_position

            # Send the goal to move_base
            self.client.send_goal(goal)

            # Wait for the robot to reach the home position
            success = self.client.wait_for_result(rospy.Duration(60))  # Wait up to 60 seconds

            if success:
                rospy.loginfo("Successfully returned to home position!")
            else:
                rospy.logwarn("Failed to return to home position.")

        else:
            rospy.logwarn("Object not collected yet. Cannot initiate homing.")

if __name__ == '__main__':
    try:
        # Instantiate the HomingBehavior class
        homing_behavior = HomingBehavior()

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Homing behavior node interrupted.")
