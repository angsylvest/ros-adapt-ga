# initiate collection behavior 

# we are assuming that collectable objects have a narrowerer range (which we want to exploit 
# to soley use the lidar readings)

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class RobotBehavior:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('robot_behavior')

        # Define thresholds (adjust these as needed)
        self.obstacle_distance_threshold = 1.0  # Obstacle is within 1 meter
        self.collectable_distance_threshold = 2.0  # Collectable object within 2 meters
        
        # Subscribe to the /scan topic
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
    def scan_callback(self, msg, front_angle_range=(-np.pi/4, np.pi/4)):
        """
        Callback function for processing LiDAR data.
        """
        # Get the range data from the LaserScan message
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        # print(len(ranges))

        print(f'angle_min: {angle_min}, angle_max: {angle_max}, angle_increment: {angle_increment}')
        
        # Calculate the start and end indices for the front-facing angles
        start_angle = front_angle_range[0] - angle_min  # angle offset from angle_min
        end_angle = front_angle_range[1] - angle_min    # angle offset from angle_min

        # Calculate indices in the range array
        start_index = int(np.floor(start_angle / angle_increment))
        end_index = int(np.floor(end_angle / angle_increment))

        # Debugging output to check calculations
        print(f'front_angle_range: {front_angle_range}')
        print(f'start_angle: {start_angle}, end_angle: {end_angle}')
        print(f'start_index: {start_index}, end_index: {end_index}')
        
        # Check the indices to ensure they fall within the valid range of the 'ranges' array
        start_index = max(0, start_index)  # Ensure it doesn't go below 0
        end_index = min(len(ranges)-1, end_index)  # Ensure it doesn't go beyond the length of ranges
        
        # Get the front-facing range data (subset of the ranges list)
        front_ranges = ranges[start_index:end_index]
        
        # Process the front-facing LiDAR data to differentiate obstacles and collectable objects
        obstacle_count = 0
        collectable_count = 0
        total_count = len(front_ranges)

        for i, distance in enumerate(front_ranges):
            # Classify based on distance
            if distance < self.obstacle_distance_threshold:
                # This is an obstacle
                obstacle_count += 1
            elif self.obstacle_distance_threshold <= distance < self.collectable_distance_threshold:
                # This is a collectable object
                collectable_count += 1

        # Calculate the proportion of obstacles and collectable objects in the front-facing range
        obstacle_proportion = obstacle_count / total_count if total_count > 0 else 0
        collectable_proportion = collectable_count / total_count if total_count > 0 else 0

        # Decision making based on proportions
        if obstacle_proportion > 0.5:  # More than 50% of the front range is an obstacle
            print(f"Obstacle detected in front. Obstacle proportion: {obstacle_proportion:.2f}")
        elif collectable_proportion > 0.1:  # If there’s a small proportion of collectable objects
            print(f"Collectable object detected. Collectable proportion: {collectable_proportion:.2f}")
        else:
            print(f"Nothing detected or detected too far for action.")

if __name__ == '__main__':
    try:
        robot_behavior = RobotBehavior()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
