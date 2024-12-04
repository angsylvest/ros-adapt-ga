#! /usr/bin/python3
from math import sqrt, atan2
import math
import rospy
import time 
import numpy as np 

from geometry_msgs.msg import PoseStamped, Twist, Pose, Point
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import random 
from std_msgs.msg import Float32, Bool

# ga relevant 
from adapt_ga.ga import GA
from adapt_ga.strategy import Strategy


class DemoRobot:
    def __init__(self):
        # keep track of robot status info 
        self.pose = Point()
        # self.lidar_info = LaserScan()

        # store initial position info (for homing behavior)
        self.initial_position = None
        self.curr_pos = None 

        self.isHoming = False 
        self.isAvoiding = False
        self.isReorienting = False
        self.inContact = False 
        self.roll = 0
        self.pitch = 0
        self.theta = 0
        
        # adapt-ga specific params 
        self.gen_time = 30 # in sec 
        self.chrm = GA() # NOTE: in sim, may just receive apriori
        # self.chrm.curr_genotype = ""
        self.processed_chrm = self.chrm.process_chromosome(self.chrm.curr_genotype)
        self.partner_chrm = "" # read from file here as well 

        # read from file here: 
        with open('/home/angelsylvester/catkin_ws/src/ros-opi-ga/init_pkg/src/adapt_ga/processed_chrm.txt', 'r') as file:
        # with open('./init_pkg/processed_chrm.txt', 'r') as file:   
            lines = file.readlines()
            self.chrm.curr_genotype = lines[0]
            self.processed_chrm = self.chrm.process_chromosome(self.chrm.curr_genotype)
            self.partner_chrm = lines[1] # read from file here as well 

        self.partner_fit = 0 
        self.st = Strategy()
        self.st.update_from_chrom(self.processed_chrm)
        self.st_gen = self.st.resample(curr_dir=0.00)
        self.fitness = 0 
        self.num_collected = 0 
        self.num_collisions = 0 

        self.obstacle_distance_threshold = 1.0
        self.collectable_distance_threshold = 2.0  

        # keep track of time elapsed 
        self.start_time = time.time()
        self.curr_time = time.time()
        self.init_time = time.time()

        # INIT NODE
        rospy.init_node('robot_1', anonymous=False)

        robot_group = "" 

        # SET UP PUBLISHERS
        self.cmd_vel_pub = rospy.Publisher(f'{robot_group}/cmd_vel', Twist, queue_size=10)
        self.cnt_pub = rospy.Publisher(f'{robot_group}/contact', Bool, queue_size=10)
        self.fit_pub = rospy.Publisher(f'{robot_group}/fit_', Float32, queue_size=10)
        print('publishers set up')

        # SET UP SUBSCRIBERS
        self.pose_subscriber = rospy.Subscriber(f'{robot_group}/odom',
                                           Odometry, self.update_pose)
        self.lidar_subscriber = rospy.Subscriber(f'{robot_group}/scan', LaserScan, self.lidar_info) 
        self.cnt_sub = rospy.Subscriber(f'{robot_group}/contact', Bool, self.contact_info) 
        self.cnt_sub = rospy.Subscriber(f'{robot_group}/fit_', Float32, self.update_partner) 

        print('subscribers set up')

        # navigation-specific info 
        self.goal_orientation = 0 
        self.step_size = 120  
	
        self.rate = rospy.Rate(10) # 10x per sec 

    def update_partner(self, data): 
        self.partner_fit = data 

    def update_pose(self, data): 
        if (time.time() - demo_robot.start_time) % self.gen_time == 0: 
            self.chrm.reproduce() 
            self.processed_chrm = self.chrm.process_chromosome(self.chrm.curr_genotype)
            self.st.update_from_chrom(self.processed_chrm)

        if (time.time() - demo_robot.start_time % 1 == 0):
            self.st.time_exploring_with_strat += 1 

        self.curr_pos = data.pose.pose 
        theta_int = data.pose.pose.orientation
        (r, p, y) = euler_from_quaternion([theta_int.x, theta_int.y, theta_int.z, theta_int.w])
        self.roll = r
        self.pitch = p
        self.theta = y

        if self.initial_position is None:
            # Store the initial position when the robot first starts
            self.initial_position = data.pose.pose
            rospy.loginfo("Initial position stored: %s", self.initial_position)
 

    def lidar_info(self, msg, front_angle_range=(-np.pi/4, np.pi/4)): 
        # Get the range data from the LaserScan message
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        
        # Calculate the start and end indices for the front-facing angles
        start_angle = front_angle_range[0] - angle_min  # angle offset from angle_min
        end_angle = front_angle_range[1] - angle_min    # angle offset from angle_min

        # Calculate indices in the range array
        start_index = int(np.floor(start_angle / angle_increment))
        end_index = int(np.floor(end_angle / angle_increment))

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
        if obstacle_proportion > 0.9: 
            print(f"Obstacle detected in front. Obstacle proportion: {obstacle_proportion:.2f}")
            self.isAvoiding = True 
        elif collectable_proportion > 0.6: 
            print(f"Collectable object detected. Collectable proportion: {collectable_proportion:.2f}")
            self.isHoming = True 
        else:
            # print(f"Nothing detected or detected too far for action.")
            self.isAvoiding = False 

    def contact_info(self, data): 
        if data: # if True, contact is happening
            self.inContact = True
            # self.chrm.best_encountered_fitness = self.partner_fit
            self.chrm.update_best_encountered(self.partner_chrm, self.partner_fit)

        else: 
            self.inContact = False 

    def homing_behavior(self): 
        # Start homing behavior (move back to initial position)
        if self.initial_position:
            rospy.loginfo("Homing to initial position...")
                    # Define the rate of publishing messages
        rate = rospy.Rate(10)
        distance = float('inf')
        self.num_collected += 1 
        self.fitness = self.chrm.update_fitness(self.num_collected, self.num_collisions)
        self.fit_pub.publish(self.fitness)

        # Loop to continuously send commands to move towards the target position
        while not rospy.is_shutdown():
            if self.initial_position:
                move_cmd = Twist()

                while distance > 0.1:  # Continue until the robot is close enough to the target position
                    # Get current position from the odometry
                    if not self.isAvoiding:
                        current_pose = self.pose.position
                        target_pose = self.initial_position

                        distance = ((current_pose.x - target_pose.position.x) ** 2 +
                                    (current_pose.y - target_pose.position.y) ** 2) ** 0.5

                        if distance < 0.5:  # threshold to stop
                            rospy.loginfo("Reached the initial position!")
                            move_cmd.linear.x = 0
                            move_cmd.angular.z = 0
                            self.cmd_vel_pub.publish(move_cmd)
                            break  # Exit loop once the robot has reached the goal
                        else:
                            dx = target_pose.position.x - current_pose.x
                            dy = target_pose.position.y - current_pose.y
                            target_angle = math.atan2(dy, dx)
                            angle_diff = target_angle - self.theta # current_pose.theta
                            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
                            if abs(angle_diff) > 0.1:  # Threshold to start turning
                                move_cmd.linear.x = 0  # Stop moving forward
                                move_cmd.angular.z = 0.2 * angle_diff  # Turn towards the target
                            else:
                                # Move forward towards the target
                                move_cmd.linear.x = 0.2  # Move forward at 0.2 m/s
                                move_cmd.angular.z = 0  # No rotation

                            # Publish the move command
                            self.cmd_vel_pub.publish(move_cmd)
                    else: 
                        self.avoidance_behavior()

            rate.sleep()
        else:
            rospy.logwarn("Initial position not set. Make sure the robot has moved to set it.")

    def get_time(self):
        # Implement a method to return current time in seconds
        return rospy.get_time()  # or any timekeeping method you're using
    
    def avoidance_behavior(self, goal_orientation=None): 
        vel_msg = Twist()
        self.num_collisions += 1 
        self.fitness = self.chrm.update_fitness(self.num_collected, self.num_collisions)
        self.fit_pub.publish(self.fitness)

        # Constants for movement
        BACKWARD_SPEED = -0.1
        TURN_SPEED = 0.2
        MAX_DURATION = 5  # Maximum time to avoid
        start_time = self.get_time()

        while self.isAvoiding:
            # print('Initiating obstacle avoidance behavior')

            # Set linear and angular velocities
            vel_msg.linear.x = BACKWARD_SPEED  # Move slightly backward
            vel_msg.angular.z = TURN_SPEED      # Rotate to avoid the obstacle
            
            # Publish the velocity message
            self.cmd_vel_pub.publish(vel_msg)
            
            # Check if enough time has passed
            if self.get_time() - start_time > MAX_DURATION:
                print('Avoidance duration exceeded, stopping avoidance behavior')
                self.isAvoiding = False
            
            self.rate.sleep()

        # Once avoidance is complete, reorient if a goal orientation is provided
        if goal_orientation:
            print('Completed avoidance, reorienting to:', goal_orientation)
            self.reorient(goal_orientation) 

    def search_behavior(self, route=None): 
        if route == None: 
            route = [1.57, 0, -1.57, 3.14]
        
        step_count = 500 
        current_orientation = self.theta

        for goal_orientation in route: 
            print(f'orienting towards: {goal_orientation}')
            self.reorient(goal_orientation)
            while (time.time() - self.init_time <= self.gen_time): 
                step_count += 1 
            
                print(f'step count: {step_count} vs step size: {self.step_size}')
                if step_count > self.step_size: 
                    step_count = 0 
                    print(f'exiting, next orientation')
                    break 

                if self.isAvoiding:
                    print(f'entering avoidance state')
                    self.avoidance_behavior()

                elif self.isHoming:
                    self.homing_behavior()

                self.rate.sleep()

    def reorient(self, goal_orientation): 
        curr_orientation = self.theta 
        vel_msg = Twist()
        print('orienting to --', goal_orientation)

        if isinstance(goal_orientation, list):
            goal_orientation = goal_orientation[0]

        while (abs(curr_orientation - goal_orientation)) > 0.2: # while re_orienting 
            if (self.isAvoiding == False):  # trying to force goal repositioning only when it's safe to do so
                vel_msg.linear.x = 0
                # Angular velocity in the z-axis.
                vel_msg.angular.z = 0.3  
                self.cmd_vel_pub.publish(vel_msg)
                curr_orientation = self.theta 
                # Rate
                self.rate.sleep()
                
            else: 
                print(f'entering avoidance behavior in reorient()')
                self.avoidance_behavior()
		    
	# will stop rotating and continue moving 
        self.move_forwards()

    def move_forwards(self):
        vel_msg = Twist()
        # Linear velocity in the x-axis.
        vel_msg.linear.x = 0.3
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # move slightly forward for set period of time 
        init_time = time.time()
        while (time.time() - init_time <= 5):  
            if self.isAvoiding: 
                print(f'moving straight but entering avoidance behavior')
                self.avoidance_behavior()
            # Publishing our vel_msg for RVis
            self.cmd_vel_pub.publish(vel_msg)


if __name__ == '__main__':
    ## Put your functions here to start
    while not rospy.is_shutdown():
        demo_robot = DemoRobot()
        strategy_generated = demo_robot.st.resample(curr_dir=0.00) # will want to eventually use curr orient
        route = demo_robot.st_gen
        demo_robot.search_behavior(route=route)


        
    
 

    