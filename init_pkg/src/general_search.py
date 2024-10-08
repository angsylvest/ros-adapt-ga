#! /usr/bin/python3
from math import sqrt, atan2
import math
import rospy
import time 

from geometry_msgs.msg import PoseStamped, Twist, Pose, Point
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from init_pkg.msg import chromosome # will hopefully be used eventually for inter-agent communication 
# from msg import chrosome
import random 

class DemoRobot:
    def __init__(self):
        self.pose = Point()
        # self.goal_pose = Point()
        self.lidar_info = LaserScan()

        self.isAvoiding = False
        self.isReorienting = False
        self.roll = 0
        self.pitch = 0
        self.theta = 0
        
        self.gen_time = 30 # in sec 
        
        self.curr_time = time.time()
        self.init_time = time.time()

        # INIT NODE
        rospy.init_node('simulation', anonymous=False)

        # SET UP PUBLISHERS AND SUBSCRIBERS
        # -----------------------------------------------------------
        # accept navigation goal from RVis on the topic move_base_simple/goal

        # ultrasonic strategically placed to determine when obstacle vs retrievable item 
        
        # publisher for RViz
        self.rviz_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # publisher for Gazebo
        self.vel_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        
        # current pose of robot
        self.pose_subscriber = rospy.Subscriber('/odom',
                                           Odometry, self.update_pose)
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.lidarInfo) 
        # subscriber to image
        self.image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        
        # group level sharing publisher and subscriber 
        # self.pop_publisher = rospy.Publisher('/chrome-fit', chromosome, queue_size=50)
        # self.pop_subscriber = rospy.Subscriber('/chrome-info', chromosome, self.process_info)
        
        self.light_pub = ""
        self.light_sub = ""
        
        self.behavior_set = ["crw", "spiral", "ballistic"]
        
        # navigation-specific info 
        self.goal_orientation = 0 
        self.step_size = 1200 # after approx 2 sec, want to switch direction (based off self.rate)	
	
        self.rate = rospy.Rate(10) # 10x per sec 

    # Helper functions
    def euclidean_distance(self, goal_pose, pose):
        """Euclidean distance between current pose and the goal."""
        goal_pose.x = int(goal_pose.x)
        goal_pose.y = int(goal_pose.y)

        return sqrt(pow((goal_pose.x - pose.x), 2) +
                    pow((goal_pose.y - pose.y), 2))

    def update_goal(self, data):
        goal_pose_int = data.pose.position
        # print('goal has been updated') 
        # self.goal_pose.x = goal_pose_int.x
        # self.goal_pose.y = goal_pose_int.y

    def update_pose(self, data):

        global goal_pose 
        self.pose = data.pose.pose.position

        theta_int = data.pose.pose.orientation
        (r, p, y) = euler_from_quaternion([theta_int.x, theta_int.y, theta_int.z, theta_int.w])
        self.roll = r
        self.pitch = p
        self.theta = y

    def lidarInfo(self, data):
        lidar_info = data
        if (lidar_info.ranges[0] < 0.3):
            # print("hit object", lidar_info.ranges[0])
            self.isAvoiding = True
        else: 
            self.isAvoiding = False 
        # if (lidar_info.ranges[0] >= 1.0):
        #     self.isAvoiding = False

    def image_callback(self, data):
         image = data  
         # print('image being fed')
         # print(data)
         
    def process_info(self, data): 
        d = data

    def process_light_sensor_data(self):
        self.light_sensor_one = ""
        self.light_sensor_two = ""
        self.light_sensor_three = ""
        self.light_sensor_four = ""

    ## Make abilities more easy to comprehend (more straightforward)

    def moveRight(self):
        vel_msg = Twist()
        # Linear velocity in the x-axis.
        vel_msg.linear.x = 0.2
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0.2

        # Publishing our vel_msg for RVis
        self.vel_publisher.publish(vel_msg)
        self.rviz_publisher.publish(vel_msg)
       

    def goalBehavior(self):
        # TODO: will have to troubleshoot 
        LIGHT_THRESHOLD = 100 

        sense_val = [self.light_sensor_one, self.light_sensor_two, self.light_sensor_three, self.light_sensor_four] 
        max_index = sense_val.index(max(sense_val))

        goal_orient = 0 
        self.reorient(goal_orient)

        while max(sense_val) > LIGHT_THRESHOLD: 
            self.moveBackwards()
            self.rate.sleep()

        return True # task successful, will continue exploring again 


    def moveLeft(self):
        vel_msg = Twist()
        # Linear velocity in the x-axis.
        vel_msg.linear.x = 0.2
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = -0.2

        # Publishing our vel_msg for RVis
        self.vel_publisher.publish(vel_msg)
        self.rviz_publisher.publish(vel_msg)

    def moveBackwards(self, duration = 10):
        vel_msg = Twist()
        # Linear velocity in the x-axis.
        vel_msg.linear.x = -0.3
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # Publishing our vel_msg for RVis
        self.vel_publisher.publish(vel_msg)
        self.rviz_publisher.publish(vel_msg)


    def moveForwards(self):
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
            # Publishing our vel_msg for RVis
            self.vel_publisher.publish(vel_msg)
            self.rviz_publisher.publish(vel_msg)


    def stop(self):
        vel_msg = Twist()
        # Linear velocity in the x-axis.
        vel_msg.linear.x = 0.3
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # Publishing our vel_msg for RVis
        self.vel_publisher.publish(vel_msg)
        self.rviz_publisher.publish(vel_msg)

        
    def shutdown(self):
        print('successfully reached goal .. shutting down') 
    
    
    def avoidanceBehavior(self, goal_orientation=None):
        vel_msg = Twist()
        
        # Constants for movement
        BACKWARD_SPEED = -0.1
        TURN_SPEED = 0.2
        MAX_DURATION = 5  # Maximum time to avoid
        start_time = self.get_time()

        while self.isAvoiding:
            print('Initiating obstacle avoidance behavior')

            # Set linear and angular velocities
            vel_msg.linear.x = BACKWARD_SPEED  # Move slightly backward
            vel_msg.angular.z = TURN_SPEED      # Rotate to avoid the obstacle
            
            # Publish the velocity message
            self.vel_publisher.publish(vel_msg)
            self.rviz_publisher.publish(vel_msg)
            
            # Check if enough time has passed
            if self.get_time() - start_time > MAX_DURATION:
                print('Avoidance duration exceeded, stopping avoidance behavior')
                self.isAvoiding = False
            
            self.rate.sleep()

        # Once avoidance is complete, reorient if a goal orientation is provided
        if goal_orientation:
            print('Completed avoidance, reorienting to:', goal_orientation)
            self.reorient(goal_orientation)

    def get_time(self):
        # Implement a method to return current time in seconds
        return rospy.get_time()  # or any timekeeping method you're using

    # def avoidanceBehavior(self, goal_orientation = None):
    #     vel_msg = Twist()
    #     while (self.isAvoiding):
    #         print('initiating obstacle behavior')
	# 	   # Linear velocity in the x-axis.
    #         vel_msg.linear.x = -0.1  # slight movement backward along the obstacle
    #         # Angular velocity in the z-axis.
    #         vel_msg.angular.z = 0.2  # intended to rotate robot away from obstacle
    #         # Publishing of vel_msg for Gazebo
    #         self.vel_publisher.publish(vel_msg)
    #         self.rviz_publisher.publish(vel_msg)
    #         # only updates after each iteration
    #         # euclidean_distance_curr = self.euclidean_distance(goal_pose, self.pose)

    #         self.rate.sleep()


    #     if goal_orientation:
    #         print('completed avoidance reorienting to: ', goal_orientation)
    #         self.reorient(goal_orientation)
    # ## Example tasks that we would make robot do
    
    
    def reorient(self, goal_orientation): 
        curr_orientation = self.theta 
        vel_msg = Twist()
        print('orienting to --', goal_orientation)

        if isinstance(goal_orientation, list):
            goal_orientation = goal_orientation[0]

        while (abs(curr_orientation - goal_orientation)) > 0.2: # while re_orienting 
            if (self.isAvoiding == False):  # trying to force goal repositioning only when it's safe to do so
                print('no obstacles .. initiating reorientation toward goal')
                vel_msg.linear.x = 0
                # Angular velocity in the z-axis.
                vel_msg.angular.z = 0.3  # a test to see if slowly reorientating will allow robot to detect change
                # print('angular velocity in reorient ----------------')
                # print(vel_msg.angular.z)
                # Publishing of vel_msg for Gazebo
                self.vel_publisher.publish(vel_msg)
                self.rviz_publisher.publish(vel_msg)

                curr_orientation = self.theta 
                # Rate
                self.rate.sleep()
                
            else: 
                self.avoidanceBehavior()
		    
	# will stop rotating and continue moving 
        self.moveForwards()	
    
    def generatePath(self, curr_orientation): 
        # will need to update since we are assuming some amount of error (potentially)
        poss_orientations = [0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi, 2)]
        curr_index = poss_orientations.index(curr_orientation)

        return poss_orientations[curr_index: ] + poss_orientations[:curr_index]


    def getProbDistrib(self, curr_orientation):
        poss_orientations = [0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi, 2)]
        dist = [1 for i in range(4)]
        print(dist)
        
        for i in range(len(dist)):
            if poss_orientations[i] == round(curr_orientation, 2):
                dist[i] += 1
        
        return dist, poss_orientations  # higher preference for current orientation
    
    def testMovement(self, route=None):
        if route == None: 
            route = [1.57, 0, -1.57, 3.14]
        
        step_count = 500 
        current_orientation = self.theta

        for goal_orientation in route: 
            print(f'orienting towards: {goal_orientation}')
            self.reorient(goal_orientation)
            while (time.time() - self.init_time <= self.gen_time): 
                step_count += 1 
            
                if step_count == self.step_size: 
                    step_count = 0 
                    print(f'exiting, next orientation')
                    break 

                self.rate.sleep()




    def initiateCRW(self):
        step_count = 500 
 
        current_orientation = self.theta
        prob_dist, _  = self.getProbDistrib(current_orientation)

        goal_orientation = random.choices([0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi, 2)], prob_dist)

        print("beginning crw ", "prob dist: ", prob_dist, "goal orientation possibility", goal_orientation)
        
        self.reorient(goal_orientation)
        # set hard time limit 
        print(self.gen_time, time.time(), self.init_time)
        while (time.time() - self.init_time <= self.gen_time): 
            # self.init_time = time.time()
            self.avoidanceBehavior(goal_orientation) # continously check for obstacles 
            
            step_count += 1 
            
            if step_count == self.step_size: 
                # re-orient again to another cardinal direction 
                prob_dist = [2, 1, 1, 1] # will need to compute properly
                print('step count == step_size: ', prob_dist, [0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi,2)])
                goal_orientation = random.choices([0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi,2)], prob_dist)
                self.reorient(goal_orientation)
                step_count = 0 
                
            self.rate.sleep()

        print('crw complete --')

        
    		
    		
    def initiateSpiralMove(self):
        step_count = 0 
 
        current_orientation = self.theta
        prob_dist = self.generatePath(current_orientation)
        print('generated prob dist: ', prob_dist)
        init_index = 0 
        
        self.reorient(prob_dist[0])
        # set hard time limit 
        while (time.time() - self.init_time <= self.gen_time and init_index <= len(prob_dist)): 
            # self.init_time = time.time()
            goal_orientation = prob_dist[init_index]
            self.avoidanceBehavior(goal_orientation) # continously check for obstacles 
            
            step_count += 1 
            
            if step_count == self.step_size: 
                # re-orient again to another cardinal direction 
                init_index+=1
                goal_orientation = prob_dist[init_index]
                self.reorient(goal_orientation)
                step_count = 0 
                
            self.rate.sleep()
    
    	
    	
    def initiateBallistic(self): 
        step_count = 500 
 
        current_orientation = self.theta
        prob_dist, _  = self.getProbDistrib(current_orientation)

        goal_orientation = random.choices([0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi, 2)], prob_dist)

        print("beginning crw ", "prob dist: ", prob_dist, "goal orientation possibility", goal_orientation)
        
        self.reorient(goal_orientation)
        # set hard time limit 
        print(self.gen_time, time.time(), self.init_time)
        while (time.time() - self.init_time <= self.gen_time): 
            # self.init_time = time.time()
            self.avoidanceBehavior(goal_orientation) # continously check for obstacles 
            
            step_count += 1 
            
            if step_count == self.step_size: 
                # re-orient again to another cardinal direction 
                prob_dist = [2, 1, 1, 1] # will need to compute properly
                print('step count == step_size: ', prob_dist, [0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi,2)])
                goal_orientation = random.choices([0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi,2)], prob_dist)
                self.reorient(goal_orientation)
                step_count = 0 
                
            self.rate.sleep()
    	
    def calculate_angle(self, pose, goal):
        # pos initialization 
        x1 = pose.x
        x2 = goal.x

        y1 = pose.y
        y2 = pose.y

        # want to use light source 
        dx = x2 - x1
        dy = y2 - y1
        angle = math.atan2(dy, dx)
        # Convert angle to degrees if desired
        # angle_degrees = math.degrees(angle)
        return angle

    def moveFromPointAtoPointB(self, goal):
        # self.goal_publisher.publish(goal_pose) 

        vel_msg = Twist()
        euclidean_distance_curr = self.euclidean_distance(goal, self.pose)
        
        home_orientation = self.calculate_angle(self.pose, goal)

        while (euclidean_distance_curr > 0.1):

            while (self.isAvoiding):
                print('initiating obstacle behavior')
                # Linear velocity in the x-axis.
                vel_msg.linear.x = -0.3  # slight movement backward along the obstacle
                # Angular velocity in the z-axis.
                vel_msg.angular.z = 0.33  # intended to rotate robot away from obstacle
                # Publishing of vel_msg for Gazebo
                self.vel_publisher.publish(vel_msg)
                self.rviz_publisher.publish(vel_msg)
                # only updates after each iteration
                euclidean_distance_curr = self.euclidean_distance(goal_pose, self.pose)

                self.rate.sleep()

            while (round(self.theta,2) != home_orientation):

                if (self.isAvoiding == False):  # trying to force goal repositioning only when it's safe to do so
                    print('no obstacles .. initiating reorientation toward goal')
                    vel_msg.linear.x = 0.0
                    # Angular velocity in the z-axis.
                    vel_msg.angular.z = 0.3  # a test to see if slowly reorientating will allow robot to detect change
                    print('angular velocity in reorient ----------------')
                    print(vel_msg.angular.z)
                    # Publishing of vel_msg for Gazebo
                    self.vel_publisher.publish(vel_msg)
                    self.rviz_publisher.publish(vel_msg)

                    euclidean_distance_curr = self.euclidean_distance(goal_pose, self.pose)

                    # Rate
                    self.rate.sleep()

            if (self.isAvoiding == False):
                self.moveForwards()
                euclidean_distance_curr = self.euclidean_distance(goal_pose, self.pose)
                self.rate.sleep()

        self.stop()
        # rospy.on_shutdown(shutdown)
        print('successful run')


## potentially use different bug algorithms to demo to students

if __name__ == '__main__':
    ## Put your functions here to start
    while not rospy.is_shutdown():
        demo_robot = DemoRobot()
        print('robot init!')
        demo_robot.testMovement()
        # demo_robot.initiateCRW()
        # demo_robot.initiateSpiralMove()
        # demo_robot.initiateBallistic
        # demo_robot.moveFromPointAtoPointB(goal=demo_robot.pose)

        rospy.spin()
