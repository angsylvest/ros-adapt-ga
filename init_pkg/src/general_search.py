#! /usr/bin/python3
from math import sqrt, atan2
import rospy
import time 

from geometry_msgs.msg import PoseStamped, Twist, Pose, Point
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from init_pkg.msg import chromosome # will hopefully be used eventually for inter-agent communication 


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
        self.goal_orientation = "" 
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
        pose_int = data.pose.pose.position
        theta_int = data.pose.pose.orientation
        (r, p, y) = euler_from_quaternion([theta_int.x, theta_int.y, theta_int.z, theta_int.w])
        self.roll = r
        self.pitch = p
        self.theta = y

        # self.pose.x = round(pose_int.x, 4)
        # self.pose.y = round(pose_int.y, 4)


    def lidarInfo(self, data):
        lidar_info = data
        if (lidar_info.ranges[0] < 1.0):
            # print("hit object")
            self.isAvoiding = True
        if (lidar_info.ranges[0] >= 1.0):
            self.isAvoiding = False

    def image_callback(self, data):
         image = data  
         # print('image being fed')
         # print(data)
         
    def process_info(self, data): 
    	d = data

    def process_light_sensor_data(self):
        self.light_sensor_one = ""
        self.light_sensor_one = ""
        self.light_sensor_one = ""
        self.light_sensor_one = ""

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

    def moveBackwards(self):
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
    

    def goalBehavior(self): 
    	# found object, will begin moving to centrally located nest 
    	# determine orientation wrt destination 
    	# will then push to center (unless given arms) 
    	pass 
    
    
    def avoidanceBehavior(self):
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
    ## Example tasks that we would make robot do
    
    
    def reorient(self, goal_orientation): 
    	curr_orientation = self.theta 
    	
    	while (abs(curr_orientation - goal_orientation)) > 0.2: # while re_orienting 
		if (self.isAvoiding == False):  # trying to force goal repositioning only when it's safe to do so
		    print('no obstacles .. initiating reorientation toward goal')
		    vel_msg.linear.x = 0
		    # Angular velocity in the z-axis.
		    vel_msg.angular.z = 0.3  # a test to see if slowly reorientating will allow robot to detect change
		    print('angular velocity in reorient ----------------')
		    print(vel_msg.angular.z)
		    # Publishing of vel_msg for Gazebo
		    self.vel_publisher.publish(vel_msg)
		    self.rviz_publisher.publish(vel_msg)

		    # Rate
		    self.rate.sleep()
		    
	    	else: 
	    	    self.avoidanceBehavior()
		    
	# will stop rotating and continue moving 
    	self.moveForwards()	
    
    def initiateCRW(self):
    	step_count = 0 
 
 	current_orientation = 0 # example, will change to actual 
 	prob_dist = [2, 1, 1, 1] # will need to compute properly
 	
 	goal_orientation = random.choices[0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi, 2), [prob_dist]]
	# set hard time limit 
    	while (time.time() - self.init_time >= self.gen_time): 
        	# self.init_time = time.time()
    		self.avoidanceBehavior() # continously check for obstacles 
    		
    		step_count += 1 
    		
    		if step_count == self.step_size: 
    			# re-orient again to another cardinal direction 
		 	prob_dist = [2, 1, 1, 1] # will need to compute properly
 			goal_orientation = random.choices[0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi,2), [prob_dist])
 			self.reorient()
 			step_count = 0 
    			
    		self.rate.sleep()
    		
    		
    def initiateSpiralMove(self):
    	# self.avoidanceBehavior() 
    	
    	
    	
    def initiateBallistic(self): 
    	# self.avoidanceBehavior()
    	


    def moveFromPointAtoPointB(self):
        # same as homework assignment
        global goal_pose
        goal_pose = Point()
        goal_pose.x = float(input('please enter x position'))
        goal_pose.y = float(input('please enter y position'))
        
        # self.goal_publisher.publish(goal_pose) 

        vel_msg = Twist()
        euclidean_distance_curr = self.euclidean_distance(goal_pose, self.pose)

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

            while (self.isReorientating):

                if (self.isAvoiding == False):  # trying to force goal repositioning only when it's safe to do so
                    print('no obstacles .. initiating reorientation toward goal')
                    vel_msg.linear.x = 0
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
        demo_robot.initiateCRW()
        # demo_robot.moveFromPointAtoPointB()

        rospy.spin()
