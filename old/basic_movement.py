#! /usr/bin/python3
from math import sqrt, atan2
import rospy
import time 
import math 

from math import pi 
import random 

from geometry_msgs.msg import PoseStamped, Twist, Pose, Point
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from init_pkg.msg import chromosome 

from std_msgs.msg import Float32 


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
        self.pop_publisher = rospy.Publisher('/chrome_fit', chromosome, queue_size=50)
        self.pop_subscriber = rospy.Subscriber('/chrome_fit', chromosome, self.process_info)
        
        
        self.ultrasonic_val = rospy.Subscriber('/ultrasonic', Float32, self.process_ultrasonic)
        
        self.light_pub = ""
        self.light_sub = ""
        
        self.behavior_set = ["crw", "spiral", "ballistic"]
        
        # navigation-specific info 
        self.goal_orientation = 0 
        self.current_orientation = 0 
        self.step_size = 1200 # after approx 2 sec, want to switch direction (based off self.rate)	
        self.homing = False 
	
        self.current_pos = (0, 0, 0)
        self.rate = rospy.Rate(10) # 10x per sec 

    # Helper functions
    def euclidean_distance(self, goal_pose, pose):
        """Euclidean distance between current pose and the goal."""
        (goal_x, goal_y, goal_z) = goal_pose 
        (pose_x, pose_y, pose_z) = pose 
        
        # goal_pose.x = int(goal_pose.x)
        # goal_pose.y = int(goal_pose.y)

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
        
        self.current_pos = pose_int
        
        self.roll = r
        self.pitch = p
        self.theta = y

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
        print('incoming chromosome msg published --', data)
        d = data

    def process_light_sensor_data(self):
        self.light_sensor_one = ""
        self.light_sensor_two = ""
        self.light_sensor_three = ""
        self.light_sensor_four = ""

    ## Make abilities more easy to comprehend (more straightforward)
    
    def process_ultrasonic(self, data):
        ultra_val = data 
        ultra_thres = 100
        dist_thres = 100
        
        if data > ultra_thres and not self.isAvoiding and math.sqrt(x^2 + y^2) > dist_thres:
            self.homing = True
        else: 
            self.homing = False
             

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
        vel_msg.linear.x = 0.2
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
        
        
    def homing_behavior(self):
        # homing behavior: 
        # 1. check ultrasonic value 
        # 2. reorient to 0,0 
        # 3. forward until within 0,0
        # 4. select new orientation (leaving sphere) 
        print('homing behavior, current pos', self.current_pos)
        (x, y, z) = self.current_pos
        dist_thrs = 0.05
        
        orientation_goal = (5,5,0)
        
        self.moveFromPointAtoPointB(orientation_goal)
        
        while math.sqrt(x^2 + y^2) > dist_thrs: 
             self.moveFromPointAtoPointB(orientation_goal)
         
        print('finished homing')
        
        
    def distribute_chromosome(self, reward, penalty, speed, threshold, fitness):
    ################ ----------chromosome pub------------ ############
        incoming_msg = chromosome()
        incoming_msg.speed = str(speed)
        incoming_msg.reward = str(reward)
        incoming_msg.penalty = str(penalty)
        incoming_msg.threshold = str(threshold)
        incoming_msg.fitness = float(fitness)
        
        print('attempting to publish --', incoming_msg)
        self.pop_publisher.publish(incoming_msg) 

    ################# ---------------------- ############
    	


    # calculates angle normal to current orientation 
    def calc_normal(self, curr_angle): 

        if (curr_angle + round(pi/2, 2) <= round(pi, 2) and curr_angle <= round(pi, 2) and curr_angle >= 0): 
            return round(curr_angle + round(pi/2, 2), 2)
    
        elif (curr_angle + round(pi/2, 2) > round(pi, 2) and curr_angle < round(pi, 2) and curr_angle > 0): 
            diff = round(pi/2, 2) - (round(pi,2) - curr_angle) 
            return round((-1*round(pi/2, 2) + diff),2)
    
        elif (curr_angle + round(pi/2, 2) < 0 and curr_angle < 0): 
            return round((-1*round(pi, 2) + curr_angle + round(pi/2, 2)),2)
        
        elif (curr_angle + round(pi/2, 2) >= 0 and curr_angle <= 0): 
            diff = abs(round(pi/2, 2) - curr_angle) 
            return round(diff,2) 
        
    
    def avoidanceBehavior(self):
        vel_msg = Twist()
        print('checking obstacles --', self.isAvoiding)
        while (self.isAvoiding):
            print('initiating obstacle behavior')
            
            # Linear velocity in the x-axis.
            vel_msg.linear.x = -0.3  # slight movement backward along the obstacle
            # Angular velocity in the z-axis.
            # vel_msg.angular.z = 0.33  # intended to rotate robot away from obstacle
            # Publishing of vel_msg for Gazebo
            self.vel_publisher.publish(vel_msg)
            self.rviz_publisher.publish(vel_msg)

            self.rate.sleep()
        
        print('reorienting normal to obstacle', self.theta)   
         
        self.reorient(self.calc_normal(self.theta)) # reorient normal to current orientation 
  
        init_time = time.time()    
        while (time.time() - init_time < 3): 
            self.moveForwards()
            self.rate.sleep() 
    ## Example tasks that we would make robot do
    
    
    def reorient(self, goal_orientation): 
        vel_msg = Twist()
        curr_orientation = round(self.theta,2) 
        print('beginning reorienting behavior towards goal', curr_orientation, goal_orientation) 
    	
        while (abs(curr_orientation - round(goal_orientation,2))) > 0.2:
         # while re_orienting
            curr_orientation = round(self.theta,2) 
            if (self.isAvoiding == False):  # trying to force goal repositioning only when it's safe to do so
                vel_msg.linear.x = 0
                # Angular velocity in the z-axis.
                vel_msg.angular.z = 0.2
                # Publishing of vel_msg for Gazebo
                self.vel_publisher.publish(vel_msg)
                self.rviz_publisher.publish(vel_msg)

                # Rate
                self.rate.sleep()
                
            else: 
                self.avoidanceBehavior()
                
        self.current_orientation = round(goal_orientation,2)
		    
    
    def getProbDistrib(self, curr_orientation): 
        poss_orientations = [0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi, 2)]
        dist = [1 for i in range(4)]
        return [dist[i] + 1 if poss_orientations[i] == round(curr_orientation, 2) else dist[i] for i in range(len(dist))]
 # higher pref for curr orientation  


    def generatePath(self, curr_orientation):
        # will need to update since we are assuming some amount of error (potentially)
        poss_orientations = [0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi, 2)]
        curr_index = poss_orientations.index(curr_orientation)
        return poss_orientations[curr_index: ] + poss_orientations[:curr_index]
        
        
    def initiateCRW(self):
        step_count = 0 
 
        current_orientation = self.theta
        prob_dist = self.getProbDistrib(current_orientation) # TODO: this is wrong (should be for spiral)
        print('current prob dist --', prob_dist) 
        
        print(len(prob_dist), len([0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi, 2)]))
        goal_orientation = random.choices([0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi, 2)], prob_dist)[0]
        
        self.reorient(goal_orientation)
        # set hard time limit 
        while (time.time() - self.init_time <= self.gen_time): 
            print('time left --', time.time() - self.init_time)
            # self.init_time = time.time()
            self.avoidanceBehavior() # continously check for obstacles 
                
            # will stop rotating and continue moving 
            print('now proceeding forward')
            self.moveForwards()	
            step_count += 1 
                
            if step_count == self.step_size: 
                # re-orient again to another cardinal direction 
                prob_dist = [2, 1, 1, 1] # will need to compute properly
                goal_orientation = random.choices([0, round(math.pi/2, 2), -round(math.pi/2, 2), round(math.pi,2)], [prob_dist])[0]
                self.reorient(goal_orientation)
                step_count = 0 
                print('resetting direction --')
                    
            self.rate.sleep()
    		
    		
    def initiateSpiralMove(self):
    
        step_count = 0 
 
        current_orientation = self.theta
        prob_dist = self.generatePath(self.current_orientation)
        print('generated spiral path', prob_dist)
        init_index = 0 
        
        # set hard time limit 
        while (time.time() - self.init_time <= self.gen_time and init_index <= len(prob_dist)): 
        
            # self.init_time = time.time()
            self.avoidanceBehavior() # continously check for obstacles 
                
            step_count += 1 
                
            if step_count == self.step_size: 
                # re-orient again to another cardinal direction 
                init_index+=1
                goal_orientation = prob_dist[init_index]
                print('reorienting to next dir --', prob_dist[init_index])
                self.reorient(goal_orientation)
                step_count = 0 
                    
            self.rate.sleep()
    	
    	
    def initiateBallistic(self): 
        step_count = 0 
 
        current_orientation = self.theta
        prob_dist = self.getProbDistrib(current_orientation)
        
        goal_orientation = self.theta
        # set hard time limit 
        while (time.time() - self.init_time <= self.gen_time): 
            # self.init_time = time.time()
            self.avoidanceBehavior() # continously check for obstacles 
                
            step_count += 1 
                
            if step_count == self.step_size: 
                # re-orient again to another cardinal direction 
                goal_orientation = self.theta
                self.reorient(goal_orientation)
                step_count = 0 
                    
            self.rate.sleep()
    	
    	
    def calculate_angle(self):

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
        
        home_orientation = self.calculate_angle()

        while (euclidean_distance_curr > 0.1):

            while (self.isAvoiding):
                print('initiating obstacle behavior')
                self.avoidanceBehavior() # continously check for obstacles

                self.rate.sleep()

            while (round(self.theta,2) != home_orientation):

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
        # demo_robot.initiateCRW()
        # demo_robot.initiateSpiralMove()
        # demo_robot.homing_behavior()
        # demo_robot.initiateBallistic()
        # demo_robot.moveFromPointAtoPointB()

        rospy.spin()
