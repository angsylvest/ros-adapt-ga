#! /usr/bin/python3
from math import sqrt, atan2
import math
import rospy
import time 

from geometry_msgs.msg import PoseStamped, Twist, Pose, Point
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from msg import chromosome
from adapt_ga.ga import *
from adapt_ga.strategy import *

num_turtlebots = 3 

def update_pose(self, data):

    global goal_pose 
    self.pose = data.pose.pose.position

    theta_int = data.pose.pose.orientation
    (r, p, y) = euler_from_quaternion([theta_int.x, theta_int.y, theta_int.z, theta_int.w])
    self.roll = r
    self.pitch = p
    self.theta = y


def process_info(self, data): 
    d = data
    
# create publishers for chromosome for each robot (hard-coded) + create listeners for each robot (hard-coded)
chrom_pubs = {}
pop_subs = {}
for i in range(num_turtlebots):
    chr_topic = f'/turtlebot{i}'
    pop_publisher = rospy.Publisher(f'{chr_topic}/chrome-fit', chromosome, queue_size=50)
    pop_subscriber = rospy.Subscriber(f'{chr_topic}/chrome-info', chromosome, process_info)

    chrom_pubs[chr_topic] = pop_publisher
    pop_subs[chr_topic] = pop_subscriber


# Communication example between robots 

# 1. each agent gets randomly assigned a chromosome 
ga_rob = GA()
ga_rob.create_individual_genotype()

ga_rob_2 = GA()
ga_rob_2.create_individual_genotype()

print(f'example genotype generated: {ga_rob.curr_genotype}')
processed_chromosome = ga_rob.process_chromosome(ga_rob.curr_genotype)
print(f'example of processed chromosome: {processed_chromosome}') # returns forward_speed, energy_cost, energy_per_item, observations_threshold

rospy.init_node('simulation', anonymous=False)
msg = chromosome()
msg.speed, msg.reward, msg.penalty, msg.threshold, msg.fitness = str(processed_chromosome[0]), str(processed_chromosome[1]), str(processed_chromosome[2]), str(processed_chromosome[3]), 0

chrom_pubs[0].publish(msg)

print(f'published custom topic')
# st = Strategy(reward=10, penalty=2, obs_thres=5, step_size=5)
# st.update_from_chrom(processed_chromosome)

# ga_rob.update_best_encountered(r2=ga_rob_2, fitness = 0)
# ga_rob.reproduce()
