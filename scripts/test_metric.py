#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, Twist, Quaternion
from std_msgs.msg import Bool
import json
import tf
from math import *

class RobotLogger():
    def __init__(self):
        rospy.init_node("robot_logger")
        self.base_pose_ground_truth_sub = rospy.Subscriber(
                "base_pose_ground_truth",
                Odometry,
                self.update_ground_truth
        )
        self.particlecloud_sub = rospy.Subscriber(
                "particlecloud",
                PoseArray,
                self.update_particlecloud
        )
        self.cmd_vel_sub = rospy.Subscriber(
                "cmd_vel",
                Twist,
                self.turn_taken
        )
        self.simulation_complete_sub = rospy.Subscriber(
                "sim_complete",
                Bool,
                self.handle_shutdown
        )
        self.init_files()
        rospy.spin()

    def init_files(self):
        with open('time_results.json', 'w+') as infile:
            pass
        open('std_dev_results.json', 'w+').close()
        open('metric_results.json', 'w+').close()
        self.time_data = []
        self.std_dev_x_data = []
        self.std_dev_y_data = []
        self.std_dev_angle_data = []
        self.std_dev_data = []
        self.metric_data = []
        self.init_time = float(rospy.Time.now().to_sec()/60)
        #Initialize with json format
        # self.time_data.extend(['{', '\\n', '\t', '"time_elapsed": [', '\\n'])
        # self.std_dev_x_data.extend(['{', '\\n', '\t', '"std_dev_x": [', '\\n'])
        # self.std_dev_y_data.extend(['{', '\\n', '\t', '"std_dev_y": [', '\\n'])
        # self.std_dev_angle_data.extend(['{', '\\n', '\t', '"std_dev_angle": [', '\\n'])
        # self.metric_data.extend(['{', '\\n', '\t', '"metric": [', '\\n'])

    def update_ground_truth(self, message):
        self.base_truth_x = message.pose.pose.position.x
        self.base_truth_y = message.pose.pose.position.y
        quaternion = (message.pose.pose.orientation.x,
    				message.pose.pose.orientation.y,
    				message.pose.pose.orientation.z,
    				message.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.base_truth_angle = euler[2]

    def update_particlecloud(self, message):
    	self.particlecloud_poses = message.poses 

    def turn_taken(self, message):
        #angle = message.angular.z
        # if angle > 0.0:
	    self.update_metric()

    def update_metric(self):
    	time_elapsed_value = float(rospy.Time.now().to_sec()/60) - self.init_time #minutes
    	self.time_data.extend([time_elapsed_value])

    	var_x_value = 0.0
    	var_y_value = 0.0
    	var_angle_value = 0.0

    	#print self.base_truth_x, self.base_truth_y, self.base_truth_angle

    	#STD DEV
    	for particle_index in range(len(self.particlecloud_poses)):
    		pose = self.particlecloud_poses[particle_index]
    		var_x_value += (pose.position.x - self.base_truth_x)**2
    		var_y_value += (pose.position.y - self.base_truth_y)**2
    		quaternion = (pose.orientation.x,
						pose.orientation.y,
						pose.orientation.z,
						pose.orientation.w)
	        particle_euler = tf.transformations.euler_from_quaternion(quaternion)
    		var_angle_value += (particle_euler[2] - self.base_truth_angle)**2

    	norm_var_x = var_x_value/len(self.particlecloud_poses)
    	norm_var_y = var_y_value/len(self.particlecloud_poses)
    	norm_var_angle = var_angle_value/len(self.particlecloud_poses)

    	std_dev_x_value = sqrt(norm_var_x)
    	std_dev_y_value = sqrt(norm_var_y)
    	std_dev_angle_value = sqrt(norm_var_angle)

    	print std_dev_x_value, std_dev_y_value, std_dev_angle_value


    	self.std_dev_x_data.extend([std_dev_x_value])
    	self.std_dev_y_data.extend([std_dev_y_value])
    	self.std_dev_angle_data.extend([std_dev_angle_value])

    	metric = float(1.0/(time_elapsed_value*(std_dev_x_value+std_dev_y_value+std_dev_angle_value)))
    	self.metric_data.extend([metric])

    def handle_shutdown(self, message):
        print "sim complete!", message.data

        self.update_metric()

        if message.data:
            with open('time_results.json', 'w') as time:
                time.write('{\n"time_elapsed" : ')
                json.dump(self.time_data, time)
                time.write('\n} \n')
            with open('std_dev_results.json', 'w') as std_dev:
                std_dev.write('{\n"std_dev_x" : ')
                json.dump(self.std_dev_x_data, std_dev)
                std_dev.write('\n} \n')
                std_dev.write('{\n"std_dev_y" : ')
                json.dump(self.std_dev_y_data, std_dev)
                std_dev.write('\n} \n')
                std_dev.write('{\n"std_dev_angle" : ')
                json.dump(self.std_dev_angle_data, std_dev)
                std_dev.write('\n} \n')
            with open('metric_results.json', 'w') as metric_file:
                metric_file.write('{\n"metric" : ')
                json.dump(self.metric_data, metric_file)
                metric_file.write('\n} \n')


if __name__ == '__main__':
    rl = RobotLogger()
