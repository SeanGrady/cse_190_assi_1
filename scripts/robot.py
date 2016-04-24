#!/usr/bin/env python

import rospy
import numpy as np
import random as r
from math import *
from copy import deepcopy
from cse_190_assi_2_2.msg import temperatureMessage, RobotProbabilities
from cse_190_assi_2_2.srv import requestTexture, moveService
from std_msgs.msg import Bool, String, Float32
from collections import deque
from read_config import read_config

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PoseArray, PointStamped, Quaternion, Point, Twist
from sensor_msgs.msg import LaserScan
import tf

import map_utils
from helper_functions import *	

class Particle:
	def __init__(self, *args):
		if len(args) == 2 and isinstance(args[0], OccupancyGrid):
		    self._init_particle(args[0], args[1])
		else:
		    pass

	def _init_particle(self,map_message, weight):
		self.x = r.random() * float(map_message.info.width)
		self.y = r.random() * float(map_message.info.height)
		self.theta = r.random() * 2 * pi
		self.pose = get_pose(self.x, self.y, self.theta)
		self.weight = weight	

	def sense(self, scan_msg, likelihood_field):
		xs, ys = self.laser_range_end(scan_msg)
		total_prob = 0
		for i in range(0, len(xs), 1):
		    likelihood = likelihood_field.get_cell(xs[i], ys[i])
		    if np.isnan(likelihood):
		        likelihood = 0
		    	pz=(laser_z_hit * likelihood + laser_z_rand)
			total_prob += pz*pz*pz
		
			if len(xs)>0:
				self.weight *= total_prob
		return self.weight

	def laser_range_end(self, scan_msg):
		theta_beam = np.arange(scan_msg.angle_min, scan_msg.angle_max,
		                       scan_msg.angle_increment)
		ranges = np.array(scan_msg.ranges)
		
		xs = []
		ys = []
		
		for i in range(len(ranges)):
			if ranges[i] == scan_msg.range_max:
				continue
				#print "Obstacle Seen"
			xs.append(self.x + ranges[i] * np.cos(self.theta + theta_beam[i]))
			ys.append(self.y + ranges[i] * np.sin(self.theta + theta_beam[i]))
		
		return xs, ys


class ParticleFilterLocalization():

	def __init__(self):
		self.config = read_config()
		r.seed(self.config['seed'])
		rospy.init_node("particle_filter_localization")
		print "Config Read Done"
		
		global laser_z_hit 
		global laser_z_rand 
		global laser_sigma_hit 
		
		laser_z_hit = 0.80
		laser_z_rand = 0.05
		laser_sigma_hit = 2
		
		self.likelihood_field = None
		self.map = None
		self.num_particles = 800
		rospy.Subscriber('map', OccupancyGrid, self.map_callback)
		rospy.Subscriber('base_scan', LaserScan, self.scan_callback)
		rospy.Subscriber('pc_pub_start', PoseArray, self.particle_pub_start_callback)
		
		self.particle_pub = rospy.Publisher('particlecloud', PoseArray, queue_size = 10)
		self.particle_pub_start = rospy.Publisher('pc_pub_start', PoseArray, queue_size = 10, latch=True)
		self.likelihood_pub = rospy.Publisher('likelihood_field', OccupancyGrid, queue_size = 10, latch=True)
		self.result_update_pub = rospy.Publisher("result_update", Bool, queue_size = 10)
		self.shutdown_pub = rospy.Publisher("sim_complete", Bool, queue_size = 10)

		# We need the map before we can initialize the particles.
		while not rospy.is_shutdown() and self.map is None:
			rospy.loginfo("Waiting for map.")
			rospy.sleep(.5)
		
		self.init_Particles()
		self.particle_pub_start.publish(self.pose_array)
		rospy.sleep(1)	
		
		if self.likelihood_field is None:
			self.update_likelihood_field(self.map, laser_sigma_hit)
		
		self.likelihood_pub.publish(self.likelihood_field.to_message())
		rospy.sleep(1)

		""" Perform Move, then Update Probabilities """
		self.motions = deque(self.config['move_list'])
		self.move_list_size = len(self.motions)

		self.first_move = 1		
		while len(self.motions) > 0:
			self.result_update_pub.publish(True)
			motion = self.motions.popleft();

			if self.first_move == 1:
				if motion[0] != 0.0:
					self.move(0.0, motion[0], 0.0, 1)
				self.move(motion[1], 0.0, 5.0, 1)	
			else:
				if motion[0] != 0.0:
					self.move(0.0, motion[0], 0.0, 0)
				self.move(motion[1], 0.0, 10.0, 0)	
			self.first_move = 0

	
		#rospy.spin()	
	
		#""" Waiting Until Shutdown """
		#while not rospy.is_shutdown():
		#	rospy.sleep(1)	

		#Publish Shutdown
		rospy.sleep(2)
		self.shutdown_pub.publish(True)
		rospy.sleep(2)
		rospy.signal_shutdown("Shutting Down after all moves")


	def particle_pub_start_callback(self, pose_array):
		print "Entered Particle Pub Start"
		while not rospy.is_shutdown():
			self.particle_pub.publish(self.pose_array)
			rospy.sleep(0.1)

	def move(self, dist, angle, dist_per_move, add_noise_every_step):

		if angle != 0:
			move_function(angle, dist)
			for particle_index in range(self.num_particles):
				noise = 0
				if add_noise_every_step == 1 :
					noise =  np.random.randn()*0.05*2*pi
				self.particles[particle_index].theta += (angle*pi/180) + noise
				self.particles[particle_index].pose = get_pose(self.particles[particle_index].x, self.particles[particle_index].y, self.particles[particle_index].theta)
	
		else:
			#Moving one pixel at a time to ensure particles don't cross obstacles (UNKNOWINGLY)
			#dist_per_move = 1.0
			while dist != 0:
				dist = dist - dist_per_move
				if (dist < 0):
					dist_per_move = dist_per_move + dist
					dist = 0
				move_function(angle, dist_per_move)
	
				""" Update particles odom """
				for particle_index in range(self.num_particles):
					self.particles[particle_index].x += dist_per_move*np.cos(self.particles[particle_index].theta)
					self.particles[particle_index].y += dist_per_move*np.sin(self.particles[particle_index].theta)

					# if add_noise_every_step == 1 :
					self.particles[particle_index].x += dist_per_move*np.cos(self.particles[particle_index].theta)*np.random.randn()*0.1 #20% 
					self.particles[particle_index].y += dist_per_move*np.sin(self.particles[particle_index].theta)*np.random.randn()*0.1
					noise = ceil(r.gauss(0, pi*dist_per_move/1800)*100.)/100. #0.2 degree std dev per metre
					self.particles[particle_index].theta += noise 

					self.particles[particle_index].pose = get_pose(self.particles[particle_index].x, self.particles[particle_index].y, self.particles[particle_index].theta)
					map_acc = map_utils.Map(self.map)
					if np.isnan(map_acc.get_cell(self.particles[particle_index].x,self.particles[particle_index].y)): #outside map
						self.particles[particle_index].weight *= 0.0 
					elif map_acc.get_cell(self.particles[particle_index].x,self.particles[particle_index].y) == 1: #On Obstacles
						self.particles[particle_index].weight *=  0.0
						
				#Normalise Weights
				total_weight = 0.0
				for particle_index in range(self.num_particles):
					total_weight += self.particles[particle_index].weight
				for particle_index in range(self.num_particles):
					self.particles[particle_index].weight /= total_weight
	
				# Publish the updated particles.
				self.pose_array = self.create_pose_array_msg()
				self.particles_resample(add_noise_every_step)
			
		self.pose_array = self.create_pose_array_msg()
		self.particles_resample(0)

	def init_Particles(self):
		self.particles = []
		for particle_count in range(self.num_particles):
			new_particle = Particle(self.map, float(1.0/self.num_particles))
			self.particles.append(new_particle)
		# Publish the newly created particles.
		self.pose_array = self.create_pose_array_msg()
		
	def create_pose_array_msg(self):
		"""Create a PoseArray object including the poses of all particles.
		Returns: geometry_msgs/PoseArray
		"""
		pose_array = PoseArray()
		pose_array.header.stamp = rospy.Time.now()
		pose_array.header.frame_id = 'map'
		pose_array.poses = []
		for p in self.particles:
		    pose_array.poses.append(p.pose)
		return pose_array

	def map_callback(self, map_msg):
		""" Store the map message in an instance variable. """
		self.map = map_msg

	def scan_callback(self, laser_scan):
		""" Laser Scan needs to keep updating a class variable so that they can be used as soon as MOVE made """
		self.laser_scan_info = laser_scan

	def update_likelihood_field(self, map_msg, laser_sigma):

		rospy.loginfo('building Likelihood map...')
		world_map = map_utils.Map(map_msg)
		
		rospy.loginfo('building KDTree')
		from sklearn.neighbors import KDTree
		occupied_points = []
		all_positions = []
		for i in range(world_map.grid.shape[0]):
			for j in range(world_map.grid.shape[1]):
				all_positions.append(world_map.cell_position(i, j))
				if world_map.grid[i, j] > .9:
					occupied_points.append(world_map.cell_position(i, j))
		
		kdt = KDTree(occupied_points)
		
		rospy.loginfo('Constructing likelihood field from KDTree.')
		self.likelihood_field = map_utils.Map(map_msg)
		#self.likelihood_field = map_utils.Map(world_map.to_message())
		dists = kdt.query(all_positions, k=1)[0][:]
		probs = np.exp(-(dists**2) / (2 * laser_sigma**2))
		#print '\n'.join(str(p) for p in probs) 
		
		self.likelihood_field.grid = probs.reshape(self.likelihood_field.grid.shape)
		rospy.loginfo('Done building likelihood field')

	def particles_resample(self, add_noise_every_step):
		w = []
		total_w = 0.0
		for particle_index in range(self.num_particles):
			w.append(self.particles[particle_index].sense(self.laser_scan_info, self.likelihood_field))
			total_w += w[particle_index]
		
		prob_pos = []
		alpha = []
		for i in range(self.num_particles):
			alpha.append(w[i]/total_w)
			
		running_total = 0.0
		for weight in alpha:
		    running_total += weight
		    prob_pos.append(running_total)
			
		new_particle_set = []
		beta = r.random();
		old_beta = beta;
		new_beta_start = 0;
		while (len(new_particle_set)!=self.num_particles):
			s = r.uniform(beta, beta+float(1.0/self.num_particles))
			for j in range(self.num_particles):
				if (j == 0 and s < prob_pos[j]) or (s > prob_pos[j-1] and s < prob_pos[j]):
					if add_noise_every_step == 1 :
						noise = ceil(r.gauss(0, 5 )*100.)/100. #0.1m std dev
						self.particles[j].x += noise 
						noise = ceil(r.gauss(0, 5)*100.)/100. #0.1m std dev
						self.particles[j].y += noise
						noise = ceil(r.gauss(0, pi/90)*100.)/100. #0.2 degree std dev
						self.particles[j].theta += noise 
						self.particles[j].pose = get_pose(self.particles[j].x, self.particles[j].y, self.particles[j].theta)					
					new_particle_set.append(deepcopy(self.particles[j]))
					break
			
			beta += float(1.0/self.num_particles)
			#Cyclic
			if (beta > 1):
				beta = beta-1
			if (beta < old_beta):
		    		new_beta_start = 1
			if (beta > old_beta and new_beta_start == 1):
		    		beta = r.random();
				old_beta = beta;
				new_beta_start = 0
				
		self.particles = deepcopy(new_particle_set)			
		# Publish the newly sampled particles.
		self.pose_array = self.create_pose_array_msg()


if __name__ == '__main__':
	pfl = ParticleFilterLocalization()
