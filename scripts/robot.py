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
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray, PointStamped, Quaternion, Point, Twist
from sensor_msgs.msg import LaserScan
import tf

import map_utils

#Add in separate file map_status.py
#class MapStatus(object):
#    def __init__(self, ):
	

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
	self.pose_update(self.x, self.y, self.theta)
	self.weight = weight

    def pose_update(self, x, y, theta):
	self.pose = Pose()
        self.pose.position.x = x
        self.pose.position.y = y
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        self.pose.orientation.x = quat[0]
        self.pose.orientation.y = quat[1]
        self.pose.orientation.z = quat[2]
        self.pose.orientation.w = quat[3]
	

    def sense(self, scan_msg, likelihood_field):
        xs, ys = self.laser_range_end(scan_msg)
	#self.scan_markers(scan_msg)	
        total_prob = 0
        #total_prob = 1
	#TODO Step used : 10 (amcl uses 30)
        #for i in range(0, len(xs), 10):
        for i in range(0, len(xs), 1):
            likelihood = likelihood_field.get_cell(xs[i], ys[i])
            if np.isnan(likelihood):
                likelihood = 0
	    pz=(laser_z_hit * likelihood + laser_z_rand)
            #total_prob += np.log(pz)
	    #pz=(laser_z_hit * likelihood + laser_z_rand*float(1.0/scan_msg.range_max))
	    #pz=laser_z_hit * likelihood
            total_prob += pz*pz*pz
	    #total_prob *= pz

        #self.weight *= np.exp(total_prob)
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
			#print ranges[i], scan_msg.range_max
			continue
		#print "Obstacle Seen"
	        xs.append(self.x + ranges[i] * np.cos(self.theta + theta_beam[i]))
        	ys.append(self.y + ranges[i] * np.sin(self.theta + theta_beam[i]))

#	if len(xs) > 0 and len(ys) > 0 :
#        	# Clear out nan entries:
#        	xs = xs[np.logical_not(np.isnan(xs))]
#        	ys = ys[np.logical_not(np.isnan(ys))]
#
#        	# Clear out inf entries:
#        	xs = xs[np.logical_not(np.isinf(xs))]
#        	ys = ys[np.logical_not(np.isinf(ys))]
        return xs, ys

    def scan_markers(self, scan_msg, color=(1, 0, 0)):
        """Returns a MarkerArray message displaying what the scan message
        would look like from the perspective of this particle.  Just
        for debugging.

        Returns:
           visualization_msgs/MarkerArray

        """

        xs, ys = self.laser_range_end(scan_msg)
        marker_array = MarkerArray()
        header = Marker().header
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        for i in range(len(xs)):
            marker = Marker()
            marker.header = header
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = xs[i]
            marker.pose.position.y = ys[i]
            marker.pose.position.z = .3
            marker.pose.orientation.w = 1.0
            marker.id = np.array([(id(self) * 13 *  + i*17) % 2**32],
                                 dtype='int32')[0]
            marker.scale.x = .02
            marker.scale.y = .02
            marker.scale.z = .02

            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            marker.lifetime = rospy.Duration(5.0)
            marker_array.markers.append(marker)

	MA_publisher.publish(marker_array)
	#rospy.sleep(0.01) 

class ParticleFilterLocalization():


    def __init__(self):
        self.config = read_config()
        rospy.init_node("particle_filter_localization")
	

	#Step1: Test Particle Addition
	#Ref: http://wiki.ros.org/amcl
	global laser_z_hit 
	global laser_z_rand 
	global laser_sigma_hit 

	laser_z_hit = 0.90
	laser_z_rand = 0.05
	laser_sigma_hit = 2

        self.likelihood_field = None

	#tf::TransformBroadcaster br;
	#tf::Transform transform;
	#transform.setOrigin( tf::Vector3(x,y,z)); // position in respect to the world frame
	#transform.setRotation( tf::Quaternion(x,y,z,w) ); // orientation in respect to the world frame
	#br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot_n"));



        self.map = None
        self.num_particles = 1000
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('base_scan', LaserScan, self.scan_callback)
        rospy.Subscriber('pc_pub_start', PoseArray, self.particle_pub_start_callback)
        #rospy.Subscriber('initialpose', PoseWithCovarianceStamped,self.initial_pose_callback)

	global MA_publisher
        MA_publisher = rospy.Publisher('MarkerArray', MarkerArray, queue_size = 10)
        self.particle_pub = rospy.Publisher('particlecloud', PoseArray, queue_size = 10)
        self.particle_pub_start = rospy.Publisher('pc_pub_start', PoseArray, queue_size = 10, latch=True)
        self.likelihood_pub = rospy.Publisher('likelihood_field', OccupancyGrid, queue_size = 10, latch=True)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

        # We need the map before we can initialize the particles.
        while not rospy.is_shutdown() and self.map is None:
            rospy.loginfo("Waiting for map.")
            rospy.sleep(.5)


	self.init_Particles()

	print "Calling Particle Pub Start"
        self.particle_pub_start.publish(self.pose_array)
	rospy.sleep(1)	

        if self.likelihood_field is None:
            self.update_likelihood_field(self.map, laser_sigma_hit)

        self.likelihood_pub.publish(self.likelihood_field.to_message())
	rospy.sleep(1)

	""" Perform Move, then Update Probabilities """
	self.move(10, 0.0, 1.0, 1)
	self.move(10, 0.0, 2.0, 1)
	self.move(100, 0.0, 2.0, 1)
	
	#Earlier more variance used to secure the significant particles, once somewhat localised - now to increase accuracy, the variance has to be reduced
	laser_sigma_hit = 1
        self.update_likelihood_field(self.map, laser_sigma_hit)
	self.move(10, 90.0, 1.0, 1)
	self.move(100, 90.0, 3.0, 1)
	self.move(15.0, -90.0, 2.0, 1)
	self.move(60.0, 0.0, 10.0, 0)
	self.move(60.0, 180.0, 10.0, 0)
	self.move(15.0, 0.0, 2.0, 0)
	self.move(20.0, 0.0, 2.0, 0)
	self.move(60.0, 0.0, 20.0, 0)
	self.move(80.0, 180.0, 20.0, 0)
	self.move(80, 90.0, 20.0, 0)
	self.move(30, -90.0, 20.0, 0)
	self.move(45, 90.0, 10.0, 0)
	self.move(50, -90.0, 10.0, 0)
	#self.laser_scan_sense(self.laser_scan_info)
	

	rospy.spin()	

	""" Waiting Until Shutdown """
	while not rospy.is_shutdown():
		rospy.sleep(1)	

    def particle_pub_start_callback(self, pose_array):
	print "Entered Particle Pub Start"
	while not rospy.is_shutdown():
        	self.particle_pub.publish(self.pose_array)
		rospy.sleep(0.1)


    def move(self, dist, angle, dist_per_move, add_noise_every_step):
	
	if abs(angle) > 20:
		for _ in range(18):	
		    	twist = Twist()
    			twist.angular.z = float(angle*pi/180.0)/18.0; #I will use 18 cycles - stage sometimes behaving wierdly during faster turns
			print twist.angular.z;
    			self.cmd_vel_pub.publish(twist)
    			rospy.sleep(1)
    			#rospy.sleep(10)
			#rospy.loginfo("Stopping!")
    			twist = Twist()
    			self.cmd_vel_pub.publish(twist)
	else: 
		twist = Twist()
    		twist.angular.z = float(angle*pi/180.0); 
		print twist.angular.z;
    		self.cmd_vel_pub.publish(twist)
    		rospy.sleep(1)
    		#rospy.sleep(10)
		#rospy.loginfo("Stopping!")
    		twist = Twist()
    		self.cmd_vel_pub.publish(twist)
		

        for particle_index in range(self.num_particles):
		noise = 0
		if add_noise_every_step == 1 :
			noise =  np.random.randn()*0.05*2*pi
		#self.particles[particle_index].theta += (angle*pi/180)*(1 + np.random.randn()*0.05)
		self.particles[particle_index].theta += (angle*pi/180) + noise
        	#noise = ceil(r.gauss(0, pi/36.0)*100.)/100. #5 degrees std dev
		#self.particles[particle_index].theta += (angle*pi/180.0) + noise 
		self.particles[particle_index].pose_update(self.particles[particle_index].x, self.particles[particle_index].y, self.particles[particle_index].theta)

	#Moving one pixel at a time to ensure particles don't cross obstacles (UNKNOWINGLY)
	#dist_per_move = 1.0
	while dist != 0:
		dist = dist - dist_per_move
		if (dist < 0):
			dist_per_move = dist_per_move + dist
			dist = 0
    		twist = Twist()
    		#twist.linear.x = dist;    
    		twist.linear.x = dist_per_move;     
    		self.cmd_vel_pub.publish(twist)
    		rospy.sleep(1)
		#rospy.loginfo("Stopping!")
    		twist = Twist()
    		self.cmd_vel_pub.publish(twist)

		""" Update particles odom """
        	for particle_index in range(self.num_particles):
			#x_sign = 1.0
			#y_sign = 1.0
			#if (angle > 90 and angle < 270): 
			#	x_sign = -1.0
			#if (angle > 180 and angle < 360):
			#	y_sign = -1.0

			self.particles[particle_index].x += dist_per_move*np.cos(self.particles[particle_index].theta)
			self.particles[particle_index].y += dist_per_move*np.sin(self.particles[particle_index].theta)
			if add_noise_every_step == 1 :
				self.particles[particle_index].x += dist_per_move*np.cos(self.particles[particle_index].theta)*np.random.randn()*0.2 #20% 
				self.particles[particle_index].y += dist_per_move*np.sin(self.particles[particle_index].theta)*np.random.randn()*0.2
	        		noise = ceil(r.gauss(0, pi*dist_per_move/900)*100.)/100. #0.2 degree std dev per metre
				self.particles[particle_index].theta += noise 
			#self.particles[particle_index].theta += (angle*pi/180)*(1 + np.random.randn()*0.05)
			#self.particles[particle_index].theta += (angle*pi/180) + np.random.randn()*0.01*2*pi
			self.particles[particle_index].pose_update(self.particles[particle_index].x, self.particles[particle_index].y, self.particles[particle_index].theta)
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
			self.particles[particle_index].weight /= total_weight; 

        	# Publish the updated particles.
        	self.pose_array = self.create_pose_array_msg()
		self.particles_resample(1) # without sense	
	self.particles_resample(0) # with sense	

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
	#rospy.loginfo("PoseArray Update")
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
	#print pose_array.header.stamp
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
		    #print world_map.cell_position(i, j)
		    #print world_map.grid[i, j]
                    occupied_points.append(world_map.cell_position(i, j))

        kdt = KDTree(occupied_points)

        rospy.loginfo('Constructing likelihood field from KDTree.')
        self.likelihood_field = map_utils.Map(world_map.to_message())
        dists = kdt.query(all_positions, k=1)[0][:]
        probs = np.exp(-(dists**2) / (2 * laser_sigma**2))
	#print '\n'.join(str(p) for p in probs) 

        self.likelihood_field.grid = probs.reshape(self.likelihood_field.grid.shape)

        rospy.loginfo('Done building likelihood field')

    def particles_resample(self, dont_sense):
	w = []
	total_w = 0.0
        for particle_index in range(self.num_particles):
		w.append(self.particles[particle_index].sense(self.laser_scan_info, self.likelihood_field))
		#if (dont_sense == 0):
		#	w.append(self.particles[particle_index].sense(self.laser_scan_info, self.likelihood_field))
		#else:
		#	w.append(self.particles[particle_index].weight)
		total_w += w[particle_index]
	
	#print '\n'.join(str(p) for p in w) 

	prob_pos = []
	alpha = []
	#alpha = [wi/sum(w) for wi in w]
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
	#for i in range(self.num_particles): 
	    s = r.uniform(beta, beta+float(1.0/self.num_particles))
	    for j in range(self.num_particles):
	        if j == 0 and s < prob_pos[j]:
		    #if self.particles[j].weight != 0:
	            	new_particle_set.append(deepcopy(self.particles[j]))
	           	break
	        elif s > prob_pos[j-1] and s < prob_pos[j]:
		    #if self.particles[j].weight != 0:
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

        #self.temperature_subscriber = rospy.Subscriber(
        #        "/temp_sensor/data",
        #        temperatureMessage,
        #        self.handle_incoming_temperature_data
        #)
        #self.move_requester = rospy.ServiceProxy(
        #        "moveService",
        #        moveService,
        #)
        #self.temp_activator = rospy.Publisher(
        #        "/temp_sensor/activation",
        #        Bool,
        #        queue_size = 10
        #)
        #self.temp_res_pub = rospy.Publisher(
        #        "/results/temperature_data",
        #        Float32,
        #        queue_size = 10
        #)
        #self.prob_res_pub = rospy.Publisher(
        #        "/results/probabilities",
        #        RobotProbabilities,
        #        queue_size = 10
        #)
        #self.shutdown_pub = rospy.Publisher(
        #        "/map_node/sim_complete",
        #        Bool,
        #        queue_size = 10
        #)
#        self.initialize_maps()
#        self.initialize_beliefs()
#        self.motions = deque(self.config['move_list'])
#        rospy.sleep(2)
#        self.simulate()
#
#    def initialize_maps(self):
#        self.heat_map = self.generate_heatmap(self.config['pipe_map'])
#
#    def initialize_beliefs(self):
#        """
#        set the initial probability matrix - everywhere is equally likely.
#        """
#        init_prob = 1.0 / float(len(self.config['texture_map'])) / float(len(self.config['texture_map'][0]))
#        starting_beliefs = [[init_prob for row in range(len(self.config['texture_map'][0]))]
#                            for col in range(len(self.config['texture_map']))]
#        self.probability_matrix = starting_beliefs
#
#    def generate_heatmap(self, pipe_map):
#        """
#        Generates a heat map based on whether there is a cold or hot pipe
#        running under each tile. Tiles with no pipes underneath will have
#        a "warm" temperature
#        """
#        temp_cold = 20.0
#        temp_hot  = 40.0
#        temp_warm = 25.0
#        heat_map = [[ 0.0 for x in row] for row in pipe_map]
#        for i in range(len(pipe_map)):
#            for j in range(len(pipe_map[0])):
#                if pipe_map[i][j] == 'C':
#                    heat_map[i][j] = temp_cold
#
#                elif pipe_map[i][j] == 'H':
#                    heat_map[i][j] = temp_hot
#                else:
#                    heat_map[i][j] = temp_warm
#        return heat_map
#
#    def simulate(self):
#        """
#        Kicks the whole thing off. Calls rospy.spin() after activating
#        the temperature sensor because from there on out everything happens
#        on callback functions.
#        """
#        self.begin_simulation()
#        rospy.spin()
#
#    def begin_simulation(self):
#        """
#        Publishes to the topic that activates the temperature sensor.
#        """
#        activation_message = Bool()
#        activation_message.data = True
#        self.temp_activator.publish(activation_message)
#
#    def end_simulation(self):
#        """
#        Publishes to the topic that stops the temperature sensor at the
#        end of the simulation.
#        """
#        activation_message = Bool()
#        activation_message.data = False
#        self.temp_activator.publish(activation_message)
#        self.shutdown_pub.publish(True)
#        rospy.sleep(1)
#        rospy.signal_shutdown("because I said so")
#
#    def handle_incoming_temperature_data(self, message):
#        """
#        Callback function for the temp data. Initiates the timestep.
#        """
#        temp_reading = message.temperature
#        self.timestep(temp_reading)
#
#    def timestep(self, temp_reading):
#        """
#        Main sense->evalute->move function
#        """
#        self.update_from_temp(temp_reading)
#        texture_reading = self.request_texture_reading()
#        self.update_from_tex(texture_reading)
#        self.publish_sensor_values(texture_reading, temp_reading)
#        if len(self.motions) > 0:
#            self.move_and_update(self.motions.popleft())
#            self.publish_beliefs()
#        else:
#            print "Motion complete"
#            self.show(self.probability_matrix)
#            self.end_simulation()
#
#    def publish_sensor_values(self, tex, temp):
#        self.tex_res_pub.publish(tex)
#        self.temp_res_pub.publish(temp)
#
#    def publish_beliefs(self):
#        prob_list = [p for row in self.probability_matrix for p in row]
#        self.prob_res_pub.publish(prob_list)
#
#    def show(self, p):
#        """
#        prints a probability matrix to the screen in a readable format
#        """
#        rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x),r)) + ']' for r in p]
#        print '[' + ',\n '.join(rows) + ']'
#
#    def update_from_temp(self, temp_reading):
#        """
#        Update the beliefs about the robot's position based on a temperature
#        measurement.
#        """
#        temp_probs = deepcopy(self.probability_matrix)
#        num_cols = len(temp_probs[0])
#        num_rows = len(temp_probs)
#        for i in range(num_rows):
#            for j in range(num_cols):
#                if self.config['temp_noise_std_dev'] == 0:
#                    prob_hit = heat_map[i][j] == temp_reading
#                else:
#                    prob_hit = 1/(m.sqrt(2*m.pi)*self.config['temp_noise_std_dev'])*m.e**(-0.5*(float(self.heat_map[i][j] - temp_reading)/self.config['temp_noise_std_dev'])**2)
#                temp_probs[i][j] = prob_hit * self.probability_matrix[i][j]
#	print "Temperature Probabilities"
#	self.show(temp_probs)
#
#        total_prob = sum(sum(p) for p in temp_probs)
#        if total_prob == 0:
#            print "sum of probabilities is zero"
#        else:
#            temp_probs = [[prob/total_prob for prob in row] for row in temp_probs]
#        self.probability_matrix = temp_probs
#	print "Temperature Probabilities NORMALIZED"
#	self.show(self.probability_matrix)
#	#print self.probability_matrix
#
#    def update_from_tex(self, texture_reading):
#        """
#        Update the beliefs about the robot's position based on a texture
#        measurement.
#        """
#        temp_probs = deepcopy(self.probability_matrix)
#        num_cols = len(temp_probs[0])
#        num_rows = len(temp_probs)
#        for i in range(num_rows):
#            for j in range(num_cols):
#                hit = int(self.config['texture_map'][i][j] == texture_reading)
#                temp_probs[i][j] = (self.config['prob_tex_correct'] * hit + (1 - self.config['prob_tex_correct']) * (1 - hit)) * self.probability_matrix[i][j]
#	print "Texture Probabilities"
#	self.show(temp_probs)
#
#        total_prob = sum(sum(p) for p in temp_probs)
#        if total_prob == 0:
#            print "sum of probabilities is zero"
#        else:
#            temp_probs = [[prob/total_prob for prob in row] for row in temp_probs]
#        self.probability_matrix = temp_probs
#	print "Texture Probabilities NORMALIZED"
#	self.show(self.probability_matrix)
#
#    def request_texture_reading(self):
#        """
#        Get a texture reading from the temperature sensor node via the
#        texture request service.
#        """
#        response = self.texture_requester()
#        texture_reading = response.data
#        return texture_reading
#
#    def move_and_update(self, move):
#        self.send_move_command(move)
#        self.update_beliefs_after_move(move)
#
#    def send_move_command(self, move_command):
#        self.move_requester(move_command)
#
#    def update_beliefs_after_move(self, move_command):
#        """
#        Update position beliefs after attempting a move based on the known
#        parameters of the motion model.
#        """
#        temp_probs = deepcopy(self.probability_matrix)
#
#        num_cols = len(temp_probs[0])
#        num_rows = len(temp_probs)
#        for i in range(num_rows):
#            for j in range(num_cols):
#                possible_starts = [
#                        [(i-1) % num_rows, j],
#                        [i, j],
#                        [(i+1) % num_rows, j],
#                        [i, (j-1) % num_cols],
#                        [i, (j+1) % num_cols]
#                ]
#                correct_start = [(i - move_command[0]) % num_rows, (j - move_command[1]) % num_cols]
#                incorrect_starts = deepcopy(possible_starts)
#                incorrect_starts.remove(correct_start)
#                #print possible_starts, incorrect_starts, correct_start
#                temp_probs[i][j] = self.config['prob_move_correct'] * self.probability_matrix[correct_start[0]][correct_start[1]]
#                for start in incorrect_starts:
#                    temp_probs[i][j] += ((1 - self.config['prob_move_correct'])/4) * temp_probs[start[0]][start[1]]
#	print "Move Probabilities"
#	self.show(temp_probs)
#
#        total_prob = sum(sum(p) for p in temp_probs)
#        if total_prob == 0:
#            print "sum of probabilities is zero"
#        else:
#            temp_probs = [[prob/total_prob for prob in row] for row in temp_probs]
#        self.probability_matrix = temp_probs
#	print "Move Probabilities NORMALIZED"
#	self.show(self.probability_matrix)


if __name__ == '__main__':
    pfl = ParticleFilterLocalization()
    r.random.seed(0)
    #rc = RobotController()
