#!/usr/bin/env python

import rospy
import numpy as np
import random as r
from math import *
from copy import deepcopy
from cse_190_assi_2.msg import temperatureMessage, RobotProbabilities
from cse_190_assi_2.srv import requestTexture, moveService
from std_msgs.msg import Bool, String, Float32
from collections import deque
from read_config import read_config

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PoseArray, PointStamped, Quaternion, Point
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
	self._pose = Pose()
        self._pose.position.x = self.x
        self._pose.position.y = self.y
        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        self._pose.orientation.x = quat[0]
        self._pose.orientation.y = quat[1]
        self._pose.orientation.z = quat[2]
        self._pose.orientation.w = quat[3]

    @property
    def x(self):
        """ x position in meters"""
	print "x getter"
        return self._pose.position.x

    @x.setter
    def x(self, x):
	print "x setter"
        self._pose.position.x = x

    @property
    def y(self):
        """ y position in meters"""
        return self._pose.position.y

    @y.setter
    def y(self, y):
        self._pose.position.y = y

    @property
    def theta(self):
        """ Orientation in radians. """
        quat = np.array([self._pose.orientation.x,
                         self._pose.orientation.y,
                         self._pose.orientation.z,
                         self._pose.orientation.w])
        euler = tf.transformations.euler_from_quaternion(quat)
        return euler[2]

    @theta.setter
    def theta(self, theta):
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        self._pose.orientation.x = quat[0]
        self._pose.orientation.y = quat[1]
        self._pose.orientation.z = quat[2]
        self._pose.orientation.w = quat[3]

    @property
    def pose(self):
        """ Pose of this particle as a geometry_msgs/Pose object """
        return self._pose

    @pose.setter
    def pose(self, pose):
        self._pose = copy.copy(pose)

        # Now normalize the quaternion.
        quat = np.array([pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w])
        quat = tf.transformations.unit_vector(quat)
        self._pose.orientation.x = quat[0]
        self._pose.orientation.y = quat[1]
        self._pose.orientation.z = quat[2]
        self._pose.orientation.w = quat[3]

class ParticleFilterLocalization():


    def __init__(self):
        self.config = read_config()
        rospy.init_node("particle_filter_localization")
	

	#Step1: Test Particle Addition
	#Ref: http://wiki.ros.org/amcl
        global likelihood_field 
	global laser_z_hit 
	global laser_z_rand 
	global laser_sigma_hit 

        likelihood_field = None
	laser_z_hit = 0.95
	laser_z_rand = 0.05
	laser_sigma_hit = 0.2

	#tf::TransformBroadcaster br;
	#tf::Transform transform;
	#transform.setOrigin( tf::Vector3(x,y,z)); // position in respect to the world frame
	#transform.setRotation( tf::Quaternion(x,y,z,w) ); // orientation in respect to the world frame
	#br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot_n"));



        self.map = None
        self.num_particles = 1000
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        #rospy.Subscriber('particle_pub_start', PoseArray, self.particle_pub_start_callback)
        #rospy.Subscriber('initialpose', PoseWithCovarianceStamped,self.initial_pose_callback)

        self.particle_pub = rospy.Publisher('particlecloud', PoseArray, queue_size = 10, latch=True)
        #self.particle_pub_start = rospy.Publisher('particle_pub_start', PoseArray, 10, latch=True)
        self.likelihood_pub = rospy.Publisher('likelihood_field', OccupancyGrid, queue_size = 10, latch=True)

        # We need the map before we can initialize the particles.
        while not rospy.is_shutdown() and self.map is None:
            rospy.loginfo("Waiting for map.")
            rospy.sleep(.5)


	self.init_Particles()

        self.particle_pub.publish(self.pose_array)
	rospy.sleep(1)	

        if likelihood_field is None:
            self.update_likelihood_field(self.map, laser_sigma_hit)

	rospy.spin()	

	""" Waiting Until Shutdown"""
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		r.sleep()	


    def init_Particles(self):
        self.particles = []
        for particle_count in range(self.num_particles):
		new_particle = Particle(self.map, 1/self.num_particles)
		self.particles.append(new_particle)
        # Publish the newly created particles.
        self.pose_array = self.create_pose_array_msg()
		

    def create_pose_array_msg(self):
        """Create a PoseArray object including the poses of all particles.

        Returns: geometry_msgs/PoseArray
        """
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
	print pose_array.header.stamp
        pose_array.header.frame_id = 'map'
        pose_array.poses = []
        for p in self.particles:
            pose_array.poses.append(p.pose)
        return pose_array

    def map_callback(self, map_msg):
        """ Store the map message in an instance variable. """
        self.map = map_msg

    def update_likelihood_field(self, map_msg, laser_sigma):
        """The likelihood field is essentially a map indicating which
        locations are likely to result in a laser hit.  Points that
        are occupied in map_msg have the highest probability, points
        that are farther away from occupied regions have a lower
        probability.  The laser_sigma argument controls how quickly
        the probability falls off.  Ideally, it should be related to
        the precision of the laser scanner being used.

        """

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
        likelihood_field = map_utils.Map(world_map.to_message())
        dists = kdt.query(all_positions, k=1)[0][:]
        probs = np.exp(-(dists**2) / (2 * laser_sigma**2))

        likelihood_field.grid = probs.reshape(likelihood_field.grid.shape)

        rospy.loginfo('Done building likelihood field')


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
    #rc = RobotController()
