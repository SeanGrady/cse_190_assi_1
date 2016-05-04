#!/usr/bin/env python

import rospy
import random as r
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.msg import temperatureMessage, RobotProbabilities
from cse_190_assi_1.srv import requestTexture, moveService
from std_msgs.msg import Bool, String, Float32
from read_config import read_config

class RobotController():
    def __init__(self):
        self.config = read_config()
        rospy.init_node("robot_controller")
        self.init_ros_things()
        self.initialize_maps()
        self.initialize_beliefs()
        self.motions = self.config['move_list'][::-1]
        self.rate = rospy.Rate(.5)
        self.simulate()

    def simulate(self):
        while not rospy.is_shutdown() and len(self.motions) < 0:
            temp = self.get_temperature()
            self.update_from_temp(temp)

            text = self.get_texture()
            self.update_from_text(text)

            move = self.motions.pop()
            self.send_move_command(move)
            self.update_from_move(move)

            self.print_probs()
            self.rate.sleep()

    def init_ros_things(self):
        self.temperature_requester = rospy.ServiceProxy(
                "requestTemperature",
                requestTemperature
        )
        self.texture_requester = rospy.ServiceProxy(
                "requestTexture",
                requestTexture
        )
        self.move_requester = rospy.ServiceProxy(
                "moveService",
                moveService,
        )

    def initialize_maps(self):
        self.heat_map = self.generate_heatmap(self.config['pipe_map'])

    def initialize_beliefs(self):
        """
        set the initial probability matrix - everywhere is equally likely.
        """
        init_prob = 1.0 / float(len(self.config['texture_map'])) / float(len(self.config['texture_map'][0]))
        starting_beliefs = [[init_prob for row in range(len(self.config['texture_map'][0]))]
                            for col in range(len(self.config['texture_map']))]
        self.probability_matrix = starting_beliefs

    def generate_heatmap(self, pipe_map):
        """
        Generates a heat map based on whether there is a cold or hot pipe
        running under each tile. Tiles with no pipes underneath will have
        a "warm" temperature
        """
        temp_cold = 20.0
        temp_hot  = 40.0
        temp_warm = 25.0
        heat_map = [[ 0.0 for x in row] for row in pipe_map]
        for i in range(len(pipe_map)):
            for j in range(len(pipe_map[0])):
                if pipe_map[i][j] == 'C':
                    heat_map[i][j] = temp_cold

                elif pipe_map[i][j] == 'H':
                    heat_map[i][j] = temp_hot
                else:
                    heat_map[i][j] = temp_warm
        return heat_map

    def send_move_command(self, move):
        self.move_requester(move)

    def get_texture(self):
        """
        Get a texture reading from the texture sensor node via the
        texture request service.
        """
        response = self.texture_requester()
        texture_reading = response.data
        return texture_reading

    def get_temperature(self):
        """
        Get a temperature reading from the temperature sensor node via the
        temperature request service.
        """
        response = self.temperature_requester()
        temperature_reading = response.data
        return temperature_reading

    def update_from_move(self, move_command):
        """
        Update position beliefs after attempting a move based on the known
        parameters of the motion model.
        """
        temp_probs = deepcopy(self.probability_matrix)

        num_cols = len(temp_probs[0])
        num_rows = len(temp_probs)
        for i in range(num_rows):
            for j in range(num_cols):
                possible_starts = [
                        [(i-1) % num_rows, j],
                        [i, j],
                        [(i+1) % num_rows, j],
                        [i, (j-1) % num_cols],
                        [i, (j+1) % num_cols]
                ]
                correct_start = [(i - move_command[0]) % num_rows, (j - move_command[1]) % num_cols]
                incorrect_starts = deepcopy(possible_starts)
                incorrect_starts.remove(correct_start)
                temp_probs[i][j] = self.config['prob_move_correct'] * self.probability_matrix[correct_start[0]][correct_start[1]]
                for start in incorrect_starts:
                    temp_probs[i][j] += ((1 - self.config['prob_move_correct'])/4) * temp_probs[start[0]][start[1]]

        total_prob = sum(sum(p) for p in temp_probs)
        if total_prob == 0:
            print "sum of probabilities is zero"
        else:
            temp_probs = [[prob/total_prob for prob in row] for row in temp_probs]
        self.probability_matrix = temp_probs

    def update_from_temp(self, temp_reading):
        """
        Update the beliefs about the robot's position based on a temperature
        measurement.
        """

        temp_probs = deepcopy(self.probability_matrix)
        num_cols = len(temp_probs[0])
        num_rows = len(temp_probs)
        for i in range(num_rows):
            for j in range(num_cols):
                if self.config['temp_noise_std_dev'] == 0:
                    prob_hit = heat_map[i][j] == temp_reading
                else:
                    prob_hit = 1/(m.sqrt(2*m.pi)*self.config['temp_noise_std_dev'])*m.e**(-0.5*(float(self.heat_map[i][j] - temp_reading)/self.config['temp_noise_std_dev'])**2)
                temp_probs[i][j] = prob_hit * self.probability_matrix[i][j]

        total_prob = sum(sum(p) for p in temp_probs)
        if total_prob == 0:
            print "sum of probabilities is zero"
        else:
            temp_probs = [[prob/total_prob for prob in row] for row in temp_probs]
        self.probability_matrix = temp_probs

    def update_from_text(self, texture_reading):
        """
        Update the beliefs about the robot's position based on a texture
        measurement.
        """
        temp_probs = deepcopy(self.probability_matrix)
        num_cols = len(temp_probs[0])
        num_rows = len(temp_probs)
        for i in range(num_rows):
            for j in range(num_cols):
                hit = int(self.config['texture_map'][i][j] == texture_reading)
                temp_probs[i][j] = (self.config['prob_tex_correct'] * hit + (1 - self.config['prob_tex_correct']) * (1 - hit)) * self.probability_matrix[i][j]

        total_prob = sum(sum(p) for p in temp_probs)
        if total_prob == 0:
            print "sum of probabilities is zero"
        else:
            temp_probs = [[prob/total_prob for prob in row] for row in temp_probs]
        self.probability_matrix = temp_probs

    def show(self):
        p = self.probability_matrix
        rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x),r)) + ']' for r in p]
        print '[' + ',\n '.join(rows) + ']'

if __name__ == '__main__':
    rc = RobotController()
