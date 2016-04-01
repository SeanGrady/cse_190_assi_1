#!/usr/bin/env python

import rospy
import random as r
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.msg import temperatureMessage
from cse_190_assi_1.srv import requestTexture, moveService

class RobotController():
    def __init__(self):
        rospy.init_node("robot_controller")
        self.temperature_subscriber = rospy.Subscriber(
                "/temp_sensor/data",
                temperatureMessage,
                self.handle_incoming_temperature_data
        )
        self.texture_requester = rospy.ServiceProxy(
                "requestTexture",
                requestTexture,
        )
        self.move_requester = rospy.ServiceProxy(
                "moveService",
                moveService,
        )
        self.initialize_constants()
        self.set_motion_commands()
        self.initialize_maps()
        self.explore()

    def initialize_constants(self):
        self.prob_texture_correct = 0.99
        self.prob_move_correct = 1.0
        std_dev_temp_sensor = 10

    def set_motion_commands(self):
        self.move_list = [[0,0],[0,1],[1,0],[1,0],[0,1],[0,1]]

    def initialize_maps(self):
        pipe_map = [['C','-','H','H','-'],
                    ['C','-','H','-','-'],
                    ['C','-','H','-','-'],
                    ['C','C','H','H','H']]
        self.heat_map = self.generate_heatmap(pipe_map)
        self.texture_map = [['S','S','S','S','R'],
                            ['R','R','S','R','R'],
                            ['R','S','S','S','S'],
                            ['S','R','R','S','R']]

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

    def begin_simulation(self):
        pass

    def handle_incoming_temperature_data(self, message):
        '''
        because of timesteps and things, this is the main sense->move->
        evaluate->repeate loop
        '''
        temp = message.temperature


    def request_texture_reading(self):
        response = self.texture_requester()
        texture_reading = response.data
        return texture_reading

    def send_move_command(self, move_command):
        self.move_requester(move_command)


if __name__ == '__main__':
    rc = RobotController()
