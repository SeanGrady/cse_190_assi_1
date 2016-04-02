#!/usr/bin/env python

import rospy
import random as r
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData, moveService

class MapServer():
    def __init__(self):
        self.uncertain_motion = False
        rospy.init_node("map_server")
        self.pipe_map = [['C','-','H','H','-'],
                         ['C','-','H','-','-'],
                         ['C','-','H','-','-'],
                         ['C','C','H','H','H']]
        self.texture_map = [['S','S','S','S','R'],
                            ['R','R','S','R','R'],
                            ['R','S','S','S','S'],
                            ['S','R','R','S','R']]
        self.map_data_service = rospy.Service(
                "requestMapData",
                requestMapData,
                self.handle_data_request
        )
        self.move_service = rospy.Service(
                "moveService",
                moveService,
                self.handle_move_request
        )
        self.pos = self.initialize_position()
        rospy.spin()

    def initialize_position(self):
        pos = [3, 3]
        return pos

    def handle_data_request(self, request):
        if request.data_type == "temp":
            temp = self.pipe_map[self.pos[0]][self.pos[1]]
            return temp
        if request.data_type == "tex":
            tex = self.texture_map[self.pos[0]][self.pos[1]]
            return tex

    def handle_move_request(self, request):
        move = request.move
        if self.uncertain_motion:
            pass
        else:
            #self.pos = [i + j for i, j in zip(self.pos, move)]
            num_rows = len(self.pipe_map)
            num_cols = len(self.pipe_map)
            self.pos[0] = (self.pos[0] + move[0]) % num_rows
            self.pos[1] = (self.pos[1] + move[1]) % num_cols
        return []

    def handle_request_maps(self, request):
        pass

    '''
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
    '''


if __name__ == '__main__':
    ms = MapServer()
