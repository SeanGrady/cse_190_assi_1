#!/usr/bin/env python

import rospy
import random as r
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData, moveService


class MapServer():
    def __init__(self):
        self.uncertain_motion = True
        self.prob_move_correct = .75
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
        self.move_list = [
                [0,0],
                [1,0],
                [0,1],
                [-1,0],
                [0,-1]
        ]
        self.seed = 0
        r.seed(self.seed)
        print "starting pos: ", self.pos
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
        move = list(request.move)
        if self.uncertain_motion:
            roll = r.uniform(0,1)
            if roll < self.prob_move_correct:
                self.make_move(move)
            else:
                possible_moves = deepcopy(self.move_list)
                possible_moves.remove(move)
                random_move = r.choice(possible_moves)
                self.make_move(random_move)
        elif not self.uncertain_motion:
            self.make_move(move)
        return []

    def make_move(self, move):
        num_rows = len(self.pipe_map)
        num_cols = len(self.pipe_map[0])
        self.pos[0] = (self.pos[0] + move[0]) % num_rows
        self.pos[1] = (self.pos[1] + move[1]) % num_cols
        print self.pos

    def handle_request_maps(self, request):
        pass


if __name__ == '__main__':
    ms = MapServer()
