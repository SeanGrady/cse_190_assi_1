#!/usr/bin/env python

import rospy
import json
import random as r
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData, moveService


class MapServer():
    def __init__(self):
        self.load_parameters()
        rospy.init_node("map_server")
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
        r.seed(self.seed)
        print "starting pos: ", self.pos
        rospy.spin()

    def load_parameters(self):
        with open('parameters.json') as param_file:
            param_dict = json.load(param_file)

        self.pipe_map = param_dict['pipe map']
        self.texture_map = param_dict['texture map']
        self.uncertain_motion = param_dict['uncertain motion']
        self.prob_move_correct = param_dict['prob move correct']
        self.moves = param_dict['move list']
        self.seed = param_dict['seed']
        self.starting_position = param_dict['starting pos']

    def initialize_position(self):
        """
        Set starting position.
        """
        pos = self.starting_position
        return pos

    def handle_data_request(self, request):
        """
        Map data service that provides ground truth temperature or texture
        data to other nodes.
        """
        if request.data_type == "temp":
            temp = self.pipe_map[self.pos[0]][self.pos[1]]
            return temp
        if request.data_type == "tex":
            tex = self.texture_map[self.pos[0]][self.pos[1]]
            return tex

    def handle_move_request(self, request):
        """
        Service that moves the robot according to a move request.
        self.uncertain_motion determines the motion model used: either
        certain motion or correct motion with a set probability (random
        otherwise).
        """
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
        """
        Changes the robot's position
        """
        num_rows = len(self.pipe_map)
        num_cols = len(self.pipe_map[0])
        self.pos[0] = (self.pos[0] + move[0]) % num_rows
        self.pos[1] = (self.pos[1] + move[1]) % num_cols
        print self.pos

    def handle_request_maps(self, request):
        """
        Might implement a way for the robot node to ask for the maps, rather
        than have the students hard-code it into the robot (the robot node
        does need to have the original pipe map and know how to go from that
        to actual temperature though)
        """
        pass


if __name__ == '__main__':
    ms = MapServer()
