#!/usr/bin/env python

import rospy
import random as r
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData, moveService
from cse_190_assi_1.msg import Move
from std_msgs.msg import Bool
from read_config import read_config


class MapServer():
    def __init__(self):
        """Read config file and setup ROS things"""
        self.config = read_config()
        rospy.init_node("map_server")
        self.map_data_service = rospy.Service(
                "requestMapData",
                requestMapData,
                self.handle_data_request
        )
        self.timer_sub = rospy.Subscriber(
                "/sim/timer",
                Bool,
                self.handle_time_tick
        )
        self.move_publisher = rospy.Publisher(
                "/map_server/move",
                Move,
                queue_size = 10
        )
        self.pos = self.initialize_position()
        print "starting pos: ", self.pos
        self.motions = self.config['move_list'][::-1]
        rospy.spin()

    def initialize_position(self):
        """Set starting position."""
        pos = self.config["starting_pos"]
        return pos

    def handle_data_request(self, request):
        """Service to provide ground truth data to sensors"""
        if request.data_type == "temp":
            temp = self.config['pipe_map'][self.pos[0]][self.pos[1]]
            return temp
        if request.data_type == "tex":
            tex = self.config['texture_map'][self.pos[0]][self.pos[1]]
            return tex

    def handle_time_tick(self, message):
        if not self.motions:
            print "out of moves, shutting down"
            rospy.signal_shutdown("sim complete")
        move = self.motions.pop()
        self.move_publisher.publish(move)

    def publish_move(self, move):
        move_msg = Move()
        move_msg.move = move
        self.move_publisher.publish(move_msg)

    def handle_move(self, move):
        """Service that moves the robot according to a move request.

        self.config['uncertain_motion'] determines the motion model
        used: either certain motion or motion with a set probability
        (randomotherwise).
        """
        if self.config['uncertain_motion']:
            roll = r.uniform(0,1)
            if roll < self.config["prob_move_correct"]:
                self.make_move(move)
            else:
                possible_moves = deepcopy(self.config['possible_moves'])
                possible_moves.remove(move)
                random_move = r.choice(possible_moves)
                self.make_move(random_move)
                self.publish_move(random_move)
        elif not self.config['uncertain_motion']:
            self.make_move(move)
            self.publish_move(move)

    def make_move(self, move):
        """Changes the robot's position"""
        num_rows = len(self.config['pipe_map'])
        num_cols = len(self.config['pipe_map'][0])
        self.pos[0] = (self.pos[0] + move[0]) % num_rows
        self.pos[1] = (self.pos[1] + move[1]) % num_cols
        #print self.pos


if __name__ == '__main__':
    ms = MapServer()
