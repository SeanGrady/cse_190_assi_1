#!/usr/bin/env python

import rospy
import json
import random as r
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData, requestTexture 

class TexSensor():
    def __init__(self):
        self.load_parameters()
        rospy.init_node("texture_sensor")
        self.texture_requester = rospy.ServiceProxy(
                "requestMapData",
                requestMapData
        )
        self.texture_service = rospy.Service(
                "requestTexture",
                requestTexture,
                self.handle_texture_request
        )
        r.seed(self.seed)
        rospy.sleep(1)
        rospy.spin()

    def load_parameters(self):
        with open('parameters.json') as param_file:
            param_dict = json.load(param_file)

        self.prob_correct = param_dict['prob tex correct']
        self.seed = param_dict['seed']

    def handle_texture_request(self, request):
        """
        Callback function for the texture service that the robot node uses.
        """
        texture = self.take_measurement()
        noisy_texture = self.add_noise(texture)
        return noisy_texture

    def take_measurement(self):
        """
        Get the texture of the current square from the map node via
        the map data service.
        """
        tex_response = self.texture_requester('tex')
        tex = tex_response.data
        return tex

    def add_noise(self, measurement):
        roll = r.uniform(0,1)
        if roll < self.prob_correct:
            return measurement
        else:
            if measurement == 'R':
                return 'S'
            elif measurement == 'S':
                return 'R'


if __name__ == '__main__':
    ts = TexSensor()
