#!/usr/bin/env python

import rospy
import random as r
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData, requestTexture 

class TexSensor():
    def __init__(self):
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
        self.prob_correct = 0.99
        self.seed = 0
        r.seed(self.seed)
        rospy.spin()

    def handle_texture_request(self, request):
        texture = self.take_measurement()
        noisy_texture = self.add_noise(texture)
        return noisy_texture

    def take_measurement(self):
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
