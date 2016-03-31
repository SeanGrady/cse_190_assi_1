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

    def handle_incoming_temperature_data(self, message):
        temp = message.temperature
        #self.latest_temp = temp

    def request_texture_reading(self):
        response = self.texture_requester()
        texture_reading = response.texture
        return texture_reading

    def send_move_command(self, move_command):
        self.move_requester(move_command)
