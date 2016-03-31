#!/usr/bin/env python

import rospy
import random as r
import math as m
import numpy as np
from copy import deepcopy

class RobotController():
    def __init__(self):
        rospy.init_node("robot_controller")
        self.temperature_subscriber = rospy.Subscriber(
                "/temp_sensor/data",
                TemperatureMessage,
                self.handle_incoming_temperature_data
        )
        self.texture_requester = rospy.ServiceProxy(
                "requestTexture",
                requestTexture,
        )
        self.move_command_publisher = rospy.Publisher(
                "/robot/move_commands",
                moveCommand,
                queue_size = 10
        )

    def handle_incoming_temperature_data(self, message):
        temp = message.temperature
        #self.latest_temp = temp

    def request_texture_reading(self):
        response = self.texture_requester()
        texture_reading = response.texture
        return texture_reading

    def send_move_command(self, move_command):
        self.move_command_publisher.publish(move_command)
