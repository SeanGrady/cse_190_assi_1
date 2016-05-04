#!/usr/bin/env python

import rospy
import random
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.srv import requestMapData, requestTemperature
from std_msgs.msg import Bool
from read_config import read_config


class TempSensor():
    def __init__(self):
        """Read config file and setup ROS things"""
        self.config = read_config()
        rospy.init_node("temperature_sensor")
        self.temperature_requester = rospy.ServiceProxy(
                "requestMapData",
                requestMapData
        )
        self.temperature_service = rospy.Service(
                "requestTemperature",
                requestTemperature,
                self.handle_temperature_request
        )
        self.temp_dict = {
                'H': 40.0,
                'C': 20.0,
                '-': 25.0
        }
        rospy.spin()

    def handle_temperature_request(self, request):
        temperature = self.take_measurement()
        noisy_temp = self.add_noise(temperature)
        return noisy_temp

    def take_measurement(self):
        """Get the temperature of the current square."""
        temp_response = self.temperature_requester('temp')
        temperature = temp_response.data
        temp = self.temp_dict[temperature]
        return temp

    def add_noise(self, true_val):
        """Returns measurement after adding Gaussian noise."""
        noise = m.ceil(random.gauss(0, self.config['temp_noise_std_dev'])*100.)/100.
        noisy_measurement = true_val + noise
        return noisy_measurement


if __name__ == '__main__':
    ts = TempSensor()
