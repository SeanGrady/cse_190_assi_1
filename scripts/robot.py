#!/usr/bin/env python

import rospy
import random as r
import math as m
import numpy as np
from std_msgs.msg import Bool, String, Float32
from cse_190_assi_3.msg import AStarPath, PolicyList
from read_config import read_config
from mdp_em import MarkovDecisionProcessWithEM
from a_star import AStarSearch


class RobotController():
    def __init__(self):
        self.config = read_config()
        rospy.init_node("robot_controller")

        self.path_list_result = rospy.Publisher(
                "/results/path_list",
                AStarPath,
                queue_size = 10
        )
        self.policy_list_result = rospy.Publisher(
                "/results/policy_list",
                PolicyList,
                queue_size = 10
        )
        self.shutdown_pub = rospy.Publisher(
                "/map_node/sim_complete",
                Bool,
                queue_size = 10
        )
        rospy.sleep(2)
        self.init()

    def init(self):
        self.a_star_instance = AStarSearch()
        self.mdp_em_instance = MarkovDecisionProcessWithEM()

        self.start_search()
        
    def start_search(self):
        self.path_list = self.a_star_instance.a_start_search()
        for step in self.path_list:
            self.path_list_result.publish(step)
        print "A Star Search Over"
        
        self.result_policy_list = self.mdp_em_instance.value_iteration()
        for i in range(len(self.result_policy_list)):
            policy = self.result_policy_list[i]
            data_to_publish = [p for row in policy for p in row]
            self.policy_list_result.publish(data_to_publish)
            rospy.sleep(0.25)
        print "MDP Value Iteration Over"
        
        rospy.sleep(2)
        self.end_simulation()

    def end_simulation(self):
        """
        Publishes to the topic that stops the temperature sensor at the
        end of the simulation.
        """
        self.shutdown_pub.publish(True)
        rospy.sleep(1)
        rospy.signal_shutdown("Searched Enough")

if __name__ == '__main__':
    rc = RobotController()
