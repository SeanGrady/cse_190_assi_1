#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from read_config import read_config

class Timer():
    def __init__(self):
        self.config = read_config()
        rospy.init_node('timer')
        self.timer_pub = rospy.Publisher(
                '/sim/timer',
                Bool,
                queue_size = 10
        )
        self.rate = rospy.Rate(self.config['rate'])
        self.timer_loop()

    def timer_loop(self):
        while not rospy.is_shutdown():
            self.timer_pub.publish(True)
            self.rate.sleep()

if __name__ == '__main__':
    tm = Timer()
