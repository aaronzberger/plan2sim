#!/usr/bin/python3

import rospy
import sys
import os
from converter import Converter


class Plan2Sim:
    def __init__(self, plan_file):
        self.converter_node = Converter(plan_file)
        self.converter_node.convert()
        self.interval_tree = self.converter_node.get_timeline()


if __name__ == "__main__":
    rospy.init_node('plan2sim', log_level=rospy.INFO)

    if len(sys.argv) != 2 or not os.path.exists(sys.argv[1]):
        rospy.logfatal('Usage: rosrun plan2sim [PLAN FILE]')
        sys.exit()

    plan2sim_node = Plan2Sim(sys.argv[1])

    rospy.spin()
