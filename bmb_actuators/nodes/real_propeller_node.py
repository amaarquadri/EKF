#!/usr/bin/env python3

import rospy
from bmb_actuators.real_propeller import RealPropeller


def main():
    rospy.init_node("real_propeller_node")
    node = RealPropeller()
    node.spin()


if __name__ == '__main__':
    main()
