#!/usr/bin/env python3

import rospy
from bmb_actuators.real_aileron import RealAileron


def main():
    rospy.init_node("real_aileron_node")
    node = RealAileron()
    node.spin()


if __name__ == '__main__':
    main()
