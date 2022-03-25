#!/usr/bin/env python3

import rospy
from bmb_actuators.real_elevator import RealElevator


def main():
    rospy.init_node("real_elevator_node")
    node = RealElevator()
    node.spin()


if __name__ == '__main__':
    main()
