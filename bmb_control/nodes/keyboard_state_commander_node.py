#!/usr/bin/env python3

import rospy
from bmb_control.keyboard_state_commander import KeyboardStateCommander


def main():
    rospy.init_node("keyboard_state_commander_node")
    node = KeyboardStateCommander()
    node.spin()


if __name__ == '__main__':
    main()
