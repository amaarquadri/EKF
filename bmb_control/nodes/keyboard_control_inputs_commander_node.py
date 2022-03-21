#!/usr/bin/env python3

import rospy
from bmb_control.keyboard_control_inputs_commander import KeyboardControlInputsCommander


def main():
    rospy.init_node("keyboard_control_inputs_commander_node")
    node = KeyboardControlInputsCommander()
    node.spin()


if __name__ == '__main__':
    main()
