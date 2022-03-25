import rospy
from bmb_msgs.msg import ControlInputs


class RealAileron:
    def __init__(self):
        self.control_inputs_sub = rospy.Subscriber(
            "control_inputs", ControlInputs, self.control_inputs_callback)

    def control_inputs_callback(self, control_inputs):
        right_aileron_angle = control_inputs.right_aileron_angle
        # TODO: replace linear model with 4-bar linkage inverse kinematics
        servo_angle = right_aileron_angle * (41 / 30)
        # TODO: send command to servo

    @staticmethod
    def spin():
        rospy.spin()
