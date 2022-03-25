import rospy
from bmb_msgs.msg import ControlInputs


class RealPropeller:
    def __init__(self):
        self.control_inputs_sub = rospy.Subscriber(
            "control_inputs", ControlInputs, self.control_inputs_callback)

    def control_inputs_callback(self, control_inputs):
        propeller_force = control_inputs.propeller_force
        # TODO: calculate voltage and send to motor

    @staticmethod
    def spin():
        rospy.spin()
