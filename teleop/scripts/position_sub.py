#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from sensapex import UMP


class JointStateSubscriber:
    def __init__(self):
        self.history_size = 2
        self.position_stack = []
        rospy.init_node("joint_state_subscriber", anonymous=True)
        rospy.Subscriber("/Geomagic/joint_states", JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        current_positions = msg.position 

        if len(self.position_stack) >= self.history_size:
            self.position_stack.pop(0) 

        self.position_stack.append(current_positions)

        if len(self.position_stack) == self.history_size:
            last_positions = self.position_stack[0] 
            relative_positions = [current - last for current, last in zip(current_positions, last_positions)]
            rospy.loginfo("Relative Positions: %s", relative_positions)
        else:
            rospy.loginfo("Initial message received. No relative position available.")


class ump_control:
    def __init__(self, bimanual = False):
        self.ump = UMP.get_ump()
        self.manipulators = self.ump.list_devices()

        print(self.manipulators)

        self.fist_manipulator = self.ump.get_device(1)

        if bimanual:
            self.second_manipulator = self.ump.get_device(2)

if __name__ == "__main__":
    try:
        joint_state_subscriber = JointStateSubscriber()
        ump = ump_control()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
