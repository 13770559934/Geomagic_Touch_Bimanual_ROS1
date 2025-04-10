#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geomagic_control.msg import DeviceButtonEvent
import socket
import pickle
import threading
import time 


class JointStateSubscriber:
    # every device should have its own 'JointStateSubscriber' class
    def __init__(self, 
                 node_name:str,
                 state_topic_name:str,
                 button_topic_name:str,
                 position_port:int,
                 button_port:int
                 ):
        
        # variables used to calcualte the relative position 
        self.history_size = 2
        self.position_stack = []

        # rospy.init_node(node_name, anonymous=True)

        # create all the subscriber class
        self.button_subscriber = rospy.Subscriber(button_topic_name, DeviceButtonEvent, self.button_call_back)
        self.joint_state_subscriber_right = rospy.Subscriber(state_topic_name, Joy, self.joint_state_callback)

        # all the server class
        self.position_server = Position_udp(port = position_port)
        self.button_server = Position_udp(port = button_port)

        self.rate = rospy.Rate(rospy.get_param('publish_rate', 10))

    def joint_state_callback(self, msg):
        current_positions = msg.axes[:3]

        #using stack to store the last and current  positions
        if len(self.position_stack) >= self.history_size:
            self.position_stack.pop(0) 
        self.position_stack.append(current_positions)

        if len(self.position_stack) == self.history_size:
            last_positions = self.position_stack[0] 
            self.relative_positions = [current - last for current, last in zip(current_positions, last_positions)]

            # rospy.loginfo("Relative Positions: %s", relative_positions)
            self.position_server.send(self.relative_positions)
        else:
            rospy.loginfo("Initial message received. No relative position available.")
    
    def button_call_back(self, msg):
        self.button_server.send([msg.grey_button, msg.white_button])

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

        
class Position_udp:
    def __init__(self, ip='localhost', port=1234):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ip = ip
        self.port = port

    def send(self, relative_positions: list):
        relative_positions.append(time.time())
        print(relative_positions, len(relative_positions))
        self.socket.sendto(pickle.dumps(relative_positions), (self.ip, self.port))

def run_device(device):
    device.run()


if __name__ == "__main__":
    try:

        rospy.init_node('joint_state_subscriber_node', anonymous=True)

        # Create threads for each device
        left_device = JointStateSubscriber(
            state_topic_name="/Geomagic/joy_LeftDevice",
            button_topic_name="/Geomagic/button_LeftDevice",
            position_port=1234,
            button_port=1235
        )
        right_device = JointStateSubscriber(
            state_topic_name="/Geomagic/joy_RightDevice",
            button_topic_name="/Geomagic/button_RightDevice",
            position_port=4321,
            button_port=4322
        )

        # Create threads for each device
        left_thread = threading.Thread(target=run_device, args=(left_device,))
        right_thread = threading.Thread(target=run_device, args=(right_device,))

        # Start the threads
        left_thread.start()
        right_thread.start()

        # Wait for the threads to finish (they won't, since rospy.spin() is blocking)
        left_thread.join()
        right_thread.join()

    except rospy.ROSInterruptException:
        pass