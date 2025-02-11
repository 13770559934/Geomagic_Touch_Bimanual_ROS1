#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from geomagic_control.msg import DeviceButtonEvent
import socket
import pickle
import time


class JointStateSubscriber:
    def __init__(self):
        self.history_size = 2
        self.position_stack = []
        rospy.init_node("joint_state_subscriber", anonymous=True)
        self.button_subscriber = rospy.Subscriber("/Geomagic/button", DeviceButtonEvent, self.button_call_back)
        self.joint_state_subscriber = rospy.Subscriber("/Geomagic/joint_states", JointState, self.joint_state_callback)
        # !!!! remeber to Subscribe another button node!!!!!
        self.postion_server = Position_tcp()
        self.connected = False
        self.rate = rospy.Rate(10)

    def joint_state_callback(self, msg):
        current_positions = msg.position 

        if len(self.position_stack) >= self.history_size:
            self.position_stack.pop(0) 

        self.position_stack.append(current_positions)

        if len(self.position_stack) == self.history_size:
            last_positions = self.position_stack[0] 
            relative_positions = [current - last for current, last in zip(current_positions, last_positions)]
            # rospy.loginfo("Relative Positions: %s", relative_positions)
            if self.connected == True:
                self.postion_server.send(relative_positions)
        else:
            rospy.loginfo("Initial message received. No relative position available.")

        if not rospy.is_shutdown():
            rospy.loginfo('sleep')
            self.rate.sleep()
    
    def button_call_back(self, msg):
        if msg.grey_button == True: # if it is the grey button, begin to send position infromation
            rospy.loginfo(msg.grey_button)
            self.postion_server.connect()
            self.connected = True

        if msg.white_button == True: 
            rospy.loginfo(msg.white_button)

class Position_tcp:
    def __init__(self, ip = 'localhost', port = 1234):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((ip, port))
        self.client_socket = None
        self.client_addr = None

    def connect(self):
        self.server_socket.listen(1)
        self.client_socket, self.client_addr = self.server_socket.accept()
        rospy.loginfo(f"Accepted connection from {self.client_addr}")

    def send(self, relative_positions: list):
        self.client_socket.sendall(pickle.dumps(relative_positions))

if __name__ == "__main__":
    try:
        joint_state_subscriber = JointStateSubscriber()

        rospy.spin()
        # while not rospy.is_shutdown():
        #     joint_state_subscriber.rate.sleep()
    except rospy.ROSInterruptException:
        pass
