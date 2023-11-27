#!/usr/bin/env python
import socket
import rospy 
from std_msgs.msg import String

rospy.init_node('udp2ros_qr_bridge', anonymous=True)
rate = rospy.Rate(10)
print("UDP to ROS QR Bridge started")

pub = rospy.Publisher('/qr_ids', String, queue_size=10)


print("UDP to ROS QR Bridge ended")

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Define the server address and port
server_address = ('130.251.13.36', 12345)

# Bind the socket to a specific address and port
sock.bind(server_address)

while not rospy.is_shutdown():
    # Receive data from clients
    data, address = sock.recvfrom(4096)
    print('Received:', data.decode(), 'from:', address)


# Close the socket
sock.close()

