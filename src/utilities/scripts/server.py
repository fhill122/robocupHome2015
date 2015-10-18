#!/usr/bin/env python
import socket   #for sockets
import sys	#forexit
import thread
import atexit
import threading
import rospy
from utilities.srv import *

HOST = ''   # Symbolic name meaning all available interfaces
PORT = 8888 # Arbitrary non-privileged port
conn = None
addr = None
data_for_ros = None # data received only for ros service call
s = None # socket
ros_called =  False
ros_request = None

SERVICE_NAME = 'send_to_android'

def main():
	#initiate ros, robot, assign variables...
	rospy.init_node("socket_server")
	print "total threads: ",threading.activeCount()
	start_server()
	return


def server_shutdown(s):
    s.close()
    print "socket closed, exiting...."

def start_server():
	global PORT, conn, addr, data_for_ros,data_received, s
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	print 'Socket created'
	atexit.register(server_shutdown,s)

	bindFail =1
	while bindFail:
		bindFail = 0
		try:
			s.bind((HOST, PORT))
		except socket.error , msg:
	##        print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message: ' + msg[1]
	##        sys.exit()
			PORT=PORT+1
			bindFail=1

	print 'Socket bind complete: ',PORT

	s.listen(10)
	print 'Socket now listening'

	# loop to wait for connect
	while 1:
		#wait to accept a connection - blocking call
		conn, addr = s.accept()
		print 'Connected with ' + addr[0] + ':' + str(addr[1])
		conn.send('Welcome to the server.\n')

		# start rosservice only when connection is established
		ros_s = rospy.Service(SERVICE_NAME , voiceCommand, handle_send_command)

		#accept data flow in
		while 1:
			# check if data flow ends
			print"waiting data from client...."
			data_received = conn.recv(1024)
			print addr[0]+":",PORT,":"+str(addr[1])+":"+data_received

			if not data_received:
				break

			# ros service not called
			if (ros_request == None):
				print "not a respond to ros command"

				if (data_received!="received\n"):
					conn.sendall("Ok.. I heard you\n")
			# ros service called
			else:
				print "this is a respond to ros command"
				data_for_ros = data_received


		#data inflow end, close ros server
		data_for_ros = None
		ros_s.shutdown("ros service shutdown: socket connection end")
		print "connection end"

# service call back
# format:
# /ask will speak string "ask" at android client, prompt for speech input, send back the speech result
# ask just speak ask at android client, user has to manually send the response
# .ask will just speak at android, no response is required, set "done" to ROS service response
def handle_send_command(req):
	global ros_request, data_for_ros

	if (req.request) [0] == ".":
		conn.sendall(req.request[0:]+"\n")
		return "done"

	conn.sendall(req.request+"\n")
	
	ros_request = req.request #or just any value?

	while(data_for_ros == None):
		rospy.sleep(0.1)
		# print "checking data_for_ros"
		continue

	#end
	temp = data_for_ros
	ros_request = None
	data_for_ros = None
	return  temp

if __name__ =="__main__":
	main()
