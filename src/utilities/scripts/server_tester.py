#!/usr/bin/env python
import socket   #for sockets
import sys	#forexit
import thread
import atexit
import threading
import rospy
from utilities.srv import *
import server

def main():
	#initiate ros, robot, assign variables...
	rospy.init_node("socket_server_tester")

	rospy.wait_for_service(server.SERVICE_NAME_INTERACT)
	try:
		srv_h=rospy.ServiceProxy(server.SERVICE_NAME_INTERACT,android_interact)
		resp =srv_h("Hi, do you want some water?", "I repeat, do you want some water? Yes or no", "OK", ["yes","no"])
	except rospy.ServiceException, e:
		print "service dall failed: %s"%e

	print "response from android: ",resp.index

	return 0

if __name__ =="__main__":
	main()
