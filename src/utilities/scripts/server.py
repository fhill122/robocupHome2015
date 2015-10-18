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

REPEAT_TIMES = 2

SERVICE_NAME = 'socket_android/command_android'
SERVICE_NAME_INTERACT = 'socket_android/android_interact'

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
        ros_s_i = rospy.Service(SERVICE_NAME_INTERACT, android_interact, handle_android_interact)
        

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
    ros_s.shutdown(SERVICE_NAME+": ros service shutdown: socket connection end")
    ros_s_i.shutdown(SERVICE_NAME_INTERACT+": ros service shutdown: socket connection end")
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

# service call back
# see android_interact.srv for format:
def handle_android_interact(req):
    ask = req.ask
    repeat = req.repeat
    confirm = req.confirm
    exp_ans = req.exp_ans
    
    index = getProperAns(ask,repeat,confirm,exp_ans)
    
    return index
    
#return the index in exp_ans, will ask first, then repeat REPEAT_TIMES, then
def getProperAns(ask,repeat,confirm,exp_ans):

	ans = getAndroidAns("/"+ask)
	index = findAnsIndex(exp_ans,ans)
	if (index != -1):
		getAndroidAns("."+confirm)
		return index
	print "failed at 1 first ask"

	# repeat if not got expected ans
	for i in range(REPEAT_TIMES):
		ans = getAndroidAns("/"+repeat)
		index = findAnsIndex(exp_ans,ans)
		if (index != -1):
			getAndroidAns("."+confirm)
			return index
	print "failed at repeat stage"

	# let user input until an answer is got
	while(True):
		ans = getAndroidAns(repeat)
		index = findAnsIndex(exp_ans,ans)
		if (index != -1):
			getAndroidAns("."+confirm)
			return index

#just get response from calling ros service
def getAndroidAns(ask):
	rospy.wait_for_service(SERVICE_NAME)
	try:
		srv_h=rospy.ServiceProxy(SERVICE_NAME,voiceCommand)
		resp =srv_h(ask)
	except rospy.ServiceException, e:
		print "service dall failed: %s"%e

	print "response from android: "+resp.response
	return resp.response

def findAnsIndex(exp_ans,ans):
	for i in range(len(exp_ans)):
		# print "i = ",i
		# print ans.lower()
		# print exp_ans[i].lower()
		if ( exp_ans[i].lower() in ans.lower() ):
			print "ans found at: ",i
			return i
	# not found
	return -1

if __name__ =="__main__":
	main()
