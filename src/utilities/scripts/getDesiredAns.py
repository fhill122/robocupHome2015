#!/usr/bin/env python
import socket   #for sockets
import sys	#forexit
import thread
import atexit
import threading
import rospy
from utilities.srv import *
import server
from utilities.srv import *

REPEAT_TIMES = 2


def main():
	#initiate ros, robot, assign variables...
	rospy.init_node("getDesiredAns_test_node")
	index = getDesiredAns(["Do you wanna some water?","I repeat, Do you wanna some water? Yes or No","Ok",["yes","no"]])
	print "final ans at ",index
	return

#return the index in exp_ans, will ask first, then repeat REPEAT_TIMES, then
def getDesiredAns(enq):
	[ask,repeat,confirm,exp_ans] =enq

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

#just get response from calling ros service
def getAndroidAns(ask):
	rospy.wait_for_service(server.SERVICE_NAME)
	try:
		srv_h=rospy.ServiceProxy(server.SERVICE_NAME,voiceCommand)
		resp =srv_h(ask)
	except rospy.ServiceException, e:
		print "service dall failed: %s"%e

	print "response from android: "+resp.response
	return resp.response

if __name__ =="__main__":
	main()
