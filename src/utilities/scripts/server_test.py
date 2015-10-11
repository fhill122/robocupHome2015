#!/usr/bin/env python

#Socket client example in python
 
import socket   #for sockets
import sys	#forexit
import thread
import atexit

HOST = ''   # Symbolic name meaning all available interfaces
PORT = 8880 # Arbitrary non-privileged port

#Function for handling connections. This will be used to create threads
def clientthread(conn,addr):
##    conn = client_connection[0]
##    addr = client_connection[1]
    #Sending message to connected client
    conn.send('Welcome to the server.\n') #send only takes string
     
    #infinite loop so that function do not terminate and thread do not end.
    while True:
        #Receiving from client
        data = conn.recv(1024)
        print addr[0]+":",PORT,":"+str(addr[1])+":"+data
        reply = 'OK...' + data
        if not data: 
            break
        conn.sendall(reply)
     
    #came out of loop
    conn.close()

def server_shutdown(s):
    s.close()
    print "exiting...."

def main():
	global PORT
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
	
	while 1:
		#wait to accept a connection - blocking call
		conn, addr = s.accept()
		print 'Connected with ' + addr[0] + ':' + str(addr[1])
		 
		#start new thread takes 1st argument as a function name to be run, second is the tuple of arguments to the function.
		thread.start_new_thread(clientthread ,(conn,addr))

if __name__ =="__main__":
    main()
