
#! /usr/bin/env python

import socket
import sys

class RobotControlClient:
	# Constructor that initializes Robot's IP and port
	def __init__(self, host, port):
		self.host = host;
		self.port = port;
		
	# Connect to the Robot
	def connect(self):
		try:
			self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		except socket.error, msg:
			print("[ERROR] %s\n" % msg)
			sys.exit(1)

		try:
			self.sock.connect((self.host, self.port))
		except socket.error, msg: 
			print("[ERROR] %s\n" % msg)
			sys.exit(2)
			
	# Disconnect from the Robot
	def disconnect(self):
		self.sock.close()
	
	# Move straight forward
	# @param speed meters / second
	def moveForward(self, speed):
		self.sendData("SetSpeed %f 0" % speed);
		
	# Move staight backward
	# @param speed meters / second
	def moveBackward(self, speed):
		self.sendData("SetSpeed %f 0" % -speed);
		
	# Turn in place couter clockwise
	# @param speed radians / second
	def turnLeft(self, speed):
		self.sendData("SetSpeed 0 %f" % speed);
		
	# Turn in place clockwise
	# @param speed radians / second
	def turnRight(self, speed):
		self.sendData("SetSpeed 0 %f" % -speed);
#		
#	def mv(self,(x,y)):
#		self.pos=(self.pos[0]+x,self.pos[1]+y,self.pos[2])

	# Custom movements
	# @param forwardSpeed in meters / second (negitive is reverse)
	# @yaw rotation in radians / second
	def customMovement(self, forwardSpeed, yaw):
		self.sendData("SetSpeed %f %f" % (forwardSpeed, yaw));
		
	# Stop the Robot
	def stop(self):
		self.sendData("SetSpeed 0 0");
		
	# Private Methods
	def sendData(self, dataString):
		self.sock.send(chr(len(dataString)) + dataString)
