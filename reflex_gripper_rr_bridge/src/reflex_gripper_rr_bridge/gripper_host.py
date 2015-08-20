#!/usr/bin/env python
import yaml
import sys
import argparse
import numpy
from os.path import join

import rospy
import rosnode
import roslib
roslib.load_manifest('reflex_gripper_rr_bridge')

from reflex_gripper.reflex_sf_hand import ReflexSFHand
import reflex_gripper_msgs.msg as MSG
import reflex_gripper_msgs.srv as SRV
from std_srvs.srv import Empty
import RobotRaconteur as RR

gripper_servicedef="""
#Service to provide simple interface to the Right Hand Robotics Gripper Hands
service ReflexGripper_Interface

option version 0.4

object Gripper

property double[] joint_positions
property double[] joint_velocities
property double[] joint_torques

function void setControlMode(uint8 mode)
function void setJointCommand(double[] command)
function void setPositionModeSpeed(double speed)
function void calibrate()

#Functions for position mode presets
#function void closeGrip()
#function void openGrip()
#function void tightenGrip()
#function void loosenGrip()
#function void tightenFinger(string finger)
#function void loosenFinger(string finger)
#function void zeroHand()
#function void setGripShape(string shape)

end object
"""

class Gripper_impl(object):
	def __init__(self, name):
		print "Initializing Node..."
		
		self._name = name
		rospy.init_node('reflex_%s_RRService'%(name))

		print "Connecting to the ROS Service for the Hand..."
		self._default_pub = rospy.Publisher('reflex_%s/command'%(name), MSG.Command, queue_size=10)
		self._pos_pub = rospy.Publisher('/reflex_%s/command_position'%(name), MSG.PoseCommand, queue_size=10)
		self._vel_pub = rospy.Publisher('/reflex_%s/command_velocity'%(name), MSG.VelocityCommand, queue_size=10)
		self._force_pub = rospy.Publisher('/reflex_%s/command_motor_force'%(name), MSG.ForceCommand, queue_size=10)
		
		self._set_speed_service = rospy.ServiceProxy('/reflex_%s/set_speeds'%(name), SRV.SetSpeed)
		self._cal_service = rospy.ServiceProxy('/reflex_%s/zero_fingers'%name, Empty)
		
		rospy.Subscriber('/reflex_%s/hand_state'%(name), MSG.Hand, self._joint_state_cb)

		print "Initializing Hand ..."
		self._joint_positions = [0]*4
		self._joint_velocities = [0]*4
		self._joint_torques = [0]*4
		self._joint_command = [0]*8
		self._position_mode_speed = [4.5]*4

		self._FINGER_CLOSED = 4.6
		self._FINGER_PINCH = 2.5
		self._PRESHAPE_CYLINDER = 0
		self._PRESHAPE_SPHERICAL = 1.5
		self._PRESHAPE_PINCH = 2.5
		self._valid_shape_names = {'cylinder' : 'cylinder',
									'c' : 'cylinder',
									'sphere' : 'sphere',
									's' : 'sphere',
									'pinch' : 'pinch',
									'p' : 'pinch'}

		self._MODE_DEFAULT  = 0
		self._MODE_POSITION = 1
		self._MODE_VELOCITY = 2
		self._MODE_TORQUE   = 3
		self._mode = self._MODE_DEFAULT

		#initial joint command is current position
		self.setJointCommand(self._joint_positions + self._joint_velocities)


	@property
	def joint_positions(self):
	    return self._joint_positions
	
	@property
	def joint_velocities(self):
	    return self._joint_velocities
	
	@property
	def joint_torques(self):
		return self._joint_torques

	def setControlMode(self, mode):
		if mode != self._MODE_DEFAULT and \
				mode != self._MODE_POSITION and \
				mode != self._MODE_VELOCITY and \
				mode != self._MODE_TORQUE:
			return

		self._mode = mode

		if mode == self._MODE_DEFAULT:
			self.setJointCommand(self._joint_positions + self._joint_velocities)
		elif mode == self._MODE_POSITION:
			self._set_speed_service(*self._position_mode_speed)
			self.setJointCommand(self._joint_positions)
		elif mode == self._MODE_VELOCITY: 
			self.setJointCommand([0]*4)
		elif mode == self._MODE_TORQUE:
			self.setJointCommand([0]*4)

		

	def setJointCommand(self, command):
		prev_command = self._joint_command
		self._joint_command = command
		if (self._mode == self._MODE_DEFAULT):
				if len(command)!=8:
					raise RuntimeError("Current control mode expects 8 inputs")
					self._joint_command = old_command
					return
				
				msg = MSG.Command()
				msg.pose = MSG.PoseCommand(*self._joint_command[0:4])
				msg.velocity = MSG.VelocityCommand(*self._joint_command[4:8])
				self._default_pub.publish(msg)
		else:
			if len(command) != 4:
				raise RuntimeError("Current control mode expects 4 inputs")
				self._joint_command = prev_command
				return
			
			if (self._mode == self._MODE_POSITION):
				msg = MSG.PoseCommand(*self._joint_command)
				self._pos_pub.publish(msg)
			elif (self._mode == self._MODE_VELOCITY):
				msg = MSG.VelocityCommand(*self._joint_command)
				self._vel_pub.publish(msg)
			elif (self._mode == self._MODE_TORQUE):
				msg = MSG.ForceCommand(*self._joint_command)
				self._force_pub.publish(msg)

	def setPositionModeSpeed(self, speed):
		if speed < 0.0:
			speed = 0.0
		self._position_mode_speed=[speed]*4


	# def closeGrip(self):
	# 	if self._joint_command[3] == self._PRESHAPE_CYLINDER:
	# 		self._joint_command[0:3] = [self._FINGER_CLOSED]*3
		
	# 	elif self._joint_command[3] == self._PRESHAPE_SPHERICAL:
	# 		self._joint_command[0:3] = [self._FINGER_PINCH]*3
		
	# 	elif self._joint_command[3] == self._PRESHAPE_PINCH:
	# 		self._joint_command[0:2] = [self._FINGER_PINCH]*2
		
	# 	self.pub.publish(*self._joint_command)
	# 	return

	# def openGrip(self):
	# 	self._joint_command[0:3] = [0]*3
	# 	self.pub.publish(*self._joint_command)
	# 	return

	# def tightenGrip(self):
	# 	for motor in self.gripper.motors:
	# 		if 'preshape' in motor:
	# 			continue
	# 		self.gripper.motors[motor].tighten()
	# 	return

	# def loosenGrip(self):
	# 	for motor in self.gripper.motors:
	# 		if 'preshape' in motor:
	# 			continue
	# 		self.gripper.motors[motor].loosen()
	# 	return

	# def tightenFinger(self, finger):
	# 	if finger.lower() not in self.gripper.fingers:
	# 		raise RuntimeError("'%s' is not a valid finger"%finger)
	# 		return
	# 	else:
	# 		fingerMotor = '/gripper_%s_%s'%(self.gripper.name, finger.lower())
	# 		self.gripper.motors[fingerMotor].tighten()
	# 		return

	# def loosenFinger(self, finger):
	# 	if finger.lower() not in self.gripper.fingers:
	# 		raise RuntimeError("'%s' is not a valid finger"%finger)
	# 		return
	# 	else:
	# 		fingerMotor = '/gripper_%s_%s'%(self.gripper.name, finger.lower())
	# 		self.gripper.motors[fingerMotor].loosen()


	# def setGripShape(self, shape):
	# 	shape = shape.lower()
	# 	if not shape in self._valid_shape_names.keys():
	# 		return

	# 	if self._valid_shape_names[shape] == 'cylinder':
	# 		self._joint_command[3] = self._PRESHAPE_CYLINDER
	# 	elif self._valid_shape_names[shape] == 'sphere':
	# 		self._joint_command[3] = self._PRESHAPE_SPHERICAL
	# 	elif self._valid_shape_names[shape] == 'pinch':
	# 		self._joint_command[3] = self._PRESHAPE_PINCH

	# 	self.pub.publish(*self._joint_command)
	# 	return

	def zeroHand(self):
		self.setControlMode(self._MODE_POSITION)
		self.setJointCommand([0]*4)
		return

	def calibrate(self):
		self.zeroHand()
		self._cal_service()

	#---------------------------
	#--- Private Functions -----
	#---------------------------
	def close(self):
		self.zeroHand()

	def readJointPositions(self,data):
		for i in range(0,4):
			self._joint_positions[i] = data.motor[i].joint_angle

	def readJointVelocities(self,data):
		for i in range(0,4):
			self._joint_velocities[i] = data.motor[i].velocity

	def readJointTorques(self,data):
		for i in range(0,4):
			self._joint_torques[i] = data.motor[i].load

	def _joint_state_cb(self,data):
		self.readJointPositions(data)
		self.readJointVelocities(data)
		self.readJointTorques(data)


def main(argv):
	print "REFLEX GRIPPER RR BRIDGE"
	print "========================"
	# Parse command line arguments
	parser = argparse.ArgumentParser(
		description='Initialize a Gripper Hand')
	parser.add_argument('--port', type=int, default=0,
		help='TCP port to host service on' +\
		'(will auto-generate if not specified)')
	parser.add_argument('name', help='The desired name for the gripper.' +\
		'Must match name given to start gripper services(e.g. left)')
	
	args = parser.parse_args(argv)
	# Check for valid name
	name = args.name
	if not "/reflex_%s"%(name) in rosnode.get_node_names():
		print "The name '%s' is not valid. The gripper services may not "%(name) +\
		"be started yet."
		raw_input("Press ENTER to close")
		return -1

	# Enable numpy
	RR.RobotRaconteurNode.s.UseNumPy = True

	# Set the node name
	RR.RobotRaconteurNode.s.NodeName = "ReflexGripperServer.%s"%name

	# Initialize the object
	gripper_obj = Gripper_impl(name)

	# Create transport, register it, and start the server
	print "Registering Transport..."
	t = RR.TcpTransport()
	t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL |
						 RR.IPNodeDiscoveryFlags_LINK_LOCAL |
						 RR.IPNodeDiscoveryFlags_SITE_LOCAL)

	RR.RobotRaconteurNode.s.RegisterTransport(t)

	t.StartServer(args.port)
	port = args.port
	if (port == 0):
		port = t.GetListenPort()

	# Register the service type and the service
	print "Starting Service..."
	RR.RobotRaconteurNode.s.RegisterServiceType(gripper_servicedef)
	RR.RobotRaconteurNode.s.RegisterService("Gripper", 
						  "ReflexGripper_Interface.Gripper", 
						  				  gripper_obj)

	print "Service started, connect via"
	print "tcp://localhost:%s/ReflexGripperServer.%s/Gripper"%(port, name)
	
	raw_input("press ENTER to quit ... \r\n")

	gripper_obj.close()

	# This must be here to prevent segfault
	RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
	main(sys.argv[1:])
