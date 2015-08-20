#!/usr/bin/env python

import rospy
import rosnode
import rospkg
import subprocess
import time
import os
import argparse
import sys

progName = os.path.basename(__file__)

class RRAction(argparse.Action):
	"""docstring for RRAction"""
	def __call__(self, parser, namespace, values, option_string=None):
		setattr(namespace, self.dest, True)
		setattr(namespace, 'RRPort', values)
def main(argv):
	
	# Parse the command line arguments 
	parser = argparse.ArgumentParser(
		description='Launch the ROS components of a ReflexSF Gripper Hand')
	
	hand_group = parser.add_argument_group('Hand Arguments', 
		'Arguments specific to launching a hand')
	hand_group.add_argument('name', nargs='+',
		help="A list of names for the grippers.")
	hand_group.add_argument('--port', nargs='+', 
		help='A list of the USB ports the hands are attached to. ' +\
		'Specified in the same order as the names. ' +\
		'(defaults to \'/dev/ttyUSB<#>\' starting at 0)')
	
	script_group = parser.add_argument_group('Script Options')
	
	script_group_hand = script_group.add_mutually_exclusive_group()
	script_group_hand.add_argument('--bkgOnly', action='store_true', 
		help='Set up all the required background nodes but ' +\
		'not the main gripper node')
	script_group_hand.add_argument('--RRService', nargs='?', 
		default=0, action=RRAction,
		help='Start the main gripper node as a Robot Raconteur Service.' +\
		'The optional argument is the RR Port number. ' +\
		'Note: you must have the reflex_gripper_rr_bridge package installed.')
	
	script_group_cal = script_group.add_mutually_exclusive_group()
	script_group_cal.add_argument('--calibrate', action='store_true',
		help='Run the default calibration script as part of the '+\
		'launcher script')
	script_group_cal.add_argument('--calibrateI', action='store_true',
		help='Calibrate the hand as part of the setup script '+\
		'using the interactive controller.')
	script_group_cal.add_argument('--nocalibrate', action='store_true',
		help='Do not calibrate as part of the launcher script. '+\
		'If a gripper with the given name has not been started ' +\
		'before the calibration script will be run by default.' )
	
	
	args = parser.parse_args(argv)

	calibrate = args.calibrate
	calibrateI = args.calibrateI
	nocalibrate = args.nocalibrate

	bkgOnly = args.bkgOnly
	RRService = args.RRService
	RRPort = args.RRPort

	names = args.name
	ports = args.port
	if ports == None:
		# Default ports to '/dev/ttyUSB<#>''
		ports = ['/dev/ttyUSB%s'%i for i in range(0,len(names))]
	elif len(ports) != len(names):
		print '%s: error: ports incorrectly listed. ' %(progName) +\
		'If --ports is specified a port must be listed for each ' +\
		'gripper specified' 
		return -1

	# Make sure a Master is already running
	print "Checking if Master Node is running..."
	try:
		rospy.has_param("/rosout")
	except Exception, e:
		print e
		print "No Master Found.\nStarting Master..."
		print "(Process will open in a new window)"
		subprocess.call(['gnome-terminal', '-e', "bash -c \"roscore\""])
		time.sleep(2)

	print "--------------------------------------------"

	# check if a profile for this name exists yet
	print "Checking for an existing configuration..."
	rospack = rospkg.RosPack()
	pkgPath = rospack.get_path('reflex_gripper')
	yamlPath = 'yaml'
	for name in names:
		filename = 'reflex_%s_zero_points.yaml' % name
		filePath = os.path.join(pkgPath, yamlPath, filename)    
		# if the name has not been loaded before. 
		# create a default zero points file for it.
		# calibration script will need to be called.
		if not os.path.isfile(filePath):
			print "No configuration found."
			print "Creating a configuration for \'%s\'..."%name
			import yaml
			nocalibrate = False
			calibrate = True

			keys = [('reflex_'+name+'_f1'),
				('reflex_'+name+'_f2'),
				('reflex_'+name+'_f3'),
				('reflex_'+name+'_preshape')]
			zero_points_yaml = {key : dict(zero_point = 0.0) for key in keys}
			with open(filePath, 'w') as outfile:
				outfile.write(yaml.dump(zero_points_yaml))
	
	print "--------------------------------------------"
	# Start the whole hand or just the fingers?
	# If only background (finger) nodes should be started
	if bkgOnly:
		print "Starting the services for the fingers..."
		print "(Each process will open a new window)"
		full_command=[]
		for i in range(0,len(names)):
			full_command += ['--tab', '-e', 
				"bash -c \"roslaunch reflex_gripper motors.launch " +\
				"Name:=%s Port:=%s\"" %(names[i],ports[i])]
		
		subprocess.call(['gnome-terminal'] + full_command)

	else:
		print "Starting the services for each hand..."
		print "(Each process will open a new window)"
		full_command=[]
		for i in range(0,len(names)):
			full_command += ['--tab', '-e', 
				"bash -c \"roslaunch reflex_gripper reflex_gripper.launch " +\
				"Name:=%s Port:=%s\"" %(names[i],ports[i])]
		subprocess.call(['gnome-terminal'] + full_command)

	print "--------------------------------------------"

	# Make sure the required nodes have successfully launched
	print "Checking for successful launch of all required nodes..."
	time.sleep(4)
	found = False
	for name in names:
		# check that finger was successfully started
		print "\tChecking %s..." % name
		found = "/dynamixel_manager_%s" % name in rosnode.get_node_names() 
		if not bkgOnly:
			found = found and "/reflex_%s" % name in rosnode.get_node_names()
		if(not found):
			print "%s: error: It appears that the required " %(progName) +\
			"nodes were not successfully started. Please try again."
			return -1       
	print "--------------------------------------------"

	# if RRService specified try starting the RR Service
	if RRService:
		try:
			RRpkg = rospack.get_path('reflex_gripper_rr_bridge')
			print "Starting RR Services for each hand..."
			print "(Each process will open a new window)"
			full_command = []
			for name in names:
				full_command += ['--tab', '-e', 'bash -c \"rosrun reflex_gripper_rr_bridge gripper_host.py %s --port %s\"'%(name,RRPort)]
			subprocess.call(['gnome-terminal'] + full_command)
		except rospkg.ResourceNotFound, e:
			print "Could not find the 'reflex_gripper_rr_bridge' package",
			"so the RR services will not be started."

		print "--------------------------------------------"
	
	# Calibrate the fingers
	if not nocalibrate:
		if not calibrate and not calibrateI:
			ans = raw_input("Would you like to calibrate the hands? (y/n)\t")
			if ans == 'y':
				ans2 = raw_input("Which calibration script would you like to use?"+\
					"\n\t(1)Default\n\t(2)Interactive" +\
					"\nEnter Number: ")
				if ans2 == '1':
					calibrate = True
					calibrateI = False
				elif ans2 == '2':
					calibrate=False
					calibrateI=True
				else:
					print "That was not a valid option. Default will be used."
					calibrate = True
					calibrateI = False
			elif ans == 'n':
				calibrate = False
				calibrateI = False
				print "Skipping calibration..."
			else:
				print "That was not a valid option."
				print "Skipping calibration..."
			print "--------------------------------------------"
		
		if calibrate or calibrateI:
			print "Starting Calibration script..."
			for name in names:
				if calibrate:
					subprocess.call('rosrun reflex_gripper calibrate.py ' + name, shell=True)
				elif calibrateI:
					time.sleep(1)
					subprocess.call('rosrun reflex_gripper calibrate_interactive.py ' + name, shell=True)               
			
			print "Calibration Finished"
			print "--------------------------------------------"

	print "Setup for all hands is complete.\nExiting the launch script"
	
if __name__=="__main__":
	main(sys.argv[1:])