#!/usr/bin/env python

from reflex_gripper.reflex_sf_hand import ReflexSFHand
from std_srvs.srv import Empty
import argparse, sys, rosnode, rospy

if __name__ == "__main__":
	parser = argparse.ArgumentParser(
		description="Calibrate a Gripper Hand")
	parser.add_argument('name', help="The name of the gripper hand to calibrate")

	args = parser.parse_args(sys.argv[1:])

	name = args.name
	
	if '/reflex_%s'%name in rosnode.get_node_names():
		print "If you do not see the script running in this window, "
				print "please check the window that the hand service is running in."
		rospy.wait_for_service('reflex_%s/zero_fingers'%name, 5)
		service = rospy.ServiceProxy('/reflex_%s/zero_fingers'%name, Empty)
		service()
	else:
		hand = ReflexSFHand(name)
		hand.calibrate()