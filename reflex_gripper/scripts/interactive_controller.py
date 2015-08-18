#!/usr/bin/env python
from reflex_gripper import ReflexSFHand, curses_interface
import curses
import sys
import argparse

def main():
	parser = argparse.ArgumentParser(
		description="Start the interactive controller")
	group = parser.add_mutually_exclusive_group(required=True)
	group.add_argument('--name', help="The name of the gripper hand to start")
	group.add_argument('--topic', help="The existing rostopic to publish to")

	args = parser.parse_args(sys.argv[1:])

	if args.name:
		name = args.name
		hand = ReflexSFHand(name)
		curses.wrapper(curses_interface.curses_fingers_interface, hand)
	elif args.topic:
		print "Sorry this option is not fully supported yet"
		return

if __name__ == "__main__":
	main()