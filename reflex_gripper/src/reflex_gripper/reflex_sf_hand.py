#!/usr/bin/env python

#############################################################################
# Code Modified by Gregory Grebe for use with the CATS Robotics Lab
# http://github.com/rpiRobotics/reflex-gripper-ros-pkg
# Aug 17, 2015
#
# Original License: 
#
# Copyright 2015 Right Hand Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#############################################################################

from os.path import join
import yaml

from dynamixel_msgs.msg import JointState
import rospkg
import rospy
from std_srvs.srv import Empty

from reflex_hand import ReflexHand
from reflex_sf_motor import ReflexSFMotor
import reflex_gripper_msgs.msg as MSG

class ReflexSFHand(ReflexHand):
    def __init__(self,name):
        self.name = name
        super(ReflexSFHand, self).__init__(self.name, ReflexSFMotor)
        self.hand_state_pub = rospy.Publisher(self.namespace + '/hand_state',
                                              MSG.Hand, queue_size=10)
        rospy.Service(self.namespace + '/zero_fingers', Empty, self.calibrate)

    def _receive_cmd_cb(self, data):
        self.disable_force_control()
        self.set_speeds(data.velocity)
        self.set_angles(data.pose)

    def _receive_angle_cmd_cb(self, data):
        self.disable_force_control()
        self.reset_speeds()
        self.set_angles(data)

    def _receive_vel_cmd_cb(self, data):
        self.disable_force_control()
        self.set_velocities(data)

    def _receive_force_cmd_cb(self, data):
        self.disable_force_control()
        self.reset_speeds()
        self.set_force_cmds(data)
        self.enable_force_control()

    def disable_torque(self):
        for ID, motor in self.motors.items():
            motor.disable_torque()

    def enable_torque(self):
        for ID, motor in self.motors.items():
            motor.enable_torque()

    def _publish_hand_state(self):
        state = MSG.Hand()
        motor_names = ('_f1', '_f2', '_f3', '_preshape')
        for i in range(4):
            state.motor[i] = self.motors[self.namespace + motor_names[i]].get_motor_msg()
        self.hand_state_pub.publish(state)

    def calibrate(self, data=None):
        for motor in sorted(self.motors):
            rospy.loginfo("Calibrating motor " + motor)
            command = raw_input("Type 't' to tighten motor, 'l' to loosen \
motor, or 'q' to indicate that the zero point has been reached\n")
            while not command.lower() == 'q':
                if command.lower() == 't' or command.lower() == 'tt':
                    print "Tightening motor " + motor
                    self.motors[motor].tighten(0.35 * len(command) - 0.3)
                elif command.lower() == 'l' or command.lower() == 'll':
                    print "Loosening motor " + motor
                    self.motors[motor].loosen(0.35 * len(command) - 0.3)
                else:
                    print "Didn't recognize that command, use 't', 'l', or 'q'"
                command = raw_input("Tighten: 't'\tLoosen: 'l'\tDone: 'q'\n")
            rospy.loginfo("Saving current position for %s as the zero point", motor)
            self.motors[motor]._set_local_motor_zero_point()
        print "Calibration complete, writing data to file"
        self._zero_current_pose()
        return []

    def _write_zero_point_data_to_file(self, filename, data):
        rospack = rospkg.RosPack()
        reflex_sf_path = rospack.get_path("reflex")
        yaml_path = "yaml"
        file_path = join(reflex_sf_path, yaml_path, filename)
        with open(file_path, "w") as outfile:
            outfile.write(yaml.dump(data))

    def _zero_current_pose(self):
        keys = [(self.namespace+ '_' + finger) for finger in self.fingerNames]
        data = { key : dict(
            zero_point = self.motors[key].get_current_raw_motor_angle() ) 
            for key in keys}
        self._write_zero_point_data_to_file(self.namespace + '_zero_points.yaml', data)


def main():
    rospy.sleep(2.0)  # To allow services and parameters to load
    hand = ReflexSFHand('hand')
    rospy.on_shutdown(hand.disable_torque)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        hand._publish_hand_state()
        r.sleep()


if __name__ == '__main__':
    main()
