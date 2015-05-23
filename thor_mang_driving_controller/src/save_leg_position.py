#!/usr/bin/env python
import roslib

roslib.load_manifest('thor_mang_driving_controller')

import rospy
from rospkg import RosPack
from sensor_msgs.msg import JointState

import yaml

joint_state_topic = '/thor_mang/joint_states'

leg_joints = ['r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll']
used_joints = [leg_joints[2], leg_joints[3], leg_joints[4]]  # hip pitch, knee, ankle pitch
leg_position_types = ['stop_position', 'forward_position', 'safety_position']


class LegPositionSaver():
    def __init__(self):
        rospy.init_node("key_position_saver")
        self.state_subscriber = rospy.Subscriber(joint_state_topic, JointState, self.joint_state_cb)
        self.leg_positions = {}
        self.joint_state = None

    def run(self):
        print '=============    Leg Position Saver    =============='
        print 'This tool is used to save 3 positions for the leg.'
        print 'To exit the tool, type "exit" or "quit".\n'

        exiting = False
        for leg_position_type in leg_position_types:
            cmd = raw_input("Press [Enter] when the robot is in " + leg_position_type + ": ")
            if cmd in ['exit', 'quit'] or rospy.is_shutdown():
                exiting = True
                break
            else:
                self.add_leg_position(leg_position_type, self.get_current_position())
        if not rospy.is_shutdown() and not exiting:
            file_name = raw_input('Enter a name for the yaml-config: ')
            if file_name != '':
                if '.yaml' not in file_name:
                    file_name += '.yaml'
            else:
                file_name = 'speed_configuration.yaml'
            self.save_leg_positions_to_disc(file_name)

    def add_leg_position(self, position_type, joint_positions):
        if joint_positions is not None:
            self.leg_positions[position_type] = joint_positions

    def save_leg_positions_to_disc(self, file_name):
        rp = RosPack()
        save_path = rp.get_path('thor_mang_driving_controller') + '/config/throttle/' + file_name

        yaml_dict = dict()
        yaml_dict.update({'leg_joints': leg_joints})
        yaml_dict.update({'speed_control_joints': used_joints})
        yaml_dict.update(self.leg_positions)
        with open(save_path, 'w+') as outfile:
            outfile.write(yaml.dump(yaml_dict, default_flow_style=False))

    def joint_state_cb(self, joint_state):
        self.joint_state = joint_state

    def get_current_position(self):
        if not self.joint_state:
            rospy.logwarn("No joint state was received yet.")
            return None
        try:
            joint_ids = [self.joint_state.name.index(joint_name) for joint_name in used_joints]
        except ValueError as e:
            print 'Some joint was not found in received joint state:\n%s' % e
            return None
        current_positions = [round(self.joint_state.position[joint_id], 4) for joint_id in joint_ids]
        return current_positions

if __name__ == '__main__':
    saver = LegPositionSaver()
    saver.run()