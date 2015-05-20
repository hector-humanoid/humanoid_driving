#!/usr/bin/env python
import roslib

roslib.load_manifest('thor_mang_driving_controller')

import rospy
from rospkg import RosPack
from sensor_msgs.msg import JointState

import yaml

joint_state_topic = '/thor_mang/joint_states'

joints = ['l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow', 'l_wrist_yaw1', 'l_wrist_roll',
          'l_wrist_yaw2']


class KeyPosition():
    def __init__(self, angle, joint_positions):
        self.angle = angle
        if len(joint_positions) == len(joints):
            self.joint_positions = joint_positions
        else:
            rospy.logerr('Tried to save KeyPos with mismatching joint number.'
                         ' Got ' + len(joint_positions) + ', expected ' + len(joints))
            self.joint_positions = [0] * len(joints)


class KeyPositionSaver():
    def __init__(self):
        rospy.init_node("key_position_saver")
        self.state_subscriber = rospy.Subscriber(joint_state_topic, JointState, self.joint_state_cb)
        self.key_positions = {}
        self.joint_state = None

    def run(self):
        print '=============    Key Position Saver    =============='
        print 'This tool is used to create key positions for steering.'
        print 'Every time you enter an angle and confirm with [Enter]\n' \
              'a new key position is created and saved in a yaml file.'
        print 'To exit the tool, type "exit" or "quit".\n'

        exiting = False
        while not exiting:
            cmd = raw_input("Enter a new angle: ")
            if cmd in ['exit', 'quit'] or rospy.is_shutdown():
                exiting = True
            else:
                try:
                    angle = float(cmd)
                except ValueError as e:
                    rospy.logerr('Invalid command: ' + cmd)
                else:
                    self.add_key_position(angle, self.get_current_position())
        if not rospy.is_shutdown():
            file_name = raw_input('Enter a name for the yaml-config: ')
            if file_name != '':
                if '.yaml' not in file_name:
                    file_name += '.yaml'
            else:
                file_name = 'key_pos.yaml'
            self.save_key_positions_to_disc(file_name)

    def add_key_position(self, angle, joint_positions):
        if joint_positions is not None:
            key = str(angle).split('.')[0]
            self.key_positions[key] = joint_positions

    def save_key_positions_to_disc(self, file_name):
        rp = RosPack()
        save_path = rp.get_path('thor_mang_driving_controller') + '/config/'

        yaml_dict = dict()
        yaml_dict.update({'joints': joints})
        yaml_dict.update({'angles': list(self.key_positions.iterkeys())})
        yaml_dict.update(self.key_positions)
        with open(save_path + file_name, 'w+') as outfile:
            outfile.write(yaml.dump(yaml_dict, default_flow_style=False))

    def joint_state_cb(self, joint_state):
        self.joint_state = joint_state

    def get_current_position(self):
        if not self.joint_state:
            rospy.logwarn("No joint state was received yet.")
            return None
        try:
            joint_ids = [self.joint_state.name.index(joint_name) for joint_name in joints]
        except ValueError as e:
            print 'Some joint was not found in received joint state:\n%s' % e
            return None
        current_positions = [round(self.joint_state.position[joint_id], 4) for joint_id in joint_ids]
        return current_positions

if __name__ == '__main__':
    saver = KeyPositionSaver()
    saver.run()