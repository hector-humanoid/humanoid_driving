//=================================================================================================
// Copyright (c) 2015, Achim Stein, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <humanoid_driving_controller/DrivingCommand.h>
#include <humanoid_driving_controller/DrivingState.h>

namespace humanoid_driving_controller {
  
class DrivingController {
public:  
  DrivingController();
  ~DrivingController();

  // ensure connection to driving widget is stable, otherwise go to all-hold
  void checkReceivedMessages();

  // move steering wheel towards target position & update driving counter
  void updateSteering();

  // move head towards target position (if there are head joints specified)
  void updateHeadPosition();
  
  // receive driving command message from driving widget
  void handleDrivingCommand(humanoid_driving_controller::DrivingCommandConstPtr msg);

  // receive new set of robot joints
  void handleNewJointStateEvent(sensor_msgs::JointStateConstPtr msg);  

  // enable / disable controller
  void handleControllerEnable(std_msgs::BoolConstPtr msg);

private:
  // update leg position drive / stop
  void updateDriveForward(bool drive);

  // go to safety pose (stop steering, leg to safety)
  void allHold();

  // load preset points
  void initKeyFrames();

  // get interpolated key frames (max_value = maximum possible angle (default = 360.0)
  std::vector<double> getInterpolatedKeyFrame(double value, double max_value);

  // get robot joint positions for the given names, replace some
  std::vector<double> getRobotJointPositions(std::vector<std::string> &joint_names, std::vector<std::string> replace_joint_names = std::vector<std::string>(), std::vector<double> replace_joint_angles = std::vector<double>());

  // generate a trajectory messages from joint names and angles
  trajectory_msgs::JointTrajectory generateTrajectoryMsg(std::vector<double> &joint_angles, std::vector<std::string> joint_names);

  // get the next smaller / next larger key frame key for value
  double getPreviousSteeringKey(double key);
  double getNextSteeringKey(double key);

  // convert rad angles to degree angles
  inline double rad2deg(double rad);

  // ROS node handle
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  // Subscribers
  ros::Subscriber joint_state_sub_;         // receive robot joint states
  ros::Subscriber driving_command_sub_;     // receive driving commands from widget
  ros::Subscriber controller_enable_sub_;   // enable / disable controller as requested

  // Publisher for controller commands
  ros::Publisher steering_control_cmd_pub_; // send steering commands to arm controller
  ros::Publisher speed_control_cmd_pub_;    // send throttle commands to leg controller
  ros::Publisher head_cmd_pub_;             // send head movement commands to head controller
  ros::Publisher all_hold_enabled_pub_;     // inform widget that all-hold was enabled (due to connection loss)
  ros::Publisher controller_enable_ack_pub_;// acknowledge controller state change
  ros::Publisher driving_state_pub_;        // inform widget of current driving state

  // steering command stuff
  humanoid_driving_controller::DrivingCommand driving_command_; // last command issued by widget
  ros::Time driving_command_time_;                              // time that last command was received

  humanoid_driving_controller::DrivingState current_state_; // current state of the robot for driving

  // joint names used for the target poses
  std::vector<std::string> steering_joint_names_;      // joint names of the arm moving the steering wheel
  std::vector<std::string> leg_joint_names_;           // joint names of the leg pressing the throttle
  std::vector<std::string> speed_control_joint_names_; // joint names of the leg joints that are actually moved for speed control

  // target joint positions
  std::map< double, std::vector<double> > steering_key_frames_; // mapping angles [deg] to vector positions for the key frames
  std::vector<double> drive_forward_position_;                  // leg joint position for driving
  std::vector<double> stop_position_;                           // leg joint position for stopping
  std::vector<double> safety_position_;                         // leg joint position for all-hold

  // current joint states of the robot
  std::vector<std::string> robot_joint_names_; // vector of all robot joint names
  std::vector<double> robot_joint_positions_;  // vector of all robot joint positions

  // head joints configuration
  std::string head_pan_joint_name_;  // joint name for head pan joint (or empty)
  std::string head_tilt_joint_name_; // joint name for head tilt joint (or empty)

  // time when the last command was received (for alive-messages)
  ros::Time last_auto_stop_info_sent_time_;

  // is the controller currently active?
  bool controller_enabled_;

  // constants
  const double MaxSteeringAngleOffset = 9.0e-4;          // maximum allowed offset between target steering angle and current steering angle (=0.05°)
  const double SteeringAngleDifferenceSpeedFactor = 0.8; // scale the steering wheel angle difference
  const double HeadAngleDifferenceSpeedFactor = 0.4;     // scale the head angle difference
  const double MovementTargetTime = 0.1;                 // time sent to the controllers as the time to reach the target position
  const double HeadSensitivity = 2.5;                    // maximum speed the robot may turn its head (in rad / time step)
  const double SteeringSensitivity = 0.08;               // maximum speed the robot may turn the steering wheel (in rad / time step)
};


}
