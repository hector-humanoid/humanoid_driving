#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <thor_mang_driving_controller/DrivingCommand.h>

namespace thor_mang_driving_controller {
  
class DrivingController {
public:

  
  DrivingController();
  ~DrivingController();

  void checkReceivedMessages();
  
  void handleDrivingCommand(thor_mang_driving_controller::DrivingCommandConstPtr msg);
  void handleNewJointStateEvent(sensor_msgs::JointStateConstPtr msg);
  void handleNewTimeFromStart(std_msgs::Float64ConstPtr msg);
  void handleControllerEnable(std_msgs::BoolConstPtr msg);

  void updateSteering(double target_angle);
  void updateDriveForward(bool drive);

  void allStop();

private:
  // load preset points
  void initKeyFrames();
  std::vector<double> getRobotJointPositions(std::vector<std::string> &joint_names, std::vector<std::string> replace_joint_names = std::vector<std::string>(), std::vector<double> replace_joint_angles = std::vector<double>());
  trajectory_msgs::JointTrajectory generateTrajectoryMsg(std::vector<double> &joint_angles, std::vector<std::string> joint_names);

  std::vector<double> getInterpolatedKeyFrame(double value, double max_value);
  double getPreviousValue(double value);
  double getNextValue(double value);

  // ROS node handle
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  // Subscribers
  ros::Subscriber joint_state_sub_;
  ros::Subscriber driving_command_sub_;
  ros::Subscriber controller_enable_sub_;

  // Publisher for controller commands
  ros::Publisher steering_control_cmd_pub_;
  ros::Publisher speed_control_cmd_pub_;
  ros::Publisher all_stop_enabled_pub_;
  ros::Publisher controller_enable_ack_pub_;

  // steering command stuff
  thor_mang_driving_controller::DrivingCommand last_command_received_;
  bool received_first_command_msg_;
  double time_from_start_;

  // joint names used for the target poses
  std::vector<std::string> steering_joint_names_;
  std::vector<std::string> leg_joint_names_;
  std::vector<std::string> speed_control_joint_names_;

  // target joint positions
  std::map< double, std::vector<double> > steering_key_frames_;
  std::vector<double> drive_forward_position_;
  std::vector<double> stop_position_;
  std::vector<double> safety_position_;

  // current joint states of the robot
  std::vector<std::string> robot_joint_names_;
  std::vector<double> robot_joint_positions_;

  // already received one set of robot positions
  bool received_robot_positions_;

  // time when the last command was received (for alive-messages)
  ros::Time last_command_received_time_;
  ros::Time last_auto_stop_info_sent_time_;

  bool controller_enabled_;
};


}
