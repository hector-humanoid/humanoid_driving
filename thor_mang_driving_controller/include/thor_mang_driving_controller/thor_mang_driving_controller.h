#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace thor_mang_driving_controller {
  
class DrivingController {
public:
  enum AxisIDs {
    STEERING=0,
  };
  
  enum ButtonIDs {
    FORWARD=1,
    ALL_STOP=2,
    STEERING_SENSITIVITY_PLUS=7,
    STEERING_SENSITIVITY_MINUS=6,
    HEAD_LEFT=4,
    HEAD_RIGHT=5
  };
  
  DrivingController();
  ~DrivingController();
  
  void handleJoyPadEvent(sensor_msgs::JoyConstPtr msg);
  void handleNewJointStateEvent(sensor_msgs::JointStateConstPtr msg);

  void setSteeringInverted(bool inverted);
  void updateSteering();
  
private:
  // load preset points
  void initKeyFrames();
  std::vector<double> getRobotJointPositions(std::vector<std::string> &joint_names, std::string replace_joint_name = "", double replace_joint_angle = 0.0);
  trajectory_msgs::JointTrajectory generateTrajectoryMsg(std::vector<double> &joint_angles, std::vector<std::string> joint_names);

  std::vector<double> getInterpolatedKeyFrame(double value, double max_value);
  double getPreviousValue(double value);
  double getNextValue(double value);

  void forwardDrive(bool drive);
  void allStop();
  void moveHead(int value);
  void changeSteeringSensitivity(double diff);
  void handleSteeringCommand(double value);

  // ROS node handle
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  // Subscriber for joypad commands
  ros::Subscriber joypad_sub_;
  ros::Subscriber joint_state_sub_;

  // Publisher for controller commands
  ros::Publisher steering_control_cmd_pub_;
  ros::Publisher speed_control_cmd_pub_;

  // topic for accessing the controllers
  std::string steering_controller_topic_;
  std::string speed_controller_topic_;
  std::string joint_state_topic_;
  
  // invert steering input (to achieve -1 = left, +1 = right)
  bool steering_inverted_;

  // current steering speed (°/t)
  double steering_speed_;

  // joint names used for the target poses
  std::vector<std::string> steering_joint_names_;
  std::vector<std::string> leg_joint_names_;
  std::string speed_control_joint_name_;

  // target joint positions
  std::map< double, std::vector<double> > steering_key_frames_;
  double drive_forward_angle_;
  double stop_angle_;
  double all_stop_angle_;
  //std::vector<double> e_stop_frame_;

  // current steering angle
  double current_steering_angle_;
  double current_absolute_angle_;

  // sensitivity of the steering commands
  double steering_sensitivity_;
  const double steering_sensitivity_step = 0.5;

  // activate e-stop mode
  bool all_stop_active_;

  // current joint states of the robot
  std::vector<std::string> robot_joint_names_;
  std::vector<double> robot_joint_positions_;
};


}
