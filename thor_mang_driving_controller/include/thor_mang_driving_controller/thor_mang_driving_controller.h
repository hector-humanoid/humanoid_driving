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
    E_STOP=2,
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
  trajectory_msgs::JointTrajectory generateTrajectoryMsg(std::vector<double> &joint_angles, std::vector<std::string> joint_names);

  std::vector<double> getInterpolatedKeyFrame(double value, double max_value);
  double getPreviousValue(double value);
  double getNextValue(double value);

  void forwardDrive(bool drive);
  void eStop();
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
  
  // invert steering input (to achieve -1 = left, +1 = right)
  bool steering_inverted_;

  // current steering speed (°/t)
  double steering_speed_;

  // joint names used for the target poses
  std::vector<std::string> steering_joint_names_;
  std::vector<std::string> leg_joint_names_;

  // target joint positions
  std::map< double, std::vector<double> > steering_key_frames_;
  std::vector<double> drive_forward_frame_;
  std::vector<double> stop_frame_;
  std::vector<double> e_stop_frame_;

  // current steering angle
  double current_steering_angle_;
  double current_absolute_angle_;

  // topic for accessing the controllers
  std::string steering_controller_topic_;
  std::string speed_controller_topic_;
  std::string joint_state_topic_;

  // sensitivity of the steering commands
  double steering_sensitivity_;
  const double steering_sensitivity_step = 0.5;

  // activate e-stop mode
  bool e_stop_active_;

  // current joint states
  std::vector<std::string> current_joint_names_;
  std::vector<double> current_joint_positions_;
};


}
