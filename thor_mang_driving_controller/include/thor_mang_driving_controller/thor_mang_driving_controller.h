#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace thor_mang_driving_controller {
  
class DrivingController {
public:
  enum AxisIDs {
    STEERING=0,
    SPEED=3
  };
  
  enum ButtonIDs {
    E_STOP=2,
    STEERING_SENSITIVITY_PLUS=7,
    STEERING_SENSITIVITY_MINUS=6,
    HEAD_LEFT=4,
    HEAD_RIGHT=5
  };
  
  DrivingController();
  ~DrivingController();
  
  void handleJoyPadEvent(sensor_msgs::JoyConstPtr msg);

  void setSteeringInverted(bool inverted);
  void updateSteering();
  
private:
  // load preset points
  void initKeyFrames();

  std::vector<double> getInterpolatedKeyFrame(double value, double max_value);
  double getPreviousValue(double value);
  double getNextValue(double value);

  void eStop();
  void moveHead(int value);
  void changeSteeringSensitivity(double diff);
  void handleSteeringCommand(double value);
  void handleSpeedCommand(double value);

  // ROS node handle
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  // Subscriber for joypad commands
  ros::Subscriber joypad_sub_;

  // Publisher for controller commands
  ros::Publisher steering_control_cmd_pub_;
  ros::Publisher speed_control_cmd_pub_;
  
  // invert steering input (to achieve -1 = left, +1 = right)
  bool steering_inverted_;

  // current steering speed (°/t)
  double steering_speed_;

  // joint names used for the target poses
  std::vector<std::string> steering_joint_names_;
  std::vector<std::string> speed_joint_names_;

  // target joint positions
  std::map< double, std::vector<double> > steering_key_frames_;
  std::map< double, std::vector<double> > speed_key_frames_;

  // current steering angle
  double current_steering_angle_;

  // topic for accessing the controllers
  std::string steering_controller_topic_;
  std::string speed_controller_topic_;

  // sensitivity of the steering commands
  double steering_sensitivity_;
  const double steering_sensitivity_step = 0.5;
};


}
