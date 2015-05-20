#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace thor_mang_driving_controller {
  
class DrivingController {
public:
  enum AxisIDs {
    FORWARD=1,
    STEERING=3
  };
  
  enum ButtonIDs {
    STOP=1
  };
  
  DrivingController();
  ~DrivingController();
  
  void handleJoyPadEvent(sensor_msgs::JoyConstPtr msg);
  void setSteeringInverted(bool inverted);
  
private:
  // load preset points
  void initPoseVector();

  void sendNewSteeringTarget(int offset);

  // ROS node handle
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  // Subscriber for joypad commands
  ros::Subscriber joypad_sub_;

  // Publisher for controller commands
  ros::Publisher steering_control_cmd_pub_;
  
  // invert steering input (to achieve -1 = left, +1 = right)
  bool steering_inverted_;

  // joint names used for the target poses
  std::vector<std::string> steering_joint_names_;

  // target joint positions
  std::vector< std::vector<double> > steering_target_positions_;

  // index of the current target position
  int current_steering_position_idx_;

  // topic for accessing the controllers
  std::string steering_controller_topic_;
};


}
