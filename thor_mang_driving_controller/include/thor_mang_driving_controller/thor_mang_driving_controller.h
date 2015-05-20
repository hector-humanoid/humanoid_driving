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
  ros::NodeHandle node_handle_;
  
  //ros::ServiceServer driving_calibration_service_;
  ros::Subscriber joypad_sub_;
  
  bool steering_inverted_;
};


}