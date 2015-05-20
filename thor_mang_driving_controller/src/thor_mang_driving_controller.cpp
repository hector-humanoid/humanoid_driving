#include "thor_mang_driving_controller/thor_mang_driving_controller.h"

namespace thor_mang_driving_controller {
  
DrivingController::DrivingController() {
  joypad_sub_ = node_handle_.subscribe("joy", 1, &DrivingController::handleJoyPadEvent, this);
  steering_inverted_ = true;
}

DrivingController::~DrivingController() {
  
}
 
void DrivingController::handleJoyPadEvent(sensor_msgs::JoyConstPtr msg) {
  double steering_value = steering_inverted_ ? -msg->axes[DrivingController::STEERING] : msg->axes[DrivingController::STEERING];
  double forward_value = msg->axes[DrivingController::FORWARD];
  bool emergency_stop = msg->buttons[DrivingController::STOP];
  
  
  forward_value = std::max(forward_value, 0.0);
  ROS_INFO("Steering = %.3f, Forward = %.3f, STOP = %d", steering_value, forward_value, emergency_stop); 
}

void DrivingController::setSteeringInverted(bool inverted) {
  steering_inverted_ = inverted;
}
 
}