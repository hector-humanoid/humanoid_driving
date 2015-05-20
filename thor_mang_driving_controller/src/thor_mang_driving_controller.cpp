#include "thor_mang_driving_controller/thor_mang_driving_controller.h"

#include <trajectory_msgs/JointTrajectory.h>

namespace thor_mang_driving_controller {
  
DrivingController::DrivingController() :
    private_node_handle_("~")
{
  initPoseVector();

  joypad_sub_ = node_handle_.subscribe("joy", 1, &DrivingController::handleJoyPadEvent, this);

  steering_inverted_ = true;
  private_node_handle_.getParam("arm_controller_topic", steering_controller_topic_);

  steering_control_cmd_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(steering_controller_topic_, 10, false);
}

DrivingController::~DrivingController() {
  
}
 
void DrivingController::handleJoyPadEvent(sensor_msgs::JoyConstPtr msg) {
  double steering_value = steering_inverted_ ? -msg->axes[DrivingController::STEERING] : msg->axes[DrivingController::STEERING];
  double forward_value = msg->axes[DrivingController::FORWARD];
  bool emergency_stop = msg->buttons[DrivingController::STOP];
  
  
  forward_value = std::max(forward_value, 0.0);
  ROS_INFO("Steering = %.3f, Forward = %.3f, STOP = %d", steering_value, forward_value, emergency_stop); 

  if( steering_value > 0.5 ) { // right
    sendNewSteeringTarget(1);
  }
  else if ( steering_value < -0.5 ) { // left
    sendNewSteeringTarget(-1);
  }
}

void DrivingController::setSteeringInverted(bool inverted) {
  steering_inverted_ = inverted;
}

void DrivingController::initPoseVector() {
    current_steering_position_idx_ = 0;
}

void DrivingController::sendNewSteeringTarget(int offset) {
    trajectory_msgs::JointTrajectory target_trajectory;
    target_trajectory.joint_names = steering_joint_names_;

    int step = (offset>=0) ? 1 : -1;
    for ( int i = 0; i != offset; i += step ) {
        trajectory_msgs::JointTrajectoryPoint trajectory_point;

        int target_idx = current_steering_position_idx_ + offset;
        if ( target_idx < 0 )
            target_idx += steering_target_positions_.size();
        else if ( target_idx >= steering_target_positions_.size() )
            target_idx -= steering_target_positions_.size();

        trajectory_point.positions = steering_target_positions_[i];
        target_trajectory.points.push_back(trajectory_point);
    }

    steering_control_cmd_pub_.publish(target_trajectory);
}
 
}
