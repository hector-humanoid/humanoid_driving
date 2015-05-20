#include "thor_mang_driving_controller/thor_mang_driving_controller.h"

#include <trajectory_msgs/JointTrajectory.h>

namespace thor_mang_driving_controller {

DrivingController::DrivingController() :
    private_node_handle_("~")
{
    initKeyFrames();

    steering_inverted_ = true;
    steering_sensitivity_ = 1.0;
    current_steering_angle_ = 0.0;
    steering_speed_ = 0.0;
    private_node_handle_.getParam("steering_controller_topic", steering_controller_topic_);
    private_node_handle_.getParam("speed_controller_topic", speed_controller_topic_);

    steering_control_cmd_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(steering_controller_topic_, 1, false);
    speed_control_cmd_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(speed_controller_topic_, 1, false);

    joypad_sub_ = node_handle_.subscribe("joy", 1, &DrivingController::handleJoyPadEvent, this);
}

DrivingController::~DrivingController() {

}

void DrivingController::updateSteering() {
    double target_angle = current_steering_angle_ + steering_speed_;
    if ( target_angle < 0.0 )
        target_angle += 360.0;
    else if ( target_angle >= 360.0)
        target_angle -= 360.0;

    //ROS_INFO("target_angle: %f", target_angle);
    std::vector<double> interpolated_frame = getInterpolatedKeyFrame(target_angle, 360.0);
    //ROS_INFO("interpolated_frame: %f", interpolated_frame[1]);

    trajectory_msgs::JointTrajectory target_trajectory;
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions = interpolated_frame;
    target_trajectory.points.push_back(trajectory_point);

    current_steering_angle_ = target_angle;

    //steering_control_cmd_pub_.publish(target_trajectory);
}

void DrivingController::initKeyFrames() {
    private_node_handle_.getParam("joints", steering_joint_names_);

    std::vector<double> steering_angles;
    private_node_handle_.getParam("angles", steering_angles);

    std::vector<std::string> steering_angles_str;
    private_node_handle_.getParam("angles", steering_angles_str);

    for ( int i = 0; i < steering_angles.size(); i++ ) {
        //std::stringstream key_position_name;
        //key_position_name << (int)steering_angles[i];

        std::vector<double> steering_key_frame;
        private_node_handle_.getParam(steering_angles_str[i].c_str(), steering_key_frame);

        steering_key_frames_[steering_angles[i]] = steering_key_frame;
    }
}

void DrivingController::handleJoyPadEvent(sensor_msgs::JoyConstPtr msg) {

    handleSteeringCommand(msg->axes[DrivingController::STEERING]);
    handleSpeedCommand(msg->axes[DrivingController::SPEED]);

    if ( msg->buttons[DrivingController::E_STOP] )
        eStop();

    if ( msg->buttons[DrivingController::STEERING_SENSITIVITY_PLUS] )
        changeSteeringSensitivity(steering_sensitivity_step);

    if ( msg->buttons[DrivingController::STEERING_SENSITIVITY_MINUS] )
        changeSteeringSensitivity(-steering_sensitivity_step);

    if ( msg->buttons[DrivingController::HEAD_LEFT] )
        moveHead(-1);

    if ( msg->buttons[DrivingController::HEAD_RIGHT] )
        moveHead(+1);
}

void DrivingController::setSteeringInverted(bool inverted) {
    steering_inverted_ = inverted;
}

void DrivingController::eStop() {

}

void DrivingController::moveHead(int value) {

}

void DrivingController::changeSteeringSensitivity(double diff) {
    steering_sensitivity_ += diff;
}

void DrivingController::handleSteeringCommand(double value) {
    steering_speed_ = steering_inverted_ ? -(steering_sensitivity_ * value) : (steering_sensitivity_ * value);
}

void DrivingController::handleSpeedCommand(double value) {
    double speed_value = std::max(value, 0.0);

    //ROS_INFO("target_value: %f", speed_value);
    std::vector<double> interpolated_frame = getInterpolatedKeyFrame(speed_value, 1.0);
    //ROS_INFO("interpolated_frame: %f", interpolated_frame[1]);

    trajectory_msgs::JointTrajectory target_trajectory;
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions = interpolated_frame;
    target_trajectory.points.push_back(trajectory_point);

    //speed_control_cmd_pub_.publish(target_trajectory);
}

std::vector<double> DrivingController::getInterpolatedKeyFrame(double value, double max_value) {
    double previous_value = getPreviousValue(value);
    double next_value = getNextValue(value);

    std::vector<double> previous_frame = steering_key_frames_[previous_value];
    std::vector<double> next_frame = steering_key_frames_[next_value];

    std::vector<double> result_frame(previous_frame.size());
    double prev_diff = value-previous_value;
    double next_diff = next_value-value;

    if ( prev_diff < 0.0 ) prev_diff += max_value;
    if ( next_diff < 0.0 ) next_diff += max_value;

    for ( int i = 0; i < previous_frame.size(); i++) {
        result_frame[i] = (next_diff * previous_frame[i] + prev_diff *next_frame[i]) / (prev_diff + next_diff);
    }

    return result_frame;
}

double DrivingController::getNextValue(double value) {
    for ( std::map<double, std::vector<double> >::const_iterator it = steering_key_frames_.begin(); it != steering_key_frames_.end(); it++ ) {
        if ( it->first > value) {
            return it->first;
        }
    }

    return steering_key_frames_.begin()->first;
}

double DrivingController::getPreviousValue(double value) {
    double return_value = 0.0;
    for ( std::map<double, std::vector<double> >::const_iterator it = steering_key_frames_.begin(); it != steering_key_frames_.end(); it++ ) {
        if ( value <= it->first ) {
            return return_value;
        }

        return_value = it->first;
    }

    return (--steering_key_frames_.end())->first;
}

}
