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
    current_absolute_angle_ = 0.0;
    steering_speed_ = 0.0;
    all_stop_active_ = false;

    private_node_handle_.getParam("steering_controller_topic", steering_controller_topic_);
    private_node_handle_.getParam("speed_controller_topic", speed_controller_topic_);
    private_node_handle_.getParam("joint_state_topic", joint_state_topic_);

    steering_control_cmd_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(steering_controller_topic_, 1, false);
    speed_control_cmd_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(speed_controller_topic_, 1, false);

    joypad_sub_ = node_handle_.subscribe("joy", 1, &DrivingController::handleJoyPadEvent, this);
    joint_state_sub_ = node_handle_.subscribe(joint_state_topic_, 1, &DrivingController::handleNewJointStateEvent, this);
}

DrivingController::~DrivingController() {

}

void DrivingController::updateSteering() {
    if ( all_stop_active_ ) {
        ROS_INFO("All-Stop active! Steering blocked!");
        return;
    }
    if (current_absolute_angle_ + steering_speed_ <= -540.0 ||
        current_absolute_angle_ + steering_speed_ >=  540.0 ) {
        ROS_INFO("Rotation blocked: No more than 1.5 turns left / right allowed");
        return; // do not allow more than 3 turns total
    }
    current_absolute_angle_ += steering_speed_;

    double target_angle = current_steering_angle_ + steering_speed_;
    if ( target_angle < 0.0 )
        target_angle += 360.0;
    else if ( target_angle >= 360.0)
        target_angle -= 360.0;

    std::vector<double> interpolated_frame = getInterpolatedKeyFrame(target_angle, 360.0);

    current_steering_angle_ = target_angle;
    trajectory_msgs::JointTrajectory trajectory_msg = generateTrajectoryMsg(interpolated_frame, steering_joint_names_);
    steering_control_cmd_pub_.publish(trajectory_msg);
}

void DrivingController::initKeyFrames() {
    // load steering key frames
    private_node_handle_.getParam("joints", steering_joint_names_);

    std::vector<double> steering_angles;
    private_node_handle_.getParam("angles", steering_angles);

    for ( int i = 0; i < steering_angles.size(); i++ ) {
        std::stringstream key_position_name;
        key_position_name << "angle_" << (int)steering_angles[i];
        std::string name = key_position_name.str();

        std::vector<double> steering_key_frame;
        private_node_handle_.getParam(name.c_str(), steering_key_frame);

        steering_key_frames_[steering_angles[i]] = steering_key_frame;
    }

    // load speed control key frames
    private_node_handle_.getParam("leg_joints", leg_joint_names_);
    private_node_handle_.getParam("speed_control_joint", speed_control_joint_name_);
    private_node_handle_.getParam("angle_forward", drive_forward_angle_);
    private_node_handle_.getParam("angle_stop", stop_angle_);

    //private_node_handle_.getParam("angle_e_stop", e_stop_frame_);

}

void DrivingController::handleJoyPadEvent(sensor_msgs::JoyConstPtr msg) {

    handleSteeringCommand(msg->axes[DrivingController::STEERING]);

    forwardDrive( msg->buttons[DrivingController::FORWARD] );

    if ( msg->buttons[DrivingController::ALL_STOP] )
        allStop();

    if ( msg->buttons[DrivingController::STEERING_SENSITIVITY_PLUS] )
        changeSteeringSensitivity(steering_sensitivity_step);

    if ( msg->buttons[DrivingController::STEERING_SENSITIVITY_MINUS] )
        changeSteeringSensitivity(-steering_sensitivity_step);

    if ( msg->buttons[DrivingController::HEAD_LEFT] )
        moveHead(-1);

    if ( msg->buttons[DrivingController::HEAD_RIGHT] )
        moveHead(+1);
}

void DrivingController::handleNewJointStateEvent(sensor_msgs::JointStateConstPtr msg) {
    robot_joint_names_ = msg->name;
    robot_joint_positions_ = msg->position;
}

void DrivingController::setSteeringInverted(bool inverted) {
    steering_inverted_ = inverted;
}

void DrivingController::allStop() {
    all_stop_active_ = !all_stop_active_;

    // stop driving
    std::vector<double> all_stop_leg_position = getRobotJointPositions(leg_joint_names_, speed_control_joint_name_, all_stop_angle_);
    trajectory_msgs::JointTrajectory trajectory_msg = generateTrajectoryMsg(all_stop_leg_position, leg_joint_names_);
    speed_control_cmd_pub_.publish(trajectory_msg);

    // stop steering
    std::vector<double> current_steering_position = getRobotJointPositions(steering_joint_names_);
    trajectory_msg = generateTrajectoryMsg(current_steering_position, steering_joint_names_);
    steering_control_cmd_pub_.publish(trajectory_msg);
}

void DrivingController::moveHead(int value) {

}

void DrivingController::changeSteeringSensitivity(double diff) {
    steering_sensitivity_ += diff;
}

void DrivingController::handleSteeringCommand(double value) {
    steering_speed_ = steering_inverted_ ? -(steering_sensitivity_ * value) : (steering_sensitivity_ * value);
}

void DrivingController::forwardDrive(bool drive) {
    if ( all_stop_active_ ) {
        ROS_INFO("E-Stop active! Steering blocked!");
        return;
    }

    trajectory_msgs::JointTrajectory trajectory_msg;
    if ( drive ) {
        std::vector<double> forward_positions = getRobotJointPositions(leg_joint_names_, speed_control_joint_name_, drive_forward_angle_);
        trajectory_msg = generateTrajectoryMsg(forward_positions, leg_joint_names_);
    }
    else {
        std::vector<double> stop_positions = getRobotJointPositions(leg_joint_names_, speed_control_joint_name_, stop_angle_);
        trajectory_msg = generateTrajectoryMsg(stop_positions, leg_joint_names_);
    }

    speed_control_cmd_pub_.publish(trajectory_msg);
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

std::vector<double> DrivingController::getRobotJointPositions(std::vector<std::string> &joint_names, std::string replace_joint_name, double replace_joint_angle) {
    std::vector<double> joint_positions;
    for ( int i = 0; i < joint_names.size(); i++ ) {
        if ( joint_names[i] == replace_joint_name ) {
            joint_positions.push_back(replace_joint_angle);
        }
        else {
            std::vector<std::string>::iterator it = std::find(robot_joint_names_.begin(), robot_joint_names_.end(), joint_names[i]);
            int idx = distance(robot_joint_names_.begin(), it);
            joint_positions.push_back(robot_joint_positions_[idx]);
        }
    }

    return joint_positions;
}

trajectory_msgs::JointTrajectory DrivingController::generateTrajectoryMsg(std::vector<double> &joint_angles, std::vector<std::string> joint_names) {
    trajectory_msgs::JointTrajectory trajectory_msg;
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions = joint_angles;
    trajectory_point.time_from_start = ros::Duration(1.0);
    trajectory_msg.points.push_back(trajectory_point);
    trajectory_msg.joint_names = joint_names;

    return trajectory_msg;
}

}
