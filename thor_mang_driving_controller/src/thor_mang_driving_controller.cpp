#include "thor_mang_driving_controller/thor_mang_driving_controller.h"

#include <trajectory_msgs/JointTrajectory.h>

namespace thor_mang_driving_controller {

DrivingController::DrivingController() :
    private_node_handle_("~")
{
    initKeyFrames();

    last_command_received_.all_stop = true;
    last_command_received_.steering_angle_step = 0.0;
    last_command_received_.drive_forward = false;
    time_from_start_ = 5.0; // 5s for initial starting position

    absolute_steering_angle_ = 0.0;

    received_robot_positions_ = false;
    received_first_command_msg_ = false;
    last_command_received_time_ = ros::Time::now();
    last_auto_stop_info_sent_time_ = ros::Time::now();

    controller_enabled_ = false;

    // Head control elements
    private_node_handle_.getParam("head_joint_names", head_joint_names_);
    private_node_handle_.getParam("head_default_position", head_default_position_);

    // Setup head control publisher
    std::string head_controller_topic;
    private_node_handle_.param("head_controller_topic", head_controller_topic, std::string("/thor_mang/head_traj_controller/command"));
    head_cmd_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(head_controller_topic, 1, false);

    // steering publisher
    std::string steering_controller_topic;
    private_node_handle_.param("steering_controller_topic", steering_controller_topic, std::string("/thor_mang/left_arm_traj_controller/command"));
    steering_control_cmd_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(steering_controller_topic, 1, false);

    std::string speed_controller_topic;
    private_node_handle_.param("speed_controller_topic", speed_controller_topic, std::string("/thor_mang/right_leg_traj_controller/command"));
    speed_control_cmd_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(speed_controller_topic, 1, false);

    // all stop enabled on robot side
    all_stop_enabled_pub_ = node_handle_.advertise<thor_mang_driving_controller::DrivingCommand>("driving_controller/all_stop", 1, false);

    // publish absolute steering angle
    absolute_steering_angle_pub_ = node_handle_.advertise<std_msgs::Float64>("driving_controller/absolute_steering_angle", 1, true);

    // steering command subscriber
    driving_command_sub_ = node_handle_.subscribe("driving_controller/driving_command", 1, &DrivingController::handleDrivingCommand, this);

    // robot state subscriber
    std::string joint_state_topic;
    private_node_handle_.param("joint_state_topic", joint_state_topic, std::string("/thor_mang/joint_states"));
    joint_state_sub_ = node_handle_.subscribe(joint_state_topic, 1, &DrivingController::handleNewJointStateEvent, this);

    // shutdown subscriber
    controller_enable_sub_ = node_handle_.subscribe("driving_controller/controller_enable", 1, &DrivingController::handleControllerEnable, this);
    controller_enable_ack_pub_ = node_handle_.advertise<std_msgs::Bool>("driving_controller/controller_enable_ack", 1, false);
    controller_reset_sub_ = node_handle_.subscribe("driving_controller/controller_reset", 1, &DrivingController::handleControllerReset, this);

}

DrivingController::~DrivingController() {

}

void DrivingController::checkReceivedMessages() {
    if ( !received_robot_positions_ || !received_first_command_msg_ || !controller_enabled_) {
        return;
    }

    ros::Duration time_since_last_msg = ros::Time::now() - last_command_received_time_;
    if ( time_since_last_msg >= ros::Duration(0.5)) { // OCS not alive? Go to "all stop"
        last_command_received_.all_stop = true;
        allStop();

        // inform OCS of current state (once a second)
        if ( ros::Time::now() - last_auto_stop_info_sent_time_ >= ros::Duration(1.0) ) {
            all_stop_enabled_pub_.publish(last_command_received_);

            std_msgs::Float64 absolute_steering_angle_msg;
            absolute_steering_angle_msg.data = absolute_steering_angle_;
            absolute_steering_angle_pub_.publish(absolute_steering_angle_msg);
            ROS_WARN("[DrivingController] OCS connection timed out. Going to All-Stop.");
            last_auto_stop_info_sent_time_ = ros::Time::now();
        }
    }
}

void DrivingController::handleDrivingCommand(thor_mang_driving_controller::DrivingCommandConstPtr msg) {
    if ( !received_robot_positions_ ) {
        ROS_ERROR("No robot positions received => No Update");
        return;
    }

    if ( !controller_enabled_ ) {
        return;
    }

    last_command_received_time_ = ros::Time::now();
    last_command_received_ = *msg;

    if ( last_command_received_.all_stop ) {
        allStop();
    }
    else {
        updateSteering();
        updateHeadPosition();
        updateDriveForward(msg->drive_forward);
    }

    // first message received => go to default behaviour
    received_first_command_msg_ = true;
    time_from_start_ = 0.1;
}

void DrivingController::handleControllerEnable(std_msgs::BoolConstPtr msg) {
    controller_enabled_ = msg->data;
    controller_enable_ack_pub_.publish(msg);
    if ( controller_enabled_ )
        ROS_INFO("[DrivingController] Controller enabled.");
    else
        ROS_INFO("[DrivingController] Controller disabled.");
}

void DrivingController::handleControllerReset(std_msgs::EmptyConstPtr msg) {
    last_command_received_.steering_angle_step = 0.0;
    last_command_received_.drive_forward = false;

    absolute_steering_angle_ = 0.0;
}

void DrivingController::updateHeadPosition() {
    if ( !controller_enabled_ || last_command_received_.all_stop) {
        return;
    }

    // get current angles and add control offset
    std::vector<double> target_head_positions( head_joint_names_.size() );

    for ( int i = 0; i < head_joint_names_.size(); i++ ) {
        for ( int j = 0; j < robot_joint_names_.size(); j++ ) {
            if ( head_joint_names_[i] == robot_joint_names_[j] ) {
                target_head_positions[i] = robot_joint_positions_[j];
                break;
            }
        }
    }

    if ( last_command_received_.head_move_to_default ) {
        bool reached_position = true;
        for ( int i = 0; i < target_head_positions.size(); i++ ) {
            if ( std::abs(target_head_positions[i] - head_default_position_[i]) > 0.1 ) {
                reached_position = false;
                break;
            }
        }

        target_head_positions = head_default_position_;
        last_command_received_.head_move_to_default = !reached_position;
    }
    else {
        if ( head_joint_names_.size() >= 1 ) {
            if ( head_joint_names_[0].find("pan") != std::string::npos )
                target_head_positions[0] += last_command_received_.head_pan_speed;

            if ( head_joint_names_[0].find("tilt") != std::string::npos )
                target_head_positions[0] += last_command_received_.head_tilt_speed;
        }

        if ( head_joint_names_.size() >= 1 ) {
            if ( head_joint_names_[1].find("pan") != std::string::npos )
                target_head_positions[1] += last_command_received_.head_pan_speed;

            if ( head_joint_names_[1].find("tilt") != std::string::npos )
                target_head_positions[1] += last_command_received_.head_tilt_speed;
        }
    }

    trajectory_msgs::JointTrajectory trajectory_msg = generateTrajectoryMsg(target_head_positions, head_joint_names_);
    if(last_command_received_.head_move_to_default)
        trajectory_msg.points[0].time_from_start = ros::Duration(1.0);

    head_cmd_pub_.publish(trajectory_msg);
}


void DrivingController::updateSteering() {
    if ( !controller_enabled_ || last_command_received_.all_stop) {
        return;
    }

    //  ros::Duration diff = ros::Time::now() - last_steering_update_;
    //  double current_step = diff.toSec() * last_command_received_.steering_angle_step;
    double current_step = last_command_received_.steering_angle_step;

    absolute_steering_angle_ += current_step;
    if ( last_command_received_.ignore_steering_limits == false && absolute_steering_angle_ >= 538.0 )
      absolute_steering_angle_ = 538.0;
    else if ( last_command_received_.ignore_steering_limits == false && absolute_steering_angle_ <= -538.0 )
      absolute_steering_angle_ = -538.0;

    double target_angle = absolute_steering_angle_;    

    // map to range [0,360]
    while ( target_angle >= 360.0 )  target_angle -= 360.0;
    while ( target_angle < 0 ) target_angle += 360.0;

    std::vector<double> interpolated_frame = getInterpolatedKeyFrame(target_angle, 360.0);
    trajectory_msgs::JointTrajectory trajectory_msg = generateTrajectoryMsg(interpolated_frame, steering_joint_names_);
    steering_control_cmd_pub_.publish(trajectory_msg);

    std_msgs::Float64 absolute_steering_angle_msg;
    absolute_steering_angle_msg.data = absolute_steering_angle_;
    absolute_steering_angle_pub_.publish(absolute_steering_angle_msg);
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
    private_node_handle_.getParam("speed_control_joints", speed_control_joint_names_);
    private_node_handle_.getParam("forward_position", drive_forward_position_);
    private_node_handle_.getParam("stop_position", stop_position_);
    private_node_handle_.getParam("safety_position", safety_position_);
}


void DrivingController::handleNewJointStateEvent(sensor_msgs::JointStateConstPtr msg) {
    robot_joint_names_ = msg->name;
    robot_joint_positions_ = msg->position;
    received_robot_positions_ = true;
}

void DrivingController::allStop() {
    // go to safety position
    std::vector<double> all_stop_leg_position = getRobotJointPositions(leg_joint_names_, speed_control_joint_names_, safety_position_);
    trajectory_msgs::JointTrajectory trajectory_msg = generateTrajectoryMsg(all_stop_leg_position, leg_joint_names_);
    speed_control_cmd_pub_.publish(trajectory_msg);

    // stop steering
    std::vector<double> current_steering_position = getRobotJointPositions(steering_joint_names_);
    trajectory_msg = generateTrajectoryMsg(current_steering_position, steering_joint_names_);
    steering_control_cmd_pub_.publish(trajectory_msg);
}


void DrivingController::updateDriveForward(bool drive) {
    if ( !controller_enabled_ ) {
        return;
    }

    trajectory_msgs::JointTrajectory trajectory_msg;
    if ( drive ) {
        std::vector<double> forward_positions = getRobotJointPositions(leg_joint_names_, speed_control_joint_names_, drive_forward_position_);
        trajectory_msg = generateTrajectoryMsg(forward_positions, leg_joint_names_);
    }
    else {
        std::vector<double> stop_positions = getRobotJointPositions(leg_joint_names_, speed_control_joint_names_, stop_position_);
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

std::vector<double> DrivingController::getRobotJointPositions(std::vector<std::string> &joint_names, std::vector<std::string> replace_joint_names, std::vector<double> replace_joint_angles) {
    std::vector<double> joint_positions;

    for ( int i = 0; i < joint_names.size(); i++ ) {
        bool joint_set = false;

        // see if this is one joint to replace
        for ( int j = 0; j < replace_joint_names.size(); j++ ) {
            if ( joint_names[i] == replace_joint_names[j] ) {
                joint_positions.push_back(replace_joint_angles[j]);

                // remove already found values
                replace_joint_names.erase (replace_joint_names.begin()+j);
                replace_joint_angles.erase(replace_joint_angles.begin()+j);

                joint_set = true;
                break;
            }
        }

        if ( joint_set == false) { // nothing found => take current position
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
    trajectory_point.time_from_start = ros::Duration(time_from_start_);
    trajectory_msg.points.push_back(trajectory_point);
    trajectory_msg.joint_names = joint_names;

    return trajectory_msg;
}

}
