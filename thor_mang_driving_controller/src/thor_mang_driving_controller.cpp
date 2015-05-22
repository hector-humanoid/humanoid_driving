#include "thor_mang_driving_controller/thor_mang_driving_controller.h"

#include <trajectory_msgs/JointTrajectory.h>

namespace thor_mang_driving_controller {

DrivingController::DrivingController() :
    private_node_handle_("~")
{
    initKeyFrames();

    all_stop_ = true;
    time_from_start_ = 0.1;
    received_robot_positions_ = false;
    last_command_received_time_ = ros::Time::now();
    last_auto_stop_info_sent_time_ = ros::Time::now();

    // steering publisher
    std::string steering_controller_topic;
    private_node_handle_.param("steering_controller_topic", steering_controller_topic, std::string("/thor_mang/left_arm_traj_controller/command"));
    steering_control_cmd_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(steering_controller_topic, 1, false);

    std::string speed_controller_topic;
    private_node_handle_.param("speed_controller_topic", speed_controller_topic, std::string("/thor_mang/right_leg_traj_controller/command"));
    speed_control_cmd_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(speed_controller_topic, 1, false);

    // all stop enabled on robot side
    all_stop_enabled_pub_ = node_handle_.advertise<std_msgs::Bool>("driving_controller/all_stop", 1, true);

    // steering command subscriber
    driving_command_sub_ = node_handle_.subscribe("driving_controller/driving_command", 1, &DrivingController::handleDrivingCommand, this);

    // robot state subscriber
    std::string joint_state_topic;
    private_node_handle_.param("joint_state_topic", joint_state_topic, std::string("/thor_mang/joint_states"));
    joint_state_sub_ = node_handle_.subscribe(joint_state_topic, 1, &DrivingController::handleNewJointStateEvent, this);

    // shutdown subscriber
    shutdown_sub_ = node_handle_.subscribe("driving_controller/shutdown", 1, &DrivingController::handleShutDown, this);


}

DrivingController::~DrivingController() {

}

void DrivingController::checkReceivedMessages() {
    ros::Duration time_since_last_msg = ros::Time::now() - last_command_received_time_;
    if ( time_since_last_msg >= ros::Duration(1.0)) { // OCS not alive? Go to "all stop"
        all_stop_ = true;
        allStop();

        // inform OCS (once a second)
        if ( ros::Time::now() - last_auto_stop_info_sent_time_ >= ros::Duration(1.0) ) {
            std_msgs::Bool all_stop_enabled_msg;
            all_stop_enabled_msg.data = true;
            all_stop_enabled_pub_.publish(all_stop_enabled_msg);
            last_auto_stop_info_sent_time_ = ros::Time::now();
        }
    }
}

void DrivingController::handleDrivingCommand(thor_mang_driving_controller::DrivingCommandConstPtr msg) {
    if ( !received_robot_positions_ ) {
        ROS_ERROR("No robot positions received => No Update");
        return;
    }

    last_command_received_time_ = ros::Time::now();

    time_from_start_ = msg->time_from_start.data;
    all_stop_ = msg->all_stop.data;

    if ( all_stop_ ) {
        allStop();
    }
    else {
        updateSteering(msg->steering_angle.data);
        updateDriveForward(msg->drive_forward.data);
    }
}

void DrivingController::handleShutDown(std_msgs::EmptyConstPtr msg) {
    ros::shutdown();
}

void DrivingController::updateSteering(double target_angle) {
    std::vector<double> interpolated_frame = getInterpolatedKeyFrame(target_angle, 360.0);
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
    private_node_handle_.getParam("speed_control_joints", speed_control_joint_names_);
    private_node_handle_.getParam("forward_position", drive_forward_position_);
    private_node_handle_.getParam("stop_position", stop_position_);
    private_node_handle_.getParam("stop_position", safety_position_);
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
