#include <humanoid_driving_controller/driving_controller.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace humanoid_driving_controller {

DrivingController::DrivingController() :
    private_node_handle_("~")
{
    initKeyFrames();

    // init current state
    current_state_.steering_wheel_angle = 0.0f;
    current_state_.head_tilt = 0.0f;
    current_state_.head_pan = 0.0f;
    current_state_.driving_counter = 0;
    current_state_.connection_loss = false;

    // init driving command with "stop" values
    driving_command_.all_hold = true;
    driving_command_.steering_wheel_angle = 0.0;
    driving_command_.drive_forward = false;
    driving_command_.head_tilt = 0.0f;
    driving_command_.head_pan = 0.0f;

    driving_command_time_ = ros::Time();

    last_auto_stop_info_sent_time_ = ros::Time::now();
    controller_enabled_ = false;

    // load head control joint names
    private_node_handle_.param("head_pan_joint_name", head_pan_joint_name_, std::string(""));
    private_node_handle_.param("head_tilt_joint_name", head_tilt_joint_name_, std::string(""));

    // Setup head control publisher
    std::string head_controller_topic;
    private_node_handle_.param("head_controller_topic", head_controller_topic, std::string("/thor_mang/head_traj_controller/command"));
    head_cmd_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(head_controller_topic, 1, false);

    // Steering publisher
    std::string steering_controller_topic;
    private_node_handle_.param("steering_controller_topic", steering_controller_topic, std::string("/thor_mang/left_arm_traj_controller/command"));
    steering_control_cmd_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(steering_controller_topic, 1, false);

    // Speed / Throttle publisher
    std::string speed_controller_topic;
    private_node_handle_.param("speed_controller_topic", speed_controller_topic, std::string("/thor_mang/right_leg_traj_controller/command"));
    speed_control_cmd_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(speed_controller_topic, 1, false);

    // All hold enabled on robot side => inform widget
    all_hold_enabled_pub_ = node_handle_.advertise<humanoid_driving_controller::DrivingCommand>("driving_controller/all_hold", 1, false);

    // Publish current driving state
    driving_state_pub_ = node_handle_.advertise<humanoid_driving_controller::DrivingState>("driving_controller/driving_state", 1, true);

    // Steering command subscriber
    driving_command_sub_ = node_handle_.subscribe("driving_controller/driving_command", 1, &DrivingController::handleDrivingCommand, this);

    // Robot state subscriber
    std::string joint_state_topic;
    private_node_handle_.param("joint_state_topic", joint_state_topic, std::string("/thor_mang/joint_states"));
    joint_state_sub_ = node_handle_.subscribe(joint_state_topic, 1, &DrivingController::handleNewJointStateEvent, this);

    // Enable / disable subscriber and acknowledgement
    controller_enable_sub_ = node_handle_.subscribe("driving_controller/controller_enable", 1, &DrivingController::handleControllerEnable, this);
    controller_enable_ack_pub_ = node_handle_.advertise<std_msgs::Bool>("driving_controller/controller_enable_ack", 1, false);
}

DrivingController::~DrivingController() {

}

void DrivingController::checkReceivedMessages() {
    if ( robot_joint_positions_.empty() || driving_command_time_.isZero() || !controller_enabled_) {
        return;
    }

    ros::Duration time_since_last_msg = ros::Time::now() - driving_command_time_;
    if ( time_since_last_msg >= ros::Duration(1.0)) { // OCS not alive? Go to "all hold"
        allHold();

        current_state_.connection_loss = true;

        // inform OCS of current state (once a second)
        if ( ros::Time::now() - last_auto_stop_info_sent_time_ >= ros::Duration(1.0) ) {
            all_hold_enabled_pub_.publish(driving_command_);
            driving_state_pub_.publish(current_state_);

            ROS_WARN("[DrivingController] OCS connection timed out. Going to Hold.");
            last_auto_stop_info_sent_time_ = ros::Time::now();
        }
    }
    else {
        current_state_.connection_loss = false;
    }
}

void DrivingController::updateSteering() {
    if ( !controller_enabled_ || driving_command_.all_hold) {
        return;
    }

    // calculate steering speed from difference to target
    double steering_speed = 0.0;
    double speed_factor = std::min(1.0, SteeringAngleDifferenceSpeedFactor*fabs(driving_command_.steering_wheel_angle - current_state_.steering_wheel_angle));
    if ( driving_command_.steering_wheel_angle - current_state_.steering_wheel_angle > MaxSteeringAngleOffset ) {
        steering_speed = SteeringSensitivity*speed_factor;
    }
    else if ( driving_command_.steering_wheel_angle - current_state_.steering_wheel_angle < -MaxSteeringAngleOffset) {
        steering_speed = -SteeringSensitivity*speed_factor;
    }

    // calculate target angle for this time step
    double target_angle = current_state_.steering_wheel_angle + steering_speed;
    current_state_.steering_wheel_angle = (float)target_angle;

    // map to range [0,360]
    double target_angle_deg = rad2deg(target_angle);
    while ( target_angle_deg >= 360.0 )  target_angle_deg -= 360.0;
    while ( target_angle_deg < 0 )       target_angle_deg += 360.0;

    // interpolate between neighboring key frames
    std::vector<double> interpolated_frame = getInterpolatedKeyFrame(target_angle, 360.0);
    trajectory_msgs::JointTrajectory trajectory_msg = generateTrajectoryMsg(interpolated_frame, steering_joint_names_);
    steering_control_cmd_pub_.publish(trajectory_msg);

    // update driving counter
    if ( driving_command_.drive_forward && !driving_command_.all_hold )
        current_state_.driving_counter++;

    driving_state_pub_.publish(current_state_);
}

void DrivingController::updateHeadPosition() {
    if ( !controller_enabled_ || driving_command_.all_hold) {
        return;
    }

    if ( robot_joint_positions_.empty() ) { // do nothing if there are currently no robot joint states
        ROS_ERROR("[DrivingController] No robot positions received => No Update");
        return;
    }

    // get current angles
    std::vector<double> current_head_positions;
    std::vector<std::string> head_joint_names;
    int head_tilt_idx = -1;
    int head_pan_idx = -1;
    for ( int i = 0; i < robot_joint_names_.size(); i++ ) {
        if ( robot_joint_names_[i] == head_pan_joint_name_ ) {
            current_head_positions.push_back(robot_joint_positions_[i]);
            head_joint_names.push_back(head_pan_joint_name_);
            head_pan_idx = (int)current_head_positions.size()-1;
        }
        else if ( robot_joint_names_[i] == head_tilt_joint_name_ ) {
            current_head_positions.push_back(robot_joint_positions_[i]);
            head_joint_names.push_back(head_tilt_joint_name_);
            head_tilt_idx = (int)current_head_positions.size()-1;
        }
    }

    if ( current_head_positions.size() == 0 )
        return;

    std::vector<double> target_head_positions = current_head_positions;

    // create new target angles for head pan
    if ( head_pan_idx >= 0 ) {
        current_state_.head_pan = (float)current_head_positions[0];

        double head_pan_speed = 0.0;
        double pan_factor = std::min(1.0, HeadAngleDifferenceSpeedFactor*fabs(driving_command_.head_pan - current_head_positions[0]));
        if ( driving_command_.head_pan >= current_head_positions[0] ) {
            head_pan_speed = pan_factor * HeadSensitivity;
        }
        else {
            head_pan_speed = -pan_factor * HeadSensitivity;
        }

        target_head_positions[head_pan_idx] += head_pan_speed;
    }

    // create new target angles for head tilt
    if ( head_tilt_idx >= 0 ) {
        current_state_.head_tilt = (float)current_head_positions[1];

        double head_tilt_speed = 0.0;
        double tilt_factor = std::min(1.0, HeadAngleDifferenceSpeedFactor*fabs(driving_command_.head_tilt - current_head_positions[1]));
        if ( driving_command_.head_tilt >= current_head_positions[1] ) {
            head_tilt_speed = tilt_factor * HeadSensitivity;
        }
        else {
            head_tilt_speed = -tilt_factor * HeadSensitivity;
        }

        target_head_positions[head_tilt_idx] += head_tilt_speed;
    }

    trajectory_msgs::JointTrajectory trajectory_msg = generateTrajectoryMsg(target_head_positions, head_joint_names);
    head_cmd_pub_.publish(trajectory_msg);
}

void DrivingController::handleDrivingCommand(humanoid_driving_controller::DrivingCommandConstPtr msg) {
    if ( !controller_enabled_ ) { // do nothing if the controller is disabled
        return;
    }

    if ( robot_joint_positions_.empty() ) { // do nothing if there are currently no robot joint states
        ROS_ERROR("[DrivingController] No robot positions received => No Update");
        return;
    }

    // update last time a command was received
    driving_command_time_ = ros::Time::now();
    driving_command_ = *msg;

    // update leg position
    if ( driving_command_.all_hold ) {
        allHold();
    }
    else {
        updateDriveForward(msg->drive_forward);
    }

    // steering and head positions are updated the next time updateSteering() and updateHeadPosition() are called
}

void DrivingController::handleNewJointStateEvent(sensor_msgs::JointStateConstPtr msg) {
    robot_joint_names_ = msg->name;
    robot_joint_positions_ = msg->position;
}

void DrivingController::handleControllerEnable(std_msgs::BoolConstPtr msg) {
    controller_enabled_ = msg->data;
    controller_enable_ack_pub_.publish(msg);
    if ( controller_enabled_ ) {
        ROS_INFO("[DrivingController] Controller enabled.");
    }
    else {
        ROS_INFO("[DrivingController] Controller disabled. Going to all-hold.");
        allHold();
    }
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

void DrivingController::allHold() {
    // just to make sure...
    driving_command_.drive_forward = false;
    driving_command_.all_hold = true;

    // go to safety position
    std::vector<double> all_hold_leg_position = getRobotJointPositions(leg_joint_names_, speed_control_joint_names_, safety_position_);
    trajectory_msgs::JointTrajectory leg_trajectory_msg = generateTrajectoryMsg(all_hold_leg_position, leg_joint_names_);
    speed_control_cmd_pub_.publish(leg_trajectory_msg);

    // stop steering
    std::vector<double> current_steering_position = getRobotJointPositions(steering_joint_names_);
    trajectory_msgs::JointTrajectory steering_trajectory_msg = generateTrajectoryMsg(current_steering_position, steering_joint_names_);
    steering_control_cmd_pub_.publish(steering_trajectory_msg);
}

std::vector<double> DrivingController::getInterpolatedKeyFrame(double value, double max_value) {
    double previous_value = getPreviousSteeringKey(value);
    double next_value = getNextSteeringKey(value);

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

double DrivingController::getNextSteeringKey(double key) {
    for ( std::map<double, std::vector<double> >::const_iterator it = steering_key_frames_.begin(); it != steering_key_frames_.end(); it++ ) {
        if ( it->first > key) {
            return it->first;
        }
    }

    return steering_key_frames_.begin()->first;
}

double DrivingController::getPreviousSteeringKey(double key) {
    double return_value = 0.0;
    for ( std::map<double, std::vector<double> >::const_iterator it = steering_key_frames_.begin(); it != steering_key_frames_.end(); it++ ) {
        if ( key <= it->first ) {
            return return_value;
        }

        return_value = it->first;
    }

    return (--steering_key_frames_.end())->first;
}

std::vector<double> DrivingController::getRobotJointPositions(std::vector<std::string> &joint_names, std::vector<std::string> replace_joint_names, std::vector<double> replace_joint_angles) {
    assert(replace_joint_names.size() == replace_joint_angles.size() && "DrivingController::getRobotJointPositions(): replace_joint_names and replace_joint_angles have different sizes\n");

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
            size_t idx = distance(robot_joint_names_.begin(), it);
            joint_positions.push_back(robot_joint_positions_[idx]);
        }
    }

    return joint_positions;
}

trajectory_msgs::JointTrajectory DrivingController::generateTrajectoryMsg(std::vector<double> &joint_angles, std::vector<std::string> joint_names) {
    trajectory_msgs::JointTrajectory trajectory_msg;
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions = joint_angles;
    trajectory_point.time_from_start = ros::Duration(MovementTargetTime);
    trajectory_msg.points.push_back(trajectory_point);
    trajectory_msg.joint_names = joint_names;

    return trajectory_msg;
}

double DrivingController::rad2deg(double rad) {
    return rad*180.0/M_PI;
}

}
