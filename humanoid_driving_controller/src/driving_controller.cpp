#include <humanoid_driving_controller/driving_controller.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace humanoid_driving_controller {

DrivingController::DrivingController() :
    private_node_handle_("~")
{
    initKeyFrames();

    connection_loss_ = false;

    head_sensitivity_ = 2.5;
    steering_sensitivity_ = 2.5;

    last_command_received_.all_stop = true;
    last_command_received_.absolute_target_steering_angle = 0.0;
    last_command_received_.drive_forward = false;
    last_command_received_.absolute_head_tilt = 0.0f;
    last_command_received_.absolute_head_pan = 0.0f;
    time_from_start_ = 0.1;

    current_absolute_steering_angle_ = 0.0f;
    current_head_tilt_ = 0.0f;
    current_head_pan_ = 0.0f;

    driving_counter_ = 0;

    received_robot_positions_ = false;
    received_first_command_msg_ = false;
    last_command_received_time_ = ros::Time::now();
    last_auto_stop_info_sent_time_ = ros::Time::now();

    controller_enabled_ = false;

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
    all_stop_enabled_pub_ = node_handle_.advertise<humanoid_driving_controller::DrivingCommand>("driving_controller/all_stop", 1, false);

    // publish absolute steering angle
    driving_state_pub_ = node_handle_.advertise<humanoid_driving_controller::DrivingState>("driving_controller/driving_state", 1, true);

    // steering command subscriber
    driving_command_sub_ = node_handle_.subscribe("driving_controller/driving_command", 1, &DrivingController::handleDrivingCommand, this);

    // robot state subscriber
    std::string joint_state_topic;
    private_node_handle_.param("joint_state_topic", joint_state_topic, std::string("/thor_mang/joint_states"));
    joint_state_sub_ = node_handle_.subscribe(joint_state_topic, 1, &DrivingController::handleNewJointStateEvent, this);

    // shutdown subscriber
    controller_enable_sub_ = node_handle_.subscribe("driving_controller/controller_enable", 1, &DrivingController::handleControllerEnable, this);
    controller_enable_ack_pub_ = node_handle_.advertise<std_msgs::Bool>("driving_controller/controller_enable_ack", 1, false);
}

DrivingController::~DrivingController() {

}

void DrivingController::checkReceivedMessages() {
    if ( !received_robot_positions_ || !received_first_command_msg_ || !controller_enabled_) {
        return;
    }

    ros::Duration time_since_last_msg = ros::Time::now() - last_command_received_time_;
    if ( time_since_last_msg >= ros::Duration(1.0)) { // OCS not alive? Go to "all stop"
        //last_command_received_.all_stop = true;
        last_command_received_.drive_forward = false;
        updateDriveForward(false);

        connection_loss_ = true;

        // inform OCS of current state (once a second)
        if ( ros::Time::now() - last_auto_stop_info_sent_time_ >= ros::Duration(1.0) ) {
            all_stop_enabled_pub_.publish(last_command_received_);

            humanoid_driving_controller::DrivingState driving_state_msg;
            driving_state_msg.current_absolute_steering_angle = current_absolute_steering_angle_;
            driving_state_msg.connection_loss = connection_loss_;
            driving_state_msg.current_head_tilt = current_head_tilt_;
            driving_state_msg.current_head_pan = current_head_pan_;
            driving_state_msg.driving_counter = driving_counter_;
            driving_state_pub_.publish(driving_state_msg);
            ROS_WARN("[DrivingController] OCS connection timed out. Going to Hold.");
            last_auto_stop_info_sent_time_ = ros::Time::now();
        }
    }
    else {
        connection_loss_ = false;
    }
}

void DrivingController::handleDrivingCommand(humanoid_driving_controller::DrivingCommandConstPtr msg) {
    if ( !received_robot_positions_ ) {
        ROS_ERROR("[DrivingController] No robot positions received => No Update");
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
        //updateSteering();
        //updateHeadPosition();
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

void DrivingController::updateHeadPosition() {
    if ( !controller_enabled_ || last_command_received_.all_stop) {
        return;
    }

    // get current angles and add control offset

    std::vector<std::string> head_joint_names = {"head_pan", "head_tilt"};
    std::vector<double> current_head_positions(head_joint_names.size());

    for ( int i = 0; i < head_joint_names.size(); i++ ) {
        for ( int j = 0; j < robot_joint_names_.size(); j++ ) {
            if ( head_joint_names[i] == robot_joint_names_[j] ) {
                current_head_positions[i] = robot_joint_positions_[j];
                break;
            }
        }
    }

    current_head_pan_ = current_head_positions[0];
    current_head_tilt_ = current_head_positions[1];

    double head_pan_speed = 0.0;
    double pan_factor = std::min(1.0, 0.4*fabs(last_command_received_.absolute_head_pan - current_head_positions[0]));
    if ( last_command_received_.absolute_head_pan >= current_head_positions[0] ) {
        head_pan_speed = pan_factor * head_sensitivity_;
    }
    else {
        head_pan_speed = -pan_factor * head_sensitivity_;
    }

    double head_tilt_speed = 0.0;
    double tilt_factor = std::min(1.0, 0.4*fabs(last_command_received_.absolute_head_tilt - current_head_positions[1]));
    if ( last_command_received_.absolute_head_tilt >= current_head_positions[1] ) {
        head_tilt_speed = tilt_factor * head_sensitivity_;
    }
    else {
        head_tilt_speed = -tilt_factor * head_sensitivity_;
    }

    std::vector<double> target_head_positions = current_head_positions;
    target_head_positions[0] += head_pan_speed;
    target_head_positions[1] += head_tilt_speed;

    trajectory_msgs::JointTrajectory trajectory_msg = generateTrajectoryMsg(target_head_positions, head_joint_names);

    head_cmd_pub_.publish(trajectory_msg);
}


void DrivingController::updateSteering() {
    if ( !controller_enabled_ || last_command_received_.all_stop) {
        return;
    }

    double steering_speed = 0.0;

    double speed_factor = std::min(1.0, 0.5*fabs(last_command_received_.absolute_target_steering_angle - current_absolute_steering_angle_));
    if ( last_command_received_.absolute_target_steering_angle - current_absolute_steering_angle_ > 0.05 ) {
        steering_speed = steering_sensitivity_*speed_factor;
    }
    else if ( last_command_received_.absolute_target_steering_angle - current_absolute_steering_angle_ < -0.05) {
        steering_speed = -steering_sensitivity_*speed_factor;
    }

    double target_angle = current_absolute_steering_angle_ + steering_speed;
    current_absolute_steering_angle_ = target_angle;

    //ROS_INFO("absolute_target_steering_angle = %f", last_command_received_.absolute_target_steering_angle);
    //ROS_INFO("current_absolute_steering_angle = %f", current_absolute_steering_angle_);


    // map to range [0,360]
    while ( target_angle >= 360.0 )  target_angle -= 360.0;
    while ( target_angle < 0 ) target_angle += 360.0;

    std::vector<double> interpolated_frame = getInterpolatedKeyFrame(target_angle, 360.0);
    trajectory_msgs::JointTrajectory trajectory_msg = generateTrajectoryMsg(interpolated_frame, steering_joint_names_);
    steering_control_cmd_pub_.publish(trajectory_msg);

    if ( last_command_received_.drive_forward && !last_command_received_.all_stop )
        driving_counter_++;

    humanoid_driving_controller::DrivingState driving_state_msg;
    driving_state_msg.current_absolute_steering_angle = current_absolute_steering_angle_;
    driving_state_msg.connection_loss = connection_loss_;
    driving_state_msg.current_head_tilt = current_head_tilt_;
    driving_state_msg.current_head_pan = current_head_pan_;
    driving_state_msg.driving_counter = driving_counter_;
    driving_state_pub_.publish(driving_state_msg);
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

    last_command_received_.drive_forward = false;

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
