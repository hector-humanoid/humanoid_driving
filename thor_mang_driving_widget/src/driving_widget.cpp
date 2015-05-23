#include "driving_widget.h"
#include "ui_driving_widget.h"

#include <ros/ros.h>

#include <QGraphicsItem>

DrivingWidget::DrivingWidget(QWidget *parent) :
    QMainWindow(parent),
    ui_(new Ui::DrivingWidget),
    node_handle_private_("~")
{
    ui_->setupUi(this);

    // steering parameters
    steering_sensitivity_ = 0.3;
    steering_correction_ = -1.0;
    head_sensitivity_ = 0.1;
    head_tilt_correction_ = -1.0;
    head_pan_correction_ = 1.0;


    // Driving control elements
    all_stop_ = true;
    steering_angle_ = 0.0;
    absolute_steering_angle_ = 0.0;
    drive_forward_ = false;
    time_from_start_ = 0.1;

    // Head control elements
    head_tilt_speed_ = 0.0;
    head_pan_speed_ = 0.0;

    // Get camera image topic
    node_handle_private_.param("camera_topic", camera_topic_, std::string("/head_cam/image_raw"));

    // Get joypad values
    joypad_command_sub_ = node_handle_.subscribe("/joy", 1, &DrivingWidget::handleJoyPadEvent, this);

    // Get current robot state
    std::string joint_state_topic;
    node_handle_private_.param("joint_state_topic", joint_state_topic, std::string("/thor_mang/joint_states"));
    robot_state_sub_ = node_handle_.subscribe(joint_state_topic, 1, &DrivingWidget::handleNewJointStateEvent, this);
    received_robot_state_ = false;

    // Setup head control publisher
    std::string head_controller_topic;
    node_handle_private_.param("head_controller_topic", head_controller_topic, std::string("/thor_mang/head_traj_controller/command"));
    head_command_pub_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(head_controller_topic, 1, false);

    // Setup driving commands
    all_stop_enabled_sub_ = node_handle_.subscribe("driving_controller/all_stop", 1, &DrivingWidget::handleAllStopEnabled, this);
    shutdown_pub_ = node_handle_.advertise<std_msgs::Empty>("driving_controller/shutdown", 1, true);
    driving_command_pub_ = node_handle_.advertise<thor_mang_driving_controller::DrivingCommand>("driving_controller/driving_command", 1, false);

    // setup user interface
    connect(ui_->spinBox_SteeringSensitivity, SIGNAL(valueChanged(double)), this, SLOT(SLO_SteeringSensitivityChanged(double)));
    connect(ui_->spinBox_HeadSensitivity, SIGNAL(valueChanged(double)), this, SLOT(SLO_HeadSensitivityChanged(double)));
    connect(ui_->spinBox_TimeFromStart, SIGNAL(valueChanged(double)), this, SLOT(SLO_TimeFromStartChanged(double)));
    connect(ui_->pushButton_ShowCameraImage, SIGNAL(toggled(bool)), this, SLOT(SLO_ShowCameraImage(bool)));
    connect(ui_->pushButton_AllStop, SIGNAL(clicked(bool)), this, SLOT(SLO_AllStopButtonChecked(bool)));
    connect(ui_->pushButton_Shutdown, SIGNAL(clicked()), this, SLOT(SLO_Shutdown()));

    timer_.start(33, this);

    // UI init
    drawWheelVisualization();
    ui_->graphicsView_Wheels->setScene(&wheel_scene_);
    updateUI();
}

DrivingWidget::~DrivingWidget()
{
    delete ui_;
}


void DrivingWidget::timerEvent(QTimerEvent *event)
{
    sendDrivingCommand();
    sendHeadCommand();

    // check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();

    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    if(event->timerId() == timer_.timerId())
        ros::spinOnce();
}

void DrivingWidget::resizeEvent(QResizeEvent *event) {
    drawWheelVisualization();
}

void DrivingWidget::updateUI() {
    double wheel_position = absolute_steering_angle_*45/540;

    // update line edits and spin boxes
    ui_->lineEdit_WheelAngle->setText( QString("%1 \260").arg(wheel_position, 3, 'f', 1));
    ui_->lineEdit_SteeringAngle->setText(QString("%1 \260").arg(steering_angle_, 3, 'f', 1));
    ui_->spinBox_SteeringSensitivity->setValue(steering_sensitivity_);
    ui_->spinBox_HeadSensitivity->setValue(head_sensitivity_);
    ui_->spinBox_TimeFromStart->setValue(time_from_start_);

    ui_->pushButton_AllStop->setChecked(all_stop_);
    ui_->label_DrivingActive->setVisible(drive_forward_);

    // set dial => map to [-180; +180]
    double limited_steering_angle = steering_angle_;
    if ( limited_steering_angle >= 360.0 )  limited_steering_angle -= 360.0;
    if ( limited_steering_angle <= -360.0 ) limited_steering_angle += 360.0;

    if ( limited_steering_angle <= -180.0 ) limited_steering_angle += 360.0;
    if ( limited_steering_angle >= 180.0 )  limited_steering_angle -= 360.0;

    ui_->dial_TargetSteeringPosition->setValue( (int)limited_steering_angle);

    drawWheelVisualization();
}

void DrivingWidget::drawWheelVisualization() {
    double wheel_angle = absolute_steering_angle_*45/540;
    double total_width = ui_->graphicsView_Wheels->visibleRegion().boundingRect().width();
    double total_height = ui_->graphicsView_Wheels->visibleRegion().boundingRect().height();
    double car_length = std::min(total_width, total_height)-120;
    double car_width = car_length / 2.0;
    double wheel_width = car_width / 4.0;
    double wheel_length = wheel_width*3.0;

    QTransform base_wheel_transform(1.0, 0.0, 0.0, 1.0, -wheel_width/2.0, -wheel_length/2.0);

    wheel_scene_.clear();
    wheel_scene_.addRect(-car_width/2.0, -car_length/2.0, car_width, car_length );

    // back wheels
    QGraphicsRectItem *wheel = wheel_scene_.addRect(-car_width/2.0, car_length/2.0, wheel_width, wheel_length, QPen(), QBrush(Qt::SolidPattern));
    wheel->setTransform(base_wheel_transform);

    wheel = wheel_scene_.addRect(car_width/2.0, car_length/2.0, wheel_width, wheel_length, QPen(), QBrush(Qt::SolidPattern));
    wheel->setTransform(base_wheel_transform);

    // front wheels
    QTransform steering_transform;
    steering_transform.rotate(wheel_angle);

    QTransform position_transform(1.0, 0.0, 0.0, 1.0, -car_width/2.0, -car_length/2.0);
    wheel = wheel_scene_.addRect(-wheel_width/2.0, -wheel_length/2.0, wheel_width, wheel_length, QPen(), QBrush(Qt::SolidPattern));
    wheel->setTransform(steering_transform*position_transform, true);

    position_transform = QTransform(1.0, 0.0, 0.0, 1.0, car_width/2.0, -car_length/2.0);
    wheel = wheel_scene_.addRect(-wheel_width/2.0, -wheel_length/2.0, wheel_width, wheel_length, QPen(), QBrush(Qt::SolidPattern));
    wheel->setTransform(steering_transform*position_transform, true);

    ui_->graphicsView_Wheels->centerOn(0.0, 0.0);
}

void DrivingWidget::handleNewCameraImage(sensor_msgs::ImageConstPtr msg) {
    QImage img(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(img.scaled( ui_->label_CameraImage->width()-8, ui_->label_CameraImage->height()-8, Qt::KeepAspectRatio));
    ui_->label_CameraImage->setPixmap(pixmap);
}

void DrivingWidget::SLO_SteeringSensitivityChanged(double sensitivity) {
    steering_sensitivity_ = sensitivity;
}

void DrivingWidget::SLO_HeadSensitivityChanged(double sensitivity) {
    head_sensitivity_ = sensitivity;
}

void DrivingWidget::SLO_TimeFromStartChanged(double time_from_start) {
    time_from_start_ = time_from_start;
}

void DrivingWidget::SLO_ShowCameraImage(bool show) {
    if ( show ) {
        camera_image_sub_ = node_handle_.subscribe(camera_topic_, 1, &DrivingWidget::handleNewCameraImage, this);
    }
    else {
        camera_image_sub_.shutdown();
        ui_->label_CameraImage->setText("No Camera Image");
    }
}

void DrivingWidget::SLO_AllStopButtonChecked(bool active) {
    all_stop_ = active;
    updateUI();
}

void DrivingWidget::SLO_Shutdown() {
    shutdown_pub_.publish(std_msgs::Empty());
    ros::spinOnce();
    this->close();
}


void DrivingWidget::handleJoyPadEvent(sensor_msgs::JoyConstPtr msg) {
    if ( !received_robot_state_ ) {
        ROS_ERROR_THROTTLE(2, "No robot state received => No Update");
        return;
    }

    handleSteeringCommand(msg->axes[DrivingWidget::STEERING]);
    handleHeadCommand(msg->axes[DrivingWidget::HEAD_TILT], msg->axes[DrivingWidget::HEAD_PAN]);

    drive_forward_ = msg->buttons[DrivingWidget::FORWARD];

    if ( msg->buttons[DrivingWidget::ALL_STOP] )
        all_stop_ = !all_stop_;

    if ( msg->buttons[DrivingWidget::STEERING_SENSITIVITY_PLUS] ) {
        steering_sensitivity_ += 0.1;
    }

    if ( msg->buttons[DrivingWidget::STEERING_SENSITIVITY_MINUS] ) {
        steering_sensitivity_ -= 0.1;
    }

    if ( msg->buttons[DrivingWidget::HEAD_SENSITIVITY_PLUS] ) {
        head_sensitivity_ += 0.1;
    }

    if ( msg->buttons[DrivingWidget::HEAD_SENSITIVITY_MINUS] ) {
        head_sensitivity_ -= 0.1;
    }

    // don't allow negative sensitivities
    steering_sensitivity_ = std::max(0.0, steering_sensitivity_);
    head_sensitivity_ = std::max(0.0, head_sensitivity_);

    updateUI();
}

void DrivingWidget::handleAllStopEnabled(thor_mang_driving_controller::DrivingCommandConstPtr msg) {
    all_stop_ = msg->all_stop.data;
    drive_forward_ = msg->drive_forward.data;
    absolute_steering_angle_ = msg->absolute_steering_angle.data;
    time_from_start_ = msg->time_from_start.data;

    while ( steering_angle_ >= 360.0 )  steering_angle_ -= 360.0;
    while ( steering_angle_ < 0 )       steering_angle_ += 360.0;

    updateUI();
}

void DrivingWidget::handleNewJointStateEvent(sensor_msgs::JointStateConstPtr msg) {
    robot_joint_names_ = msg->name;
    robot_joint_positions_ = msg->position;
    received_robot_state_ = true;
}

void DrivingWidget::sendDrivingCommand() {
    calculateSteeringAngle();

    thor_mang_driving_controller::DrivingCommand driving_command_msg;
    driving_command_msg.all_stop.data = all_stop_;
    driving_command_msg.absolute_steering_angle.data = absolute_steering_angle_;
    driving_command_msg.drive_forward.data = drive_forward_;
    driving_command_msg.time_from_start.data = time_from_start_;
    driving_command_pub_.publish(driving_command_msg);
}


void DrivingWidget::sendHeadCommand() {
    if ( all_stop_ ) {
        //ROS_INFO("All-Stop active! Steering blocked!");
        return;
    }

    if ( !received_robot_state_) {
        return;
    }

    // get current angles and add control offset
    std::vector<std::string> head_joint_names = {"head_pan", "head_tilt"};
    std::vector<double> target_head_positions(2);
    for ( int i = 0; i < head_joint_names.size(); i++ ) {
        for ( int j = 0; j < robot_joint_names_.size(); j++ ) {
            if ( head_joint_names[i] == robot_joint_names_[j] ) {
                target_head_positions[i] = robot_joint_positions_[j];
                break;
            }
        }
    }
    target_head_positions[0] += head_pan_speed_;
    target_head_positions[1] += head_tilt_speed_;

    trajectory_msgs::JointTrajectory trajectory_msg = generateTrajectoryMsg(target_head_positions, head_joint_names);
    head_command_pub_.publish(trajectory_msg);
}

void DrivingWidget::handleHeadCommand(double tilt, double pan) {
    head_tilt_speed_ = head_tilt_correction_ * head_sensitivity_ * tilt;
    head_pan_speed_  = head_pan_correction_ * head_sensitivity_ * pan;
}

void DrivingWidget::handleSteeringCommand(double step) {
    steering_speed_ = steering_correction_ * steering_sensitivity_ * step;
}

void DrivingWidget::calculateSteeringAngle() {
    if ( !received_robot_state_ ) {
        ROS_ERROR_THROTTLE(2, "No robot positions received => No Update");
        return;
    }
    if ( all_stop_ ) {
        //ROS_INFO("All-Stop active! Steering blocked!");
        return;
    }
    if (absolute_steering_angle_ + steering_speed_ <= -540.0 ||
        absolute_steering_angle_ + steering_speed_ >=  540.0 ) {
        ROS_INFO("Rotation blocked: No more than 1.5 turns left / right allowed");
        return; // do not allow more than 3 turns total
    }
    absolute_steering_angle_ += steering_speed_;

    // publish current absolute angle for visualization

    steering_angle_ += steering_speed_;
    if ( steering_angle_ < 0.0 )
        steering_angle_ += 360.0;
    else if ( steering_angle_ >= 360.0)
        steering_angle_ -= 360.0;

    updateUI();
}

trajectory_msgs::JointTrajectory DrivingWidget::generateTrajectoryMsg(std::vector<double> &joint_angles, std::vector<std::string> joint_names) {
    trajectory_msgs::JointTrajectory trajectory_msg;
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions = joint_angles;
    trajectory_point.time_from_start = ros::Duration(time_from_start_);
    trajectory_msg.points.push_back(trajectory_point);
    trajectory_msg.joint_names = joint_names;

    return trajectory_msg;
}

