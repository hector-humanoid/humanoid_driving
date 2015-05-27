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
    steering_sensitivity_ = 2.5;
    head_sensitivity_ = 0.01;
    allow_head_sensitivity_change_ = false;
    allow_steering_sensitivity_change_ = false;
    ignore_steering_limits_ = false;

    // Driving control elements
    all_stop_ = true;
    current_steering_angle_ = 0.0;
    current_absolute_steering_angle_ = 0.0;
    absolute_target_steering_angle_ = 0.0;
    drive_forward_ = false;

    controller_enabled_ = false;
    head_target_pan_ = 0.0;
    head_target_tilt_ = 0.0;
    head_tilt_speed_ = 0.0;
    head_pan_speed_ = 0.0;

    connection_lost_ = false;

    setGUIEnabled(false);

    // Get camera image topic
    node_handle_private_.param("camera_topic", camera_topic_, std::string("/head_cam/image_raw"));

    // Get joypad values
    joypad_command_sub_ = node_handle_.subscribe("/joy", 1, &DrivingWidget::handleJoyPadEvent, this);

    // Get joypad ids
    node_handle_private_.param("joypad_axis_steering", joypad_ids_[AXIS_STEERING], 0);
    node_handle_private_.param("joypad_axis_head_pan", joypad_ids_[AXIS_HEAD_PAN], 2);
    node_handle_private_.param("joypad_axis_head_tilt", joypad_ids_[AXIS_HEAD_TILT], 3);
    node_handle_private_.param("joypad_axis_head_tilt_2", joypad_ids_[AXIS_HEAD_TILT_2], 4);
    node_handle_private_.param("joypad_axis_head_pan_2", joypad_ids_[AXIS_HEAD_PAN_2], 5);

    node_handle_private_.param("joypad_button_forward", joypad_ids_[BUTTON_FORWARD], 1);
    node_handle_private_.param("joypad_button_all_stop", joypad_ids_[BUTTON_ALL_STOP], 2);
    node_handle_private_.param("joypad_button_steering_sensitivity_plus", joypad_ids_[BUTTON_STEERING_SENSITIVITY_PLUS], 7);
    node_handle_private_.param("joypad_button_steering_sensitivity_minus", joypad_ids_[BUTTON_STEERING_SENSITIVITY_MINUS], 6);
    node_handle_private_.param("joypad_button_head_sensitivity_plus", joypad_ids_[BUTTON_HEAD_SENSITIVITY_PLUS], 5);
    node_handle_private_.param("joypad_button_head_sensitivity_minus", joypad_ids_[BUTTON_HEAD_SENSITIVITY_MINUS], 4);
    node_handle_private_.param("joypad_button_mode_to_default", joypad_ids_[BUTTON_HEAD_MODE_TO_DEFAULT], 11);

    node_handle_private_.param("steering_correction_factor", steering_correction_, -1.0);
    node_handle_private_.param("head_tilt_correction_factor", head_tilt_correction_, -1.0);
    node_handle_private_.param("head_pan_correction_factor", head_pan_correction_, 1.0);

    if ( joypad_ids_[BUTTON_STEERING_SENSITIVITY_PLUS] < 0 || joypad_ids_[BUTTON_STEERING_SENSITIVITY_PLUS] < 0 )
        allow_steering_sensitivity_change_ = false;
    if ( joypad_ids_[BUTTON_HEAD_SENSITIVITY_PLUS] < 0 || joypad_ids_[BUTTON_HEAD_SENSITIVITY_PLUS] < 0 )
        allow_head_sensitivity_change_ = false;

    // Setup driving commands
    all_stop_enabled_sub_ = node_handle_.subscribe("driving_controller/all_stop", 1, &DrivingWidget::handleAllStopEnabled, this);
    driving_command_pub_ = node_handle_.advertise<thor_mang_driving_controller::DrivingCommand>("driving_controller/driving_command", 1, false);

    // Enable / Disable controller, Reset
    controller_enable_pub_ = node_handle_.advertise<std_msgs::Bool>("driving_controller/controller_enable", 1, true);
    controller_enable_ack_sub_ = node_handle_.subscribe("driving_controller/controller_enable_ack", 1, &DrivingWidget::handleControllerEnableACK, this);
    controller_reset_pub_ = node_handle_.advertise<std_msgs::Empty>("driving_controller/controller_reset", 1, false);

    // Get absolute steering angle from controller
    absolute_steering_angle_sub_ = node_handle_.subscribe("driving_controller/absolute_steering_angle", 1, &DrivingWidget::handleNewAbsoluteSteeringAngle, this);

    connection_loss_sub_ = node_handle_.subscribe("driving_controller/connection_loss", 1, &DrivingWidget::handleConnectionLoss, this);

    // setup user interface
    connect(ui_->pushButton_ConfirmSteeringSensitivity, SIGNAL(clicked()), this, SLOT(SLO_SteeringSensitivityConfirmed()));
    connect(ui_->spinBox_SteeringSensitivity, SIGNAL(valueChanged(double)), this, SLOT(SLO_SteeringSensitivityChanged()));
    connect(ui_->pushButton_ConfirmHeadSensitivity, SIGNAL(clicked()), this, SLOT(SLO_HeadSensitivityConfirmed()));
    connect(ui_->spinBox_HeadSensitivity, SIGNAL(valueChanged(double)), this, SLOT(SLO_HeadSensitivityChanged()));
    connect(ui_->pushButton_ShowCameraImage, SIGNAL(toggled(bool)), this, SLOT(SLO_ShowCameraImage(bool)));
    connect(ui_->pushButton_AllStop, SIGNAL(clicked(bool)), this, SLOT(SLO_AllStopButtonChecked(bool)));
    connect(ui_->pushButton_ToggleDrivingMode, SIGNAL(clicked()), this, SLOT(SLO_ToggleDrivingMode()));
    connect(ui_->pushButton_OverrideLimits, SIGNAL(toggled(bool)), this, SLOT(SLO_OverrideLimits(bool)));

    timer_.start(33, this);

    // UI init
    drawWheelVisualization();
    ui_->graphicsView_Wheels->setScene(&wheel_scene_);
    updateUI(true, true);
}

DrivingWidget::~DrivingWidget()
{
    delete ui_;
}


void DrivingWidget::timerEvent(QTimerEvent *event)
{
    sendDrivingCommand();

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

void DrivingWidget::updateUI(bool update_steering_sensitivity, bool update_head_sensitivity) {
    double wheel_position = current_absolute_steering_angle_*45/540;

    // update line edits and spin boxes
    ui_->lineEdit_WheelAngle->setText( QString("%1 \260").arg(wheel_position, 3, 'f', 1));
    ui_->lineEdit_SteeringAngle->setText(QString("%1 \260").arg(current_absolute_steering_angle_, 3, 'f', 1));

    if ( update_steering_sensitivity ) {
        ui_->spinBox_SteeringSensitivity->blockSignals(true);
        ui_->spinBox_SteeringSensitivity->setValue(steering_sensitivity_);
        ui_->spinBox_SteeringSensitivity->blockSignals(false);
    }

    if ( update_head_sensitivity ) {
        ui_->spinBox_HeadSensitivity->blockSignals(true);
        ui_->spinBox_HeadSensitivity->setValue(head_sensitivity_);
        ui_->spinBox_HeadSensitivity->blockSignals(false);
    }

    ui_->pushButton_AllStop->setChecked(all_stop_);
    ui_->label_DrivingActive->setVisible(drive_forward_);

    // set current steering angle => map to [-180; +180]
    double limited_steering_angle = current_steering_angle_;
    while ( limited_steering_angle >= 360.0 )  limited_steering_angle -= 360.0;
    while ( limited_steering_angle <= -360.0 ) limited_steering_angle += 360.0;

    if ( limited_steering_angle <= -180.0 ) limited_steering_angle += 360.0;
    if ( limited_steering_angle >= 180.0 )  limited_steering_angle -= 360.0;

    ui_->dial_CurrentSteeringPosition->setValue( (int)limited_steering_angle);

    // set target steering angle => map to [-180; +180]
    double limited_target_steering_angle = absolute_target_steering_angle_;
    while ( limited_target_steering_angle >= 360.0 )  limited_target_steering_angle -= 360.0;
    while ( limited_target_steering_angle <= -360.0 ) limited_target_steering_angle += 360.0;

    if ( limited_target_steering_angle <= -180.0 ) limited_target_steering_angle += 360.0;
    if ( limited_target_steering_angle >= 180.0 )  limited_target_steering_angle -= 360.0;

    ui_->dial_TargetSteeringPosition->setValue( (int)limited_target_steering_angle);

    drawWheelVisualization();

    ui_->slider_HeadPan->setValue(head_target_pan_*100.0);
    ui_->slider_HeadTilt->setValue(head_target_tilt_*100.0);


}

void DrivingWidget::drawWheelVisualization() {
    double wheel_angle = current_absolute_steering_angle_*45/540;
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
    QColor wheel_color = Qt::black;
    if ( fabs(wheel_angle) >= 44.8 ) {
        wheel_color = Qt::red;
    }

    QTransform steering_transform;
    steering_transform.rotate(wheel_angle);

    QTransform position_transform(1.0, 0.0, 0.0, 1.0, -car_width/2.0, -car_length/2.0);
    wheel = wheel_scene_.addRect(-wheel_width/2.0, -wheel_length/2.0, wheel_width, wheel_length, QPen(), QBrush(wheel_color, Qt::SolidPattern));
    wheel->setTransform(steering_transform*position_transform, true);

    position_transform = QTransform(1.0, 0.0, 0.0, 1.0, car_width/2.0, -car_length/2.0);
    wheel = wheel_scene_.addRect(-wheel_width/2.0, -wheel_length/2.0, wheel_width, wheel_length, QPen(), QBrush(wheel_color, Qt::SolidPattern));
    wheel->setTransform(steering_transform*position_transform, true);

    // show all stop
    // Draw all stop sign in img

    if ( all_stop_ ) {
        /*
        QPainter painter(&img); // sorry i forgot the "&"

        painter.fillRect(70, 70, img.width()-140, img.height()-140, Qt::red);

        painter.setPen(Qt::white);
        painter.setFont(QFont("Arial", 48));
        painter.drawText(img.rect(), Qt::AlignCenter, "All Stop!");*/

        QGraphicsRectItem *background_item = wheel_scene_.addRect(-car_width/2.0+0.3, 0.0-car_length/12.0, car_width-0.6, car_length/6.0, QPen(Qt::red), QBrush(Qt::red));

        QFont textFont;
        textFont.setBold(true);
        textFont.setPixelSize(background_item->boundingRect().height()-0.5);
        QGraphicsTextItem *allStopTextItem = wheel_scene_.addText("Stop!", textFont);
        allStopTextItem->setDefaultTextColor(Qt::white);
        allStopTextItem->moveBy( -allStopTextItem->boundingRect().width()/2.0, -allStopTextItem->boundingRect().height()/2.0);
    }
    else if ( connection_lost_ ) {
        /*
        QPainter painter(&img); // sorry i forgot the "&"

        painter.fillRect(70, 70, img.width()-140, img.height()-140, Qt::red);

        painter.setPen(Qt::white);
        painter.setFont(QFont("Arial", 48));
        painter.drawText(img.rect(), Qt::AlignCenter, "All Stop!");*/

        QGraphicsRectItem *background_item = wheel_scene_.addRect(-car_width/2.0+0.3, 0.0-car_length/12.0, car_width-0.6, car_length/6.0, QPen(Qt::yellow), QBrush(Qt::yellow));

        QFont textFont;
        textFont.setBold(true);
        textFont.setPixelSize(background_item->boundingRect().height()-0.5);
        QGraphicsTextItem *allStopTextItem = wheel_scene_.addText("No Signal!", textFont);
        allStopTextItem->setDefaultTextColor(Qt::black);
        allStopTextItem->moveBy( -allStopTextItem->boundingRect().width()/2.0, -allStopTextItem->boundingRect().height()/2.0);
    }



    ui_->graphicsView_Wheels->centerOn(0.0, 0.0);
}

void DrivingWidget::setGUIEnabled(bool enable) {
    ui_->pushButton_AllStop->setEnabled(enable);
    ui_->pushButton_ConfirmHeadSensitivity->setEnabled(enable);
    ui_->pushButton_ConfirmSteeringSensitivity->setEnabled(enable);
    //ui_->pushButton_ShowCameraImage->setEnabled(enable);
    ui_->pushButton_OverrideLimits->setEnabled(enable);
    ui_->spinBox_HeadSensitivity->setEnabled(enable);
    ui_->spinBox_SteeringSensitivity->setEnabled(enable);
    ui_->lineEdit_SteeringAngle->setEnabled(enable);
    ui_->lineEdit_WheelAngle->setEnabled(enable);
}

void DrivingWidget::handleNewCameraImage(sensor_msgs::ImageConstPtr msg) {
    QImage img(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
    img = img.rgbSwapped();
    QPixmap pixmap = QPixmap::fromImage(img.scaled( ui_->label_CameraImage->width()-8, ui_->label_CameraImage->height()-8, Qt::KeepAspectRatio));
    ui_->label_CameraImage->setPixmap(pixmap);
}

void DrivingWidget::handleNewAbsoluteSteeringAngle(std_msgs::Float64ConstPtr msg) {
    current_absolute_steering_angle_ = msg->data;

    if ( all_stop_ ) {
        absolute_target_steering_angle_ = current_absolute_steering_angle_;
    }

    current_steering_angle_ = current_absolute_steering_angle_;
    while ( current_steering_angle_ >= 360.0 )  current_steering_angle_ -= 360.0;
    while ( current_steering_angle_ < 0 )       current_steering_angle_ += 360.0;

    updateUI();
}

void DrivingWidget::handleConnectionLoss(std_msgs::BoolConstPtr msg) {
    connection_lost_ = msg->data;
}

void DrivingWidget::SLO_SteeringSensitivityChanged() {
    ui_->pushButton_ConfirmSteeringSensitivity->setStyleSheet("color:#FF0000");
}

void DrivingWidget::SLO_SteeringSensitivityConfirmed() {
    steering_sensitivity_ = ui_->spinBox_SteeringSensitivity->value();
    ui_->pushButton_ConfirmSteeringSensitivity->setStyleSheet("color:#000000");
}

void DrivingWidget::SLO_HeadSensitivityChanged() {
    ui_->pushButton_ConfirmHeadSensitivity->setStyleSheet("color:#FF0000");
}

void DrivingWidget::SLO_HeadSensitivityConfirmed() {
    head_sensitivity_ = ui_->spinBox_HeadSensitivity->value();
    ui_->pushButton_ConfirmHeadSensitivity->setStyleSheet("color:#000000");
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

void DrivingWidget::SLO_ToggleDrivingMode() {
    ui_->label_WaitingForACK->setText("Waiting for ACK...");

    std_msgs::Bool enable_msg;
    enable_msg.data = !controller_enabled_;
    controller_enable_pub_.publish(enable_msg);

    setGUIEnabled(false);
}

void DrivingWidget::SLO_OverrideLimits(bool override) {
    if ( override ) {
        ui_->pushButton_OverrideLimits->setStyleSheet("color:#FF0000");
    }

    ignore_steering_limits_ = override;
}

void DrivingWidget::handleJoyPadEvent(sensor_msgs::JoyConstPtr msg) {
    if ( controller_enabled_ == false )
        return;

    handleSteeringCommand(msg->axes[ joypad_ids_[DrivingWidget::AXIS_STEERING] ]);
    handleHeadCommand(msg->axes[ joypad_ids_[DrivingWidget::AXIS_HEAD_TILT_2] ], msg->axes[ joypad_ids_[DrivingWidget::AXIS_HEAD_PAN_2]]);
    handleHeadCommand(msg->axes[ joypad_ids_[DrivingWidget::AXIS_HEAD_TILT] ], msg->axes[ joypad_ids_[DrivingWidget::AXIS_HEAD_PAN]]);

    drive_forward_ = msg->buttons[ joypad_ids_[DrivingWidget::BUTTON_FORWARD] ];

    if ( msg->buttons[ joypad_ids_[ DrivingWidget::BUTTON_ALL_STOP] ] )
        all_stop_ = !all_stop_;

    bool update_steering_sensitivity = false;
    if ( allow_steering_sensitivity_change_ ) {
        if ( msg->buttons[ joypad_ids_[DrivingWidget::BUTTON_STEERING_SENSITIVITY_PLUS]] ) {
            steering_sensitivity_ += 0.1;
            update_steering_sensitivity = true;
        }

        if ( msg->buttons[ joypad_ids_[DrivingWidget::BUTTON_STEERING_SENSITIVITY_MINUS]] ) {
            steering_sensitivity_ -= 0.1;
            update_steering_sensitivity = true;
        }
    }

    bool update_head_sensitivity = false;
    if ( allow_head_sensitivity_change_ ) {
        if ( msg->buttons[ joypad_ids_[DrivingWidget::BUTTON_HEAD_SENSITIVITY_PLUS]] ) {
            head_sensitivity_ += 0.1;
            update_head_sensitivity = true;
        }

        if ( msg->buttons[ joypad_ids_[DrivingWidget::BUTTON_HEAD_SENSITIVITY_MINUS]] ) {
            head_sensitivity_ -= 0.1;
            update_head_sensitivity = true;
        }
    }

    if ( msg->buttons[ joypad_ids_[DrivingWidget::BUTTON_HEAD_MODE_TO_DEFAULT]] ) {
        head_target_pan_ = 0.0;
        head_target_tilt_ = 0.0;
    }

    // don't allow negative sensitivities
    steering_sensitivity_ = std::max(0.0, steering_sensitivity_);
    head_sensitivity_ = std::max(0.0, head_sensitivity_);

    updateUI(update_steering_sensitivity, update_head_sensitivity);
}

void DrivingWidget::handleAllStopEnabled(thor_mang_driving_controller::DrivingCommandConstPtr msg) {
    all_stop_ = msg->all_stop;
    drive_forward_ = msg->drive_forward;

    if ( all_stop_ ) {
        absolute_target_steering_angle_ = current_absolute_steering_angle_;
    }

    connection_lost_ = true;

    updateUI();
}

void DrivingWidget::handleControllerEnableACK(std_msgs::BoolConstPtr msg) {
    if ( msg->data ) { // controller enabled
        ui_->pushButton_ToggleDrivingMode->setText("Disable Driving Mode");
        ui_->label_WaitingForACK->setText("Driving Mode Active!");
        controller_enabled_ = true;
    }
    else { // controller disabled
        ui_->pushButton_ToggleDrivingMode->setText("Enable Driving Mode");
        ui_->label_WaitingForACK->setText("Driving Mode Disabled!");
        ui_->pushButton_ShowCameraImage->setChecked(false);
        controller_enabled_ = false;
    }

    setGUIEnabled(controller_enabled_);
}

void DrivingWidget::sendDrivingCommand() {
    if ( controller_enabled_ == false )
        return;

    checkSteeringLimits();

    absolute_target_steering_angle_ += steering_speed_;
    head_target_pan_ += head_pan_speed_;
    head_target_tilt_ += head_tilt_speed_;

    thor_mang_driving_controller::DrivingCommand driving_command_msg;
    driving_command_msg.all_stop = all_stop_;
    driving_command_msg.absolute_target_steering_angle = absolute_target_steering_angle_;
    driving_command_msg.drive_forward = drive_forward_;
    driving_command_msg.absolute_head_pan = head_target_pan_;
    driving_command_msg.absolute_head_tilt = head_target_tilt_;
    driving_command_pub_.publish(driving_command_msg);
}

void DrivingWidget::handleHeadCommand(double tilt, double pan) {
    head_tilt_speed_ = head_tilt_correction_ * head_sensitivity_ * tilt;
    head_pan_speed_ = head_pan_correction_ * head_sensitivity_ * pan;
}

void DrivingWidget::handleSteeringCommand(double step) {
    steering_speed_ = steering_correction_ * steering_sensitivity_ * step;
}

void DrivingWidget::checkSteeringLimits() {
    if ( head_target_pan_ + head_pan_speed_ <= -1.57 ||
         head_target_pan_ + head_pan_speed_ >= 1.57 ) {
        head_pan_speed_ = 0.0;
    }

    if ( head_target_tilt_ + head_tilt_speed_ <= -1.32 ||
         head_target_tilt_ + head_tilt_speed_ >= 0.79 ) {
        head_tilt_speed_ = 0.0;
    }

    if (ignore_steering_limits_ == false ) {
        if (current_absolute_steering_angle_ + steering_speed_ < -540.0) {
            absolute_target_steering_angle_ = -540.0;
            steering_speed_ = 0.0;
        }
        else if ( current_absolute_steering_angle_ + steering_speed_ >  540.0 ) {
            absolute_target_steering_angle_ = 540.0;
            steering_speed_ = 0.0;
        }
    }
}

trajectory_msgs::JointTrajectory DrivingWidget::generateTrajectoryMsg(std::vector<double> &joint_angles, std::vector<std::string> joint_names) {
    trajectory_msgs::JointTrajectory trajectory_msg;
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    trajectory_point.positions = joint_angles;
    trajectory_point.time_from_start = ros::Duration(0.1);
    trajectory_msg.points.push_back(trajectory_point);
    trajectory_msg.joint_names = joint_names;

    return trajectory_msg;
}

