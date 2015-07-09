#include <humanoid_driving_widget/driving_widget.h>
#include "ui_driving_widget.h"

#include <ros/ros.h>

#include <QGraphicsItem>

namespace humanoid_driving_widget {

DrivingWidget::DrivingWidget(QWidget *parent) :
    QMainWindow(parent),
    ui_(new Ui::DrivingWidget),
    node_handle_private_("~")
{
    ui_->setupUi(this);

    // steering parameters    
    steering_speed_ = 0.0;    
    ignore_steering_limits_ = false;

    // Driving control elements
    current_state_.steering_wheel_angle = 0.0;
    current_state_.head_pan = 0.0;
    current_state_.head_tilt = 0.0;
    current_state_.connection_loss = false;
    current_state_.driving_counter = 0;

    target_command_.all_hold = true;
    target_command_.steering_wheel_angle = 0.0;
    target_command_.drive_forward = false;
    target_command_.head_pan = 0.0;
    target_command_.head_tilt = 0.0;

    controller_enabled_ = false;

    head_tilt_speed_ = 0.0;
    head_pan_speed_ = 0.0;

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
    node_handle_private_.param("joypad_button_all_hold", joypad_ids_[BUTTON_ALL_HOLD], 2);
    node_handle_private_.param("joypad_button_steering_sensitivity_plus", joypad_ids_[BUTTON_STEERING_SENSITIVITY_PLUS], 7);
    node_handle_private_.param("joypad_button_steering_sensitivity_minus", joypad_ids_[BUTTON_STEERING_SENSITIVITY_MINUS], 6);
    node_handle_private_.param("joypad_button_head_sensitivity_plus", joypad_ids_[BUTTON_HEAD_SENSITIVITY_PLUS], 5);
    node_handle_private_.param("joypad_button_head_sensitivity_minus", joypad_ids_[BUTTON_HEAD_SENSITIVITY_MINUS], 4);
    node_handle_private_.param("joypad_button_head_to_default", joypad_ids_[BUTTON_HEAD_TO_DEFAULT], 11);
    node_handle_private_.param("joypad_button_steering_to_default", joypad_ids_[BUTTON_STEERING_TO_DEFAULT], 10);

    node_handle_private_.param("steering_sensitivity", steering_sensitivity_, -0.08);
    node_handle_private_.param("head_tilt_sensitivity", head_tilt_sensitivity_, -0.05);
    node_handle_private_.param("head_pan_sensitivity", head_pan_sensitivity_, 0.05);

    // Setup driving commands
    all_hold_enabled_sub_ = node_handle_.subscribe("driving_controller/all_hold", 1, &DrivingWidget::handleAllHoldEnabled, this);
    driving_command_pub_ = node_handle_.advertise<humanoid_driving_controller::DrivingCommand>("driving_controller/driving_command", 1, false);

    // Enable / Disable controller, Reset
    controller_enable_pub_ = node_handle_.advertise<std_msgs::Bool>("driving_controller/controller_enable", 1, true);
    controller_enable_ack_sub_ = node_handle_.subscribe("driving_controller/controller_enable_ack", 1, &DrivingWidget::handleControllerEnableACK, this);

    // Get absolute steering angle from controller
    driving_state_sub_ = node_handle_.subscribe("driving_controller/driving_state", 1, &DrivingWidget::handleNewDrivingState, this);

    // Publish wheel angle
    wheel_angle_pub_ = node_handle_.advertise<std_msgs::Float64>("driving_widget/wheel_angle_rad", 1, true);

    // setup user interface
    connect(ui_->pushButton_ConfirmSteeringSensitivity, SIGNAL(clicked()), this, SLOT(SLO_SteeringSensitivityConfirmed()));
    connect(ui_->spinBox_SteeringSensitivity, SIGNAL(valueChanged(double)), this, SLOT(SLO_SteeringSensitivityChanged()));
    connect(ui_->pushButton_ConfirmHeadTiltSensitivity, SIGNAL(clicked()), this, SLOT(SLO_HeadTiltSensitivityConfirmed()));
    connect(ui_->spinBox_HeadTiltSensitivity, SIGNAL(valueChanged(double)), this, SLOT(SLO_HeadTiltSensitivityChanged()));
    connect(ui_->pushButton_ConfirmHeadPanSensitivity, SIGNAL(clicked()), this, SLOT(SLO_HeadPanSensitivityConfirmed()));
    connect(ui_->spinBox_HeadPanSensitivity, SIGNAL(valueChanged(double)), this, SLOT(SLO_HeadPanSensitivityChanged()));
    connect(ui_->pushButton_ShowCameraImage, SIGNAL(toggled(bool)), this, SLOT(SLO_ShowCameraImage(bool)));
    connect(ui_->pushButton_AllHold, SIGNAL(clicked(bool)), this, SLOT(SLO_AllHoldButtonChecked(bool)));
    connect(ui_->pushButton_ToggleDrivingMode, SIGNAL(clicked()), this, SLOT(SLO_ToggleDrivingMode()));
    connect(ui_->pushButton_OverrideLimits, SIGNAL(toggled(bool)), this, SLOT(SLO_OverrideLimits(bool)));

    timer_.start(66, this);

    // UI init
    updateUI(true, true);
    ui_->graphicsView_Wheels->setScene(&wheel_scene_);
}

DrivingWidget::~DrivingWidget()
{
    delete ui_;
}

void DrivingWidget::handleJoyPadEvent(sensor_msgs::JoyConstPtr msg) {
    if ( controller_enabled_ == false )
        return;

    handleSteeringCommand(msg->axes[ joypad_ids_[DrivingWidget::AXIS_STEERING] ]);
    handleHeadCommand(msg->axes[ joypad_ids_[DrivingWidget::AXIS_HEAD_TILT_2] ], msg->axes[ joypad_ids_[DrivingWidget::AXIS_HEAD_PAN_2]]);
    handleHeadCommand(msg->axes[ joypad_ids_[DrivingWidget::AXIS_HEAD_TILT] ], msg->axes[ joypad_ids_[DrivingWidget::AXIS_HEAD_PAN]]);

    target_command_.drive_forward = msg->buttons[ joypad_ids_[DrivingWidget::BUTTON_FORWARD] ];

    if ( msg->buttons[ joypad_ids_[ DrivingWidget::BUTTON_ALL_HOLD] ] )
        target_command_.all_hold = !target_command_.all_hold;

    bool update_steering_sensitivity = false;
    if ( joypad_ids_[DrivingWidget::BUTTON_STEERING_SENSITIVITY_PLUS] >= 0 ) {
        if ( msg->buttons[ joypad_ids_[DrivingWidget::BUTTON_STEERING_SENSITIVITY_PLUS]] ) {
            steering_sensitivity_ += SteeringSensitivityStep;
            update_steering_sensitivity = true;
        }

        if ( msg->buttons[ joypad_ids_[DrivingWidget::BUTTON_STEERING_SENSITIVITY_MINUS]] ) {
            steering_sensitivity_ -= SteeringSensitivityStep;
            update_steering_sensitivity = true;
        }
    }

    bool update_head_sensitivity = false;
    if ( joypad_ids_[DrivingWidget::BUTTON_HEAD_SENSITIVITY_PLUS] >= 0 ) {
        if ( msg->buttons[ joypad_ids_[DrivingWidget::BUTTON_HEAD_SENSITIVITY_PLUS]] ) {
            head_tilt_sensitivity_ += HeadSensitivityStep;
            head_pan_sensitivity_ += HeadSensitivityStep;
            update_head_sensitivity = true;
        }

        if ( msg->buttons[ joypad_ids_[DrivingWidget::BUTTON_HEAD_SENSITIVITY_MINUS]] ) {
            head_tilt_sensitivity_ -= HeadSensitivityStep;
            head_pan_sensitivity_ -= HeadSensitivityStep;
            update_head_sensitivity = true;
        }
    }

    if ( msg->buttons[ joypad_ids_[DrivingWidget::BUTTON_HEAD_TO_DEFAULT]] ) {
        target_command_.head_pan = 0.0;
        target_command_.head_tilt = 0.0;
    }

    if ( msg->buttons[ joypad_ids_[DrivingWidget::BUTTON_STEERING_TO_DEFAULT]] ) {
        target_command_.steering_wheel_angle = 0.0;
    }

    updateUI(update_steering_sensitivity, update_head_sensitivity);
}

void DrivingWidget::handleNewCameraImage(sensor_msgs::ImageConstPtr msg) {
    QImage img(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(img.scaled( ui_->label_CameraImage->width()-8, ui_->label_CameraImage->height()-8, Qt::KeepAspectRatio));
    ui_->label_CameraImage->setPixmap(pixmap);
}

void DrivingWidget::handleAllHoldEnabled(humanoid_driving_controller::DrivingCommandConstPtr msg) {
    target_command_.all_hold = msg->all_hold;
    target_command_.drive_forward = msg->drive_forward;

    if ( target_command_.all_hold ) {
        target_command_.steering_wheel_angle = current_state_.steering_wheel_angle;
        target_command_.head_pan = msg->head_pan;
        target_command_.head_tilt = msg->head_tilt;
    }
}

void DrivingWidget::handleControllerEnableACK(std_msgs::BoolConstPtr msg) {
    if ( msg->data ) { // controller enabled
        ui_->pushButton_ToggleDrivingMode->setText(tr("Disable Driving Mode"));
        ui_->label_WaitingForACK->setText(tr("Driving Mode Active!"));
        controller_enabled_ = true;
    }
    else { // controller disabled
        ui_->pushButton_ToggleDrivingMode->setText(tr("Enable Driving Mode"));
        ui_->label_WaitingForACK->setText(tr("Driving Mode Disabled!"));
        ui_->pushButton_ShowCameraImage->setChecked(false);
        controller_enabled_ = false;
    }

    setGUIEnabled(controller_enabled_);
}

void DrivingWidget::handleNewDrivingState(humanoid_driving_controller::DrivingStateConstPtr msg) {
    current_state_ = *msg;

    // Publish wheel angle in rad
    double wheel_angle_rad = current_state_.steering_wheel_angle*WheelAnglePerSteeringRotation / MaxSteeringWheelRotation;

    std_msgs::Float64 wheel_angle_msg;
    wheel_angle_msg.data = -wheel_angle_rad;
    wheel_angle_pub_.publish(wheel_angle_msg);
}


void DrivingWidget::timerEvent(QTimerEvent *event)
{
    sendDrivingCommand();

    updateUI();

    // check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();

    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    if(event->timerId() == timer_.timerId())
        ros::spinOnce();
}


void DrivingWidget::SLO_SteeringSensitivityChanged() {
    ui_->pushButton_ConfirmSteeringSensitivity->setStyleSheet("color:#FF0000");
}

void DrivingWidget::SLO_SteeringSensitivityConfirmed() {
    steering_sensitivity_ = ui_->spinBox_SteeringSensitivity->value();
    ui_->pushButton_ConfirmSteeringSensitivity->setStyleSheet("color:#000000");
}

void DrivingWidget::SLO_HeadTiltSensitivityChanged() {
    ui_->pushButton_ConfirmHeadTiltSensitivity->setStyleSheet("color:#FF0000");
}

void DrivingWidget::SLO_HeadTiltSensitivityConfirmed() {
    head_tilt_sensitivity_ = ui_->spinBox_HeadTiltSensitivity->value();
    ui_->pushButton_ConfirmHeadTiltSensitivity->setStyleSheet("color:#000000");
}

void DrivingWidget::SLO_HeadPanSensitivityChanged() {
    ui_->pushButton_ConfirmHeadPanSensitivity->setStyleSheet("color:#FF0000");
}

void DrivingWidget::SLO_HeadPanSensitivityConfirmed() {
    head_pan_sensitivity_ = ui_->spinBox_HeadPanSensitivity->value();
    ui_->pushButton_ConfirmHeadPanSensitivity->setStyleSheet("color:#000000");
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

void DrivingWidget::SLO_AllHoldButtonChecked(bool active) {
    target_command_.all_hold = active;
}

void DrivingWidget::SLO_ToggleDrivingMode() {
    ui_->label_WaitingForACK->setText(tr("Waiting for ACK..."));

    std_msgs::Bool enable_msg;
    enable_msg.data = !controller_enabled_;
    controller_enable_pub_.publish(enable_msg);

    setGUIEnabled(false);
}

void DrivingWidget::SLO_OverrideLimits(bool override) {
    if ( override ) {
        ui_->pushButton_OverrideLimits->setStyleSheet("color:#FF0000");
    }
    else {
        ui_->pushButton_OverrideLimits->setStyleSheet("color:000000");
    }

    ignore_steering_limits_ = override;
}



void DrivingWidget::sendDrivingCommand() {
    if ( controller_enabled_ == false )
        return;

    checkSteeringLimits();

    if( target_command_.all_hold ) { // all hold active => target state = current state
        target_command_.steering_wheel_angle = current_state_.steering_wheel_angle;
        target_command_.head_pan = current_state_.head_pan;
        target_command_.head_tilt = current_state_.head_tilt;
    }
    else { // update target state from current speeds
        target_command_.steering_wheel_angle += steering_speed_;
        target_command_.head_pan += head_pan_speed_;
        target_command_.head_tilt += head_tilt_speed_;
    }

    driving_command_pub_.publish(target_command_);
}

void DrivingWidget::checkSteeringLimits() {
    if ( target_command_.head_pan + head_pan_speed_ <= -1.57 ||
         target_command_.head_pan + head_pan_speed_ >= 1.57 ) {
        head_pan_speed_ = 0.0;
    }

    if ( target_command_.head_tilt + head_tilt_speed_ <= -1.32 ||
         target_command_.head_tilt + head_tilt_speed_ >= 0.79 ) {
        head_tilt_speed_ = 0.0;
    }

    if (ignore_steering_limits_ == false ) {
        if (current_state_.steering_wheel_angle + steering_speed_ < -MaxSteeringWheelRotation ) {
            target_command_.steering_wheel_angle = -MaxSteeringWheelRotation;
            steering_speed_ = 0.0;
        }
        else if ( current_state_.steering_wheel_angle + steering_speed_ > MaxSteeringWheelRotation ) {
            target_command_.steering_wheel_angle = MaxSteeringWheelRotation;
            steering_speed_ = 0.0;
        }
    }
}

void DrivingWidget::updateUI(bool update_steering_sensitivity, bool update_head_sensitivity) {
    double steering_wheel_angle_deg = rad2deg(current_state_.steering_wheel_angle);
    double wheel_angle_deg = rad2deg(current_state_.steering_wheel_angle*WheelAnglePerSteeringRotation/MaxSteeringWheelRotation);

    // update line edits and spin boxes
    ui_->lineEdit_WheelAngle->setText( QString("%1 \260").arg(wheel_angle_deg, 3, 'f', 1));
    ui_->lineEdit_SteeringAngle->setText(QString("%1 \260").arg(steering_wheel_angle_deg, 3, 'f', 1));

    if ( update_steering_sensitivity ) {
        ui_->spinBox_SteeringSensitivity->blockSignals(true);
        ui_->spinBox_SteeringSensitivity->setValue(steering_sensitivity_);
        ui_->spinBox_SteeringSensitivity->blockSignals(false);
    }

    if ( update_head_sensitivity ) {
        ui_->spinBox_HeadTiltSensitivity->blockSignals(true);
        ui_->spinBox_HeadTiltSensitivity->setValue(head_tilt_sensitivity_);
        ui_->spinBox_HeadTiltSensitivity->blockSignals(false);

        ui_->spinBox_HeadPanSensitivity->blockSignals(true);
        ui_->spinBox_HeadPanSensitivity->setValue(head_pan_sensitivity_);
        ui_->spinBox_HeadPanSensitivity->blockSignals(false);
    }

    ui_->pushButton_AllHold->setChecked(target_command_.all_hold);
    if ( target_command_.drive_forward )
        ui_->label_DrivingActive->setStyleSheet("background-color:#00C000;color:#FFFFFF;");
    else
        ui_->label_DrivingActive->setStyleSheet("");

    // set current steering angle => map to [-180; +180]
    double limited_steering_wheel_angle_deg = rad2deg(current_state_.steering_wheel_angle);
    while ( limited_steering_wheel_angle_deg >= 360.0 )  limited_steering_wheel_angle_deg -= 360.0;
    while ( limited_steering_wheel_angle_deg <= -360.0 ) limited_steering_wheel_angle_deg += 360.0;

    if ( limited_steering_wheel_angle_deg <= -180.0 ) limited_steering_wheel_angle_deg += 360.0;
    if ( limited_steering_wheel_angle_deg >= 180.0 )  limited_steering_wheel_angle_deg -= 360.0;

    ui_->dial_CurrentSteeringPosition->setValue( (int)limited_steering_wheel_angle_deg);

    // set target steering angle => map to [-180; +180]
    double limited_target_steering_wheel_angle_deg = rad2deg(target_command_.steering_wheel_angle);
    while ( limited_target_steering_wheel_angle_deg >= 360.0 )  limited_target_steering_wheel_angle_deg -= 360.0;
    while ( limited_target_steering_wheel_angle_deg <= -360.0 ) limited_target_steering_wheel_angle_deg += 360.0;

    if ( limited_target_steering_wheel_angle_deg <= -180.0 ) limited_target_steering_wheel_angle_deg += 360.0;
    if ( limited_target_steering_wheel_angle_deg >= 180.0 )  limited_target_steering_wheel_angle_deg -= 360.0;

    ui_->dial_TargetSteeringPosition->setValue( (int)limited_target_steering_wheel_angle_deg);

    drawWheelVisualization();

    ui_->slider_HeadPan->setValue(target_command_.head_pan*100.0);
    ui_->slider_HeadTilt->setValue(target_command_.head_tilt*100.0);
    ui_->slider_CurrentHeadPan->setValue(current_state_.head_pan*100.0);
    ui_->slider_CurrentHeadTilt->setValue(current_state_.head_tilt*100.0);

    ui_->lineEdit_DrivingCounter->setText(QString("%1").arg(current_state_.driving_counter));
}

void DrivingWidget::drawWheelVisualization() {
    if ( ui_->graphicsView_Wheels->isVisible() == false )
        return;

    double wheel_angle = current_state_.steering_wheel_angle*WheelAnglePerSteeringRotation/MaxSteeringWheelRotation;    
    double total_width = ui_->graphicsView_Wheels->width()-12;
    double total_height = ui_->graphicsView_Wheels->height()-12;
    double car_dist_from_border = std::min(total_width, total_height) * 0.05;
    double car_length = std::min(total_width, total_height)-car_dist_from_border;
    double car_width = car_length / 2.0;
    double wheel_width = car_width / 4.0;
    double wheel_length = wheel_width*3.0;

    QTransform base_wheel_transform(1.0, 0.0, 0.0, 1.0, -wheel_width/2.0, -wheel_length/2.0);

    wheel_scene_.clear();
    wheel_scene_.setSceneRect(-total_width/2.0, -total_height/2.0, total_width, total_height);
    wheel_scene_.addRect(-car_width/2.0, -car_length/2.0, car_width, car_length );

    // back wheels
    QGraphicsRectItem *wheel = wheel_scene_.addRect(-car_width/2.0, car_length/2.0, wheel_width, wheel_length, QPen(), QBrush(Qt::SolidPattern));
    wheel->setTransform(base_wheel_transform);

    wheel = wheel_scene_.addRect(car_width/2.0, car_length/2.0, wheel_width, wheel_length, QPen(), QBrush(Qt::SolidPattern));
    wheel->setTransform(base_wheel_transform);

    // front wheels
    QColor wheel_color = Qt::black;
    if ( fabs(wheel_angle) >= MaxWheelAngle-MaxWheelAngleWarningOffset ) {
        wheel_color = Qt::red;
    }

    QTransform steering_transform;
    steering_transform.rotateRadians(wheel_angle);

    QTransform position_transform(1.0, 0.0, 0.0, 1.0, -car_width/2.0, -car_length/2.0);
    wheel = wheel_scene_.addRect(-wheel_width/2.0, -wheel_length/2.0, wheel_width, wheel_length, QPen(), QBrush(wheel_color, Qt::SolidPattern));
    wheel->setTransform(steering_transform*position_transform, true);

    position_transform = QTransform(1.0, 0.0, 0.0, 1.0, car_width/2.0, -car_length/2.0);
    wheel = wheel_scene_.addRect(-wheel_width/2.0, -wheel_length/2.0, wheel_width, wheel_length, QPen(), QBrush(wheel_color, Qt::SolidPattern));
    wheel->setTransform(steering_transform*position_transform, true);

    // Draw all hold sign in img
    if ( target_command_.all_hold ) {
        QGraphicsRectItem *background_item = wheel_scene_.addRect(-car_width/2.0+0.3, 0.0-car_length/12.0, car_width-0.6, car_length/6.0, QPen(Qt::red), QBrush(Qt::red));

        QFont textFont;
        textFont.setBold(true);
        textFont.setPixelSize(background_item->boundingRect().height()-0.5);
        QGraphicsTextItem *allHoldTextItem = wheel_scene_.addText("Hold!", textFont);
        allHoldTextItem->setDefaultTextColor(Qt::white);
        allHoldTextItem->moveBy( -allHoldTextItem->boundingRect().width()/2.0, -allHoldTextItem->boundingRect().height()/2.0);
    }
    else if ( current_state_.connection_loss ) {
        QGraphicsRectItem *background_item = wheel_scene_.addRect(-car_width/2.0+0.3, 0.0-car_length/12.0, car_width-0.6, car_length/6.0, QPen(Qt::yellow), QBrush(Qt::yellow));

        QFont textFont;
        textFont.setBold(true);
        textFont.setPixelSize(background_item->boundingRect().height()-0.5);
        QGraphicsTextItem *connectionLossItem = wheel_scene_.addText("No Signal!", textFont);
        connectionLossItem->setDefaultTextColor(Qt::black);
        connectionLossItem->moveBy( -connectionLossItem->boundingRect().width()/2.0, -connectionLossItem->boundingRect().height()/2.0);
    }



    ui_->graphicsView_Wheels->centerOn(0.0, 0.0);
}

void DrivingWidget::setGUIEnabled(bool enable) {
    ui_->pushButton_AllHold->setEnabled(enable);
    ui_->pushButton_ConfirmHeadTiltSensitivity->setEnabled(enable);
    ui_->pushButton_ConfirmHeadPanSensitivity->setEnabled(enable);
    ui_->pushButton_ConfirmSteeringSensitivity->setEnabled(enable);
    ui_->pushButton_OverrideLimits->setEnabled(enable);
    ui_->spinBox_HeadTiltSensitivity->setEnabled(enable);
    ui_->spinBox_HeadPanSensitivity->setEnabled(enable);
    ui_->spinBox_SteeringSensitivity->setEnabled(enable);
    ui_->lineEdit_SteeringAngle->setEnabled(enable);
    ui_->lineEdit_WheelAngle->setEnabled(enable);
}

void DrivingWidget::handleHeadCommand(double tilt, double pan) {
    head_tilt_speed_ = head_tilt_sensitivity_ * tilt;
    head_pan_speed_ = head_pan_sensitivity_ * pan;
}

void DrivingWidget::handleSteeringCommand(double step) {
    steering_speed_ = steering_sensitivity_ * step;
}


double DrivingWidget::rad2deg(double rad) {
    return rad*180.0/M_PI;
}

}


