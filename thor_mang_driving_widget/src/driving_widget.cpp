#include "driving_widget.h"
#include "ui_driving_widget.h"

#include <ros/ros.h>
#include <ros/package.h>


#include<QFile>
#include<QTextStream>
#include<QDebug>

DrivingWidget::DrivingWidget(QWidget *parent) :
    QMainWindow(parent),
    ui_(new Ui::DrivingWidget),
    node_handle_private_("~")
{
    ui_->setupUi(this);

    node_handle_private_.param("camera_topic", camera_topic_, std::string("/head_cam/image_raw"));

    speed_factor_pub_ = node_handle_.advertise<std_msgs::Float64>("driving_widget/speed_factor", 1, false);
    time_from_start_pub_ = node_handle_.advertise<std_msgs::Float64>("driving_widget/time_from_start", 1, false);

    target_steering_position_sub_ = node_handle_.subscribe("driving_controller/target_steering_position", 1, &DrivingWidget::handleTargetSteeringPositionChanged, this);
    speed_factor_sub_ = node_handle_.subscribe("driving_controller/speed_factor", 1, &DrivingWidget::handleNewSpeedFactor, this);
    all_stop_enabled_sub_ = node_handle_.subscribe("driving_controller/all_stop_enabled", 1, &DrivingWidget::handleAllStopEnabled, this);

    timer_.start(33, this);

    connect(ui_->spinBox_SpeedControlFactor, SIGNAL(valueChanged(double)), this, SLOT(SLO_SpeedFactorChanged(double)));
    connect(ui_->spinBox_TimeFromStart, SIGNAL(valueChanged(double)), this, SLOT(SLO_TimeFromStartChanged(double)));
    connect(ui_->pushButton_ShowCameraImage, SIGNAL(toggled(bool)), this, SLOT(SLO_ShowCameraImage(bool)));
}

DrivingWidget::~DrivingWidget()
{
    delete ui_;
}


void DrivingWidget::timerEvent(QTimerEvent *event)
{
    // check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();

    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    if(event->timerId() == timer_.timerId())
        ros::spinOnce();
}

void DrivingWidget::handleTargetSteeringPositionChanged(std_msgs::Float64ConstPtr msg ) {
    double target_steering_position = msg->data;
    double wheel_position = target_steering_position*45/540;

    ui_->lineEdit_WheelAngle->setText( QString("%1 \260").arg(wheel_position, 3, 'f', 1));
    ui_->lineEdit_SteeringAngle->setText(QString("%1 \260").arg(target_steering_position, 3, 'f', 1));


    // set dial => map to [-180; +180]
    if ( target_steering_position >= 360.0 )  target_steering_position -= 360.0;
    if ( target_steering_position <= -360.0 ) target_steering_position += 360.0;

    if ( target_steering_position <= -180.0 ) target_steering_position += 360.0;
    if ( target_steering_position >= 180.0 )  target_steering_position -= 360.0;

    ui_->dial_TargetSteeringPosition->setValue( (int)target_steering_position);
}

void DrivingWidget::handleNewSpeedFactor(std_msgs::Float64ConstPtr msg) {
    ui_->spinBox_SpeedControlFactor->blockSignals(true);
    ui_->spinBox_SpeedControlFactor->setValue(msg->data);
    ui_->spinBox_SpeedControlFactor->blockSignals(false);
}

void DrivingWidget::handleNewCameraImage(sensor_msgs::ImageConstPtr msg) {
    QImage img(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
    ui_->label_CameraImage->setPixmap(QPixmap::fromImage(img));
}

void DrivingWidget::handleAllStopEnabled(std_msgs::BoolConstPtr msg) {
    ui_->label_AllStopActive->setVisible(msg->data);
}

void DrivingWidget::SLO_SpeedFactorChanged(double factor) {
    std_msgs::Float64 speed_factor_msg;
    speed_factor_msg.data = factor;
    speed_factor_pub_.publish(speed_factor_msg);

}

void DrivingWidget::SLO_TimeFromStartChanged(double time_from_start) {
    std_msgs::Float64 time_from_start_msg;
    time_from_start_msg.data = time_from_start;
    time_from_start_pub_.publish(time_from_start_msg);
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


