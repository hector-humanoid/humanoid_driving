#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>

#include <QMainWindow>
#include <QBasicTimer>

namespace Ui {
class DrivingWidget;
}


class DrivingWidget : public QMainWindow
{
    Q_OBJECT

public:
    explicit DrivingWidget(QWidget *parent = 0);
    ~DrivingWidget();

    void handleTargetSteeringPositionChanged(std_msgs::Float64ConstPtr msg );
    void handleNewSpeedFactor(std_msgs::Float64ConstPtr msg);
    void handleNewCameraImage(sensor_msgs::ImageConstPtr msg);
    void handleAllStopEnabled(std_msgs::BoolConstPtr msg);

protected:
    void timerEvent(QTimerEvent *event);

public slots:
    void SLO_SpeedFactorChanged(double factor);
    void SLO_TimeFromStartChanged(double time_from_start);
    void SLO_ShowCameraImage(bool show);

private:
    Ui::DrivingWidget *ui_;
    QBasicTimer timer_;

    ros::NodeHandle node_handle_;
    ros::NodeHandle node_handle_private_;

    ros::Publisher speed_factor_pub_;
    ros::Publisher time_from_start_pub_;

    ros::Subscriber target_steering_position_sub_;
    ros::Subscriber speed_factor_sub_;
    ros::Subscriber camera_image_sub_;
    ros::Subscriber all_stop_enabled_sub_;

    std::string camera_topic_;
};

