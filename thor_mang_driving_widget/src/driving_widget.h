#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <thor_mang_driving_controller/DrivingCommand.h>

#include <QMainWindow>
#include <QBasicTimer>
#include <QGraphicsScene>

namespace Ui {
class DrivingWidget;
}


class DrivingWidget : public QMainWindow
{
    Q_OBJECT

    enum AxisIDs {
      STEERING=0,
      HEAD_PAN=2,
      HEAD_TILT=3
    };

    enum ButtonIDs {
      FORWARD=1,
      ALL_STOP=2,
      STEERING_SENSITIVITY_PLUS=7,
      STEERING_SENSITIVITY_MINUS=6,
      HEAD_SENSITIVITY_PLUS=5,
      HEAD_SENSITIVITY_MINUS=4,
      HEAD_MODE_TO_DEFAULT=11
    };

public:
    explicit DrivingWidget(QWidget *parent = 0);
    ~DrivingWidget();

    void handleJoyPadEvent(sensor_msgs::JoyConstPtr msg);
    void handleNewJointStateEvent(sensor_msgs::JointStateConstPtr msg);

    void handleNewCameraImage(sensor_msgs::ImageConstPtr msg);
    void handleAllStopEnabled(thor_mang_driving_controller::DrivingCommandConstPtr msg);

    void handleControllerEnableACK(std_msgs::BoolConstPtr msg);

protected:
    void timerEvent(QTimerEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

public slots:
    void SLO_SteeringSensitivityConfirmed();
    void SLO_SteeringSensitivityChanged();

    void SLO_HeadSensitivityConfirmed();
    void SLO_HeadSensitivityChanged();

    void SLO_ShowCameraImage(bool show);
    void SLO_AllStopButtonChecked(bool active);
    void SLO_ToggleDrivingMode();

private:
    void sendDrivingCommand();
    void sendHeadCommand();
    void calculateSteeringAngle();

    void updateUI(bool update_steering_sensitivity = false, bool update_head_sensitivity = false);
    void drawWheelVisualization();
    void setGUIEnabled(bool enable);

    void handleHeadCommand(double tilt, double pan);
    void handleSteeringCommand(double step);

    trajectory_msgs::JointTrajectory generateTrajectoryMsg(std::vector<double> &joint_angles, std::vector<std::string> joint_names);


    ros::NodeHandle node_handle_;
    ros::NodeHandle node_handle_private_;

    Ui::DrivingWidget *ui_;
    QBasicTimer timer_;

    // Wheel visualization
    QGraphicsScene wheel_scene_;

    // Robot State Subscriber
    ros::Subscriber robot_state_sub_;
    bool received_robot_state_;

    // Joypad Command Input
    ros::Subscriber joypad_command_sub_;

    // Camera Image Subscriber
    std::string camera_topic_;
    ros::Subscriber camera_image_sub_;

    // Robot Enabled All Stop Subscriber
    ros::Subscriber all_stop_enabled_sub_;

    // Steering Publishers
    ros::Publisher head_command_pub_;
    ros::Publisher driving_command_pub_;

    // Enable / Disable controller
    ros::Publisher controller_enable_pub_;
    ros::Subscriber controller_enable_ack_sub_;

    // Steering parameters
    double steering_sensitivity_;
    double steering_speed_;
    double steering_correction_;
    double head_sensitivity_;
    double head_tilt_correction_;
    double head_pan_correction_;


    // Driving control elements
    bool all_stop_;
    double steering_angle_;
    double absolute_steering_angle_;
    bool drive_forward_;
    double time_from_start_;

    // Head control elements
    std::vector<std::string> head_joint_names_;
    std::vector<double> head_default_position_;
    double head_tilt_speed_;
    double head_pan_speed_;
    bool head_move_to_default_;

    // allow sensitivity changes
    bool allow_head_sensitivity_change_;
    bool allow_steering_sensitivity_change_;

    // Current joint state
    std::vector<std::string> robot_joint_names_;
    std::vector<double> robot_joint_positions_;

    // is controller enabled
    bool controller_enabled_;
};

