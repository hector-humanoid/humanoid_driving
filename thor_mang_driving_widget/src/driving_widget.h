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
#include <thor_mang_driving_controller/DrivingState.h>

#include <QMainWindow>
#include <QBasicTimer>
#include <QGraphicsScene>

namespace Ui {
class DrivingWidget;
}


class DrivingWidget : public QMainWindow
{
    Q_OBJECT

    enum JoypadIDs {
      AXIS_STEERING=0,
      AXIS_HEAD_PAN,
      AXIS_HEAD_TILT,
      AXIS_HEAD_PAN_2,
      AXIS_HEAD_TILT_2,
      BUTTON_FORWARD,
      BUTTON_ALL_STOP,
      BUTTON_STEERING_SENSITIVITY_PLUS,
      BUTTON_STEERING_SENSITIVITY_MINUS,
      BUTTON_HEAD_SENSITIVITY_PLUS,
      BUTTON_HEAD_SENSITIVITY_MINUS,
      BUTTON_HEAD_TO_DEFAULT,
      BUTTON_STEERING_TO_DEFAULT,
      NUM_JOYPAD_IDS
    };

public:
    explicit DrivingWidget(QWidget *parent = 0);
    ~DrivingWidget();

    void handleJoyPadEvent(sensor_msgs::JoyConstPtr msg);
    void handleNewJointStateEvent(sensor_msgs::JointStateConstPtr msg);

    void handleNewCameraImage(sensor_msgs::ImageConstPtr msg);
    void handleAllStopEnabled(thor_mang_driving_controller::DrivingCommandConstPtr msg);

    void handleControllerEnableACK(std_msgs::BoolConstPtr msg);
    void handleNewDrivingState(thor_mang_driving_controller::DrivingStateConstPtr msg);

protected:
    void timerEvent(QTimerEvent *event) override;

public slots:
    void SLO_SteeringSensitivityConfirmed();
    void SLO_SteeringSensitivityChanged();

    void SLO_HeadSensitivityConfirmed();
    void SLO_HeadSensitivityChanged();

    void SLO_ShowCameraImage(bool show);
    void SLO_AllStopButtonChecked(bool active);
    void SLO_ToggleDrivingMode();

    void SLO_OverrideLimits(bool override);

private:
    void sendDrivingCommand();
    void checkSteeringLimits();

    void updateUI(bool update_steering_sensitivity = false, bool update_head_sensitivity = false);
    void drawWheelVisualization();
    void setGUIEnabled(bool enable);

    void handleHeadCommand(double tilt, double pan);
    void handleSteeringCommand(double step);

    ros::NodeHandle node_handle_;
    ros::NodeHandle node_handle_private_;

    Ui::DrivingWidget *ui_;
    QBasicTimer timer_;

    // Wheel visualization
    QGraphicsScene wheel_scene_;

    // Joypad Command Input
    ros::Subscriber joypad_command_sub_;

    // Camera Image Subscriber
    std::string camera_topic_;
    ros::Subscriber camera_image_sub_;

    // Robot Enabled All Stop Subscriber
    ros::Subscriber all_stop_enabled_sub_;

    // Steering Publishers
    ros::Publisher driving_command_pub_;
    ros::Publisher head_move_to_default_pub_;

    // Enable / Disable controller, Reset
    ros::Publisher controller_enable_pub_;
    ros::Subscriber controller_enable_ack_sub_;

    // Get absolute steering angle from robot
    ros::Subscriber driving_state_sub_;

    // Connection loss repaired?
    ros::Subscriber connection_loss_sub_;

    // Publish wheel angle for visualization
    ros::Publisher wheel_angle_pub_;

    // Steering parameters
    double steering_sensitivity_;
    double steering_speed_;
    double steering_correction_;
    double head_sensitivity_;
    double head_tilt_correction_;
    double head_pan_correction_;


    // Driving control elements
    bool all_stop_;
    double current_steering_angle_;
    double current_absolute_steering_angle_;
    double absolute_target_steering_angle_;
    bool drive_forward_;
    unsigned int driving_counter_;
    double time_from_start_;

    // Head control elements
    double head_target_tilt_;
    double head_target_pan_;
    double head_tilt_speed_;
    double head_pan_speed_;
    double current_head_pan_;
    double current_head_tilt_;

    // allow sensitivity changes
    bool allow_head_sensitivity_change_;
    bool allow_steering_sensitivity_change_;

    // is controller enabled
    bool controller_enabled_;

    bool ignore_steering_limits_;

    // joypad ids
    int joypad_ids_[NUM_JOYPAD_IDS];

    bool connection_loss_;
};

