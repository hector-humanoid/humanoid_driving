//=================================================================================================
// Copyright (c) 2015, Achim Stein, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <humanoid_driving_controller/DrivingCommand.h>
#include <humanoid_driving_controller/DrivingState.h>

#include <QMainWindow>
#include <QBasicTimer>
#include <QGraphicsScene>

namespace Ui {
class DrivingWidget;
}

namespace humanoid_driving_widget {
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
      BUTTON_ALL_HOLD,
      BUTTON_STEERING_SENSITIVITY_PLUS,
      BUTTON_STEERING_SENSITIVITY_MINUS,
      BUTTON_HEAD_SENSITIVITY_PLUS,
      BUTTON_HEAD_SENSITIVITY_MINUS,
      BUTTON_HEAD_TO_DEFAULT,
      BUTTON_STEERING_TO_DEFAULT,
      NUM_JOYPAD_IDS
    };

public:
    explicit DrivingWidget(QWidget *parent = nullptr);
    ~DrivingWidget();

    // act on joypad action
    void handleJoyPadEvent(sensor_msgs::JoyConstPtr msg);

    // show a new camera image in the view
    void handleNewCameraImage(sensor_msgs::ImageConstPtr msg);

    // receive all-hold event from driving controller
    void handleAllHoldEnabled(humanoid_driving_controller::DrivingCommandConstPtr msg);

    // handle acknowledgement after driving controller was enabled
    void handleControllerEnableACK(std_msgs::BoolConstPtr msg);

    // update UI to show current driving state
    void handleNewDrivingState(humanoid_driving_controller::DrivingStateConstPtr msg);

protected:
    // regular timer for steering wheel and head target position updates
    void timerEvent(QTimerEvent *event) override;

protected slots:
    // change steering sensitivity through GUI
    void SLO_SteeringSensitivityConfirmed();
    void SLO_SteeringSensitivityChanged();

    // change head tilt sensitivity through GUI
    void SLO_HeadTiltSensitivityConfirmed();
    void SLO_HeadTiltSensitivityChanged();

    // change head pan sensitivity through GUI
    void SLO_HeadPanSensitivityConfirmed();
    void SLO_HeadPanSensitivityChanged();

    // show / hide camera image
    void SLO_ShowCameraImage(bool show);

    // handle clicking on all-hold button
    void SLO_AllHoldButtonChecked(bool active);

    // enable / disable driving controller
    void SLO_ToggleDrivingMode();

    // allow / deny overriding the steering limits
    void SLO_OverrideLimits(bool override);

private:
    // update target and send driving command
    void sendDrivingCommand();

    // make sure all inputs are within the steering limits
    void checkSteeringLimits();

    // update UI from current state
    void updateUI(bool update_steering_sensitivity = false, bool update_head_sensitivity = false);

    // draw car image with wheels
    void drawWheelVisualization();

    // enable / disable UI
    void setGUIEnabled(bool enable);

    // change head tilt and pan speed
    void handleHeadCommand(double tilt, double pan);

    // change steering speed
    void handleSteeringCommand(double step);

    // convert rad to degree angles
    inline double rad2deg(double rad);

    // Node handles
    ros::NodeHandle node_handle_;
    ros::NodeHandle node_handle_private_;

    // UI widget itself
    Ui::DrivingWidget *ui_;

    // Update timer
    QBasicTimer timer_;

    // Wheel visualization
    QGraphicsScene wheel_scene_;

    // Joypad Command Input
    ros::Subscriber joypad_command_sub_;

    // Camera Image Subscriber
    std::string camera_topic_;
    ros::Subscriber camera_image_sub_;

    // All Hold Subscriber
    ros::Subscriber all_hold_enabled_sub_;

    // Command Publisher
    ros::Publisher driving_command_pub_;

    // Enable / Disable controller
    ros::Publisher controller_enable_pub_;
    ros::Subscriber controller_enable_ack_sub_;

    // Get current state from robot
    ros::Subscriber driving_state_sub_;

    // Connection loss repaired?
    ros::Subscriber connection_loss_sub_;

    // Publish wheel angle for visualization
    ros::Publisher wheel_angle_pub_;

    // current and target states
    humanoid_driving_controller::DrivingState current_state_;
    humanoid_driving_controller::DrivingCommand target_command_;

    // Steering parameters
    double steering_sensitivity_;
    double steering_speed_;
    bool ignore_steering_limits_;

    // Head control parameters
    double head_tilt_speed_;
    double head_pan_speed_;
    double head_tilt_sensitivity_;
    double head_pan_sensitivity_;

    // is controller enabled?
    bool controller_enabled_;    

    // joypad ids
    int joypad_ids_[NUM_JOYPAD_IDS];

    // constants
    const double WheelAnglePerSteeringRotation = M_PI/6.0; // 30° wheel rotation for 360° steering wheel rotation
    const double MaxSteeringWheelRotation = 3*M_PI;        // 540° max steering wheel rotation
    const double MaxWheelAngleWarningOffset = 3.5e-3;      // Warn about reaching maximum steering angle about 0.2° before
    const double SteeringSensitivityStep = 0.1;            // sensitivity change on joypad button press
    const double HeadSensitivityStep = 0.1;                // sensitivity change on joypad button press
    const double MaxWheelAngle = MaxSteeringWheelRotation*WheelAnglePerSteeringRotation/MaxSteeringWheelRotation;

    // minimum and maximum values for head tilt and pan (TODO: read from urdf or similar...)
    const double MinHeadPan = -1.57;
    const double MaxHeadPan = 1.57;
    const double MinHeadTilt = -1.32;
    const double MaxHeadTilt = 0.79;
};

}

