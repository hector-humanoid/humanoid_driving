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

#include <ros/ros.h>
#include <keyboard/Key.h>
#include <sensor_msgs/Joy.h>

namespace key_to_joy_mapper {
class KeyToJoyMapper {
public:
    KeyToJoyMapper() : private_node_handle_("~") {
      loadKeyMappings();

      joy_pub_ = node_handle_.advertise<sensor_msgs::Joy>("/joy", 1, false);
      key_up_sub_ = node_handle_.subscribe("/keyboard/keyup", 1, &KeyToJoyMapper::handleKeyUp, this);
      key_down_sub_ = node_handle_.subscribe("/keyboard/keydown", 1, &KeyToJoyMapper::handleKeyDown, this);
    }
    
    void handleKeyUp(keyboard::KeyConstPtr key_up_msg) {
      int key = key_up_msg->code;
      if ( buttons_.find(key) != buttons_.end() ) {
          joy_state_msg_.buttons[ buttons_[key]] = 0;
      }
      else if ( axis_plus_.find(key) != axis_plus_.end() ) {
          joy_state_msg_.axes[ axis_plus_[key] ] = 0.0;
      }
      else if ( axis_minus_.find(key) != axis_minus_.end() ) {
          joy_state_msg_.axes[ axis_minus_[key] ] = 0.0;
      }

      joy_state_msg_.header.stamp = ros::Time::now();
      joy_pub_.publish(joy_state_msg_);
      
    }
    
    void handleKeyDown(keyboard::KeyConstPtr key_down_msg) {
      int key = key_down_msg->code;
      if ( buttons_.find(key) != buttons_.end() ) {
          joy_state_msg_.buttons[ buttons_[key]] = 1;
      }
      else if ( axis_plus_.find(key) != axis_plus_.end() ) {
          joy_state_msg_.axes[ axis_plus_[key] ] = 1.0;
      }
      else if ( axis_minus_.find(key) != axis_minus_.end() ) {
          joy_state_msg_.axes[ axis_minus_[key] ] = -1.0;
      }

      joy_state_msg_.header.stamp = ros::Time::now();
      joy_pub_.publish(joy_state_msg_);
    }
    
private:
    void loadKeyMappings() {
      std::vector<int> button_keys = private_node_handle_.param("buttons", std::vector<int>());
      for ( int i = 0; i < button_keys.size(); i++ ) {
          if ( button_keys[i] != -1 ) {
            buttons_[button_keys[i]] = i;
          }
      }

      std::vector<int> axis_plus_keys = private_node_handle_.param("axis_plus", std::vector<int>());
      for ( int i = 0; i < axis_plus_keys.size(); i++ ) {
          if ( axis_plus_keys[i] != -1 ) {
            axis_plus_[axis_plus_keys[i]] = i;
          }

      }
      
      std::vector<int> axis_minus_keys = private_node_handle_.param("axis_minus", std::vector<int>());
      for ( int i = 0; i < axis_minus_keys.size(); i++ ) {
          if ( axis_minus_keys[i] != -1 ) {
            axis_minus_[axis_minus_keys[i]] = i;
          }
      }
      
      if ( axis_plus_.size() != axis_minus_.size() ) {
        ROS_FATAL("[key_to_joy_mapper] Different number of buttons for axis_plus and axis_minus. This cannot be right!");
        ros::shutdown();
        exit(-1);
      }

      if ( axis_plus_.size() == 0 && buttons_.size() == 0 ) {
          ROS_FATAL("[key_to_joy_mapper] Neither axis nor button mapping set. This cannot be right!");
          ros::shutdown();
          exit(-1);
      }

      joy_state_msg_.axes.assign( axis_minus_keys.size(), 0.0);
      joy_state_msg_.buttons.assign( button_keys.size(), 0);
    }

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;

    ros::Subscriber key_up_sub_;
    ros::Subscriber key_down_sub_;
    ros::Publisher joy_pub_;

    sensor_msgs::Joy joy_state_msg_;

    std::map<int, size_t> buttons_;
    std::map<int, size_t> axis_plus_;
    std::map<int, size_t> axis_minus_;
};
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "key_to_joy_mapper");
  key_to_joy_mapper::KeyToJoyMapper mapper;
  ros::spin();  
}
