/***************************************************************************************************
 * Copyright (C) 2020, Roborock, Inc.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 *    and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************************************/

/**
 * @file relocate_gazebo_tool.h
 * @author Xiaoying Chen
 */

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>

#include "rviz/tool.h"

namespace rviz {
class FloatProperty;
}

namespace rock {
namespace custom_tools {

/**
 * @class RelocateGazeboTool
 * @brief The tool to relocate the robot from gazebo's realtime robot pose.
 */
class RelocateGazeboTool : public rviz::Tool {
public:
  RelocateGazeboTool();

  virtual void activate();

  virtual void deactivate() {}

private:
  void OnPoseUpdated(const gazebo_msgs::ModelStates &msg);

  geometry_msgs::PoseStamped current_pose_;
  bool is_updated_;
  rviz::FloatProperty* std_dev_x_;
  rviz::FloatProperty* std_dev_y_;
  rviz::FloatProperty* std_dev_theta_;

  ros::Publisher location_pub_;
  ros::Subscriber pose_sub_;
};

}   //namespace custom_tools
}   //namespace rock
