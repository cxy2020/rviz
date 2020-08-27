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
 * @file task_control_tool.h
 * @author Xiaoying Chen
 */

#pragma once

#include <ros/ros.h>

#include "rviz/tool.h"

namespace rock {
namespace custom_tools {

/**
 * @class TaskControlTool
 * @brief Plugin for navigation task control, including pausing/resuming the task, canceling the
 *  task, and reset the task_controller form exception.
 */
class TaskControlTool : public rviz::Tool {
public:
  TaskControlTool();

  void activate() override {}

  void deactivate() override {}

  int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

private:
  ros::ServiceClient start_task_client_;
  ros::ServiceClient pause_task_client_;
  ros::ServiceClient emergency_stop_client_;
  ros::ServiceClient cancel_task_client_;
};

}   //namespace custom_tools
}   //namespace rock
