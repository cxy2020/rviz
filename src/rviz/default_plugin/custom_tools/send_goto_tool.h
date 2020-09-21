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
 * @file send_goto_tool.h
 * @author Xiaoying Chen
 */

#pragma once

#include "rviz/default_plugin/tools/pose_tool.h"
#include "rviz/default_plugin/custom_tools/task_control_tool.h"

namespace rock {
namespace custom_tools {

/**
 * @class SendGotoTool
 * @brief The tool button to send goal of "goto" path to task_controller.
 */
class SendGotoTool : public rviz::PoseTool, public TaskControlTool {
public:
  SendGotoTool();

  int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private:
  ros::NodeHandle nh_;
  ros::ServiceClient set_task_client_;
};

}   //namespace custom_tools
}   //namespace rock
