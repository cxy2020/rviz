/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "send_goto_task_tool.h"

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include <QMessageBox>
#include <Eigen/Dense>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"

#include "goal_custom_tool.h"

#include "task_control_service.h"

namespace rviz
{

SendGotoTaskTool::SendGotoTaskTool() : is_file_saved_(false)
{
  shortcut_key_ = 'g';

  topic_property_ = new StringProperty( "Topic", "goal",
                                        "The topic on which to publish goto task.",
                                        getPropertyContainer(), SLOT( updateTopic() ), this );
}

void SendGotoTaskTool::onInitialize()
{
  PoseTool::onInitialize();
  setName( "SendGoto" );
  updateTopic();
}

void SendGotoTaskTool::updateTopic()
{
  try {
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_->getStdString(), 1 );
  } catch (const ros::Exception& e) {
    ROS_ERROR_STREAM_NAMED("SendGoto", e.what());
  }
}

void SendGotoTaskTool::SaveGoalPath(const ros::TimerEvent &event)
{
  GoalCustomTool::SaveToFile(poses_to_save_, is_file_saved_);
  poses_to_save_.clear();
  is_file_saved_ = true;
}

void SendGotoTaskTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal;
  tf::poseStampedTFToMsg(p, goal);

  TaskControlService::get_instance()->AddGotoTask(goal);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::SendGotoTaskTool, rviz::Tool )
