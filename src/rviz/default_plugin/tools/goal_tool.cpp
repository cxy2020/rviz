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

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include <QMessageBox>
#include <Eigen/Dense>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"

#include "rviz/default_plugin/tools/goal_tool.h"
#include "custom/goal_custom_tool.h"
#include "sync_pos_tool.h"

namespace rviz
{

GoalTool::GoalTool() : move_client_("move_base_flex/move_base", true), is_file_saved_(false)
{
  shortcut_key_ = 'g';

  topic_property_ = new StringProperty( "Topic", "goal",
                                        "The topic on which to publish navigation goals.",
                                        getPropertyContainer(), SLOT( updateTopic() ), this );
}

void GoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName( "2D Nav Goal" );
  updateTopic();
}

void GoalTool::updateTopic()
{
  try {
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_->getStdString(), 1 );
  } catch (const ros::Exception& e) {
    ROS_ERROR_STREAM_NAMED("GoalTool", e.what());
  }
}

void GoalTool::MoveBaseUpdate(const mbf_msgs::MoveBaseFeedbackConstPtr &move_base_feedback)
{
  poses_to_save_.push_back(move_base_feedback->current_pose);
}

void GoalTool::MoveBaseDoneCallback(const actionlib::SimpleClientGoalState & state,
                                    const mbf_msgs::MoveBaseResultConstPtr & result)
{
  save_file_timer_ = nh_.createTimer(ros::Duration(0.05), &GoalTool::SaveGoalPath, this, true);
}

void GoalTool::SaveGoalPath(const ros::TimerEvent &event)
{
  GoalCustomTool::SaveToFile(poses_to_save_, is_file_saved_);
  poses_to_save_.clear();
  is_file_saved_ = true;
}

void GoalTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal;
  tf::poseStampedTFToMsg(p, goal);

  int try_times = 0;
  while(!move_client_.waitForServer(ros::Duration(1.0))){
    ROS_INFO("Waiting for Move Base server to come up");
    if(++try_times > 2) {
      QMessageBox::information(nullptr, "connect mbf server", "cannot connect mbf server!");
      return;
    }
  }
  mbf_msgs::MoveBaseGoal target;
  target.target_pose = goal;
  move_client_.sendGoal(target,
                       boost::bind(&GoalTool::MoveBaseDoneCallback, this, _1, _2),
                       MovePointClient::SimpleActiveCallback(),
                       boost::bind(&GoalTool::MoveBaseUpdate, this, _1));
}

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::GoalTool, rviz::Tool )
