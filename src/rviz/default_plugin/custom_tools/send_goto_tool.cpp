#include "rviz/default_plugin/custom_tools/send_goto_tool.h"

#include <tf/transform_listener.h>

#include "task_msgs/SendTaskInfo.h"
#include "nav_tools/nav/path_type.h"
#include "nav_tools/path/base_path_tool.h"
#include "rviz/display_context.h"

using namespace rock::nav_tools;

namespace rock {
namespace custom_tools {

SendGotoTool::SendGotoTool() {
  ros::NodeHandle nh;
  set_task_client_ = nh.serviceClient<task_msgs::SendTaskInfo>("task_control/set_task");
}

void SendGotoTool::onPoseSet(double x, double y, double theta) {
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  task_msgs::SendTaskInfoRequest req;
  task_msgs::SendTaskInfoResponse res;
  req.path.path_id = 1;
  req.path.frame_id = fixed_frame;
  req.path.path_type = nav_tools::GOTO;
  req.path.can_smooth = false;
  req.path.is_end_pose_set = true;
  req.path.is_path_reversed = false;

  task_msgs::Point2d target_point;
  target_point.x = x;
  target_point.y = y;
  target_point.angle = theta;
  Vec2f dir;
  calc_dir(theta, dir);
  target_point.dir_x = dir.x();
  target_point.dir_y = dir.y();
  req.path.poses.emplace_back(target_point);

  set_task_client_.call(req, res);
}

}   //namespace custom_tools
}   //namespace rock

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rock::custom_tools::SendGotoTool, rviz::Tool)
