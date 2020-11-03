#include "rviz/default_plugin/custom_tools/relocate_amcl_tool.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <amcl/RectPara.h>
#include <QMessageBox>

#include "task_msgs/SendTaskInfo.h"
#include "nav_tools/nav/path_type.h"
#include "nav_tools/path/base_path_tool.h"
#include "rviz/display_context.h"
#include "rviz/window_manager_interface.h"

using namespace rock::nav_tools;

namespace rock {
namespace custom_tools {

RelocateAmclTool::RelocateAmclTool() : square_width_(1.0) {
  ros::NodeHandle nh;
  set_task_client_ = nh.serviceClient<task_msgs::SendTaskInfo>("task_control/set_task");
  ros::NodeHandle private_nh("~");
  private_nh.param("relocat_width", square_width_, square_width_);
}

void RelocateAmclTool::onPoseSet(double x, double y, double /*theta*/) {
  if (!ros::service::waitForService("range_localization", ros::Duration(5))) {
    QMessageBox::warning(context_->getWindowManager()->getParentWindow(),
                         "Wait service result", "Waiting for localization service failed!");
    return;
  }

  //Generate the square according to the current pose set.
  double half_edge = square_width_ * 0.5;
  amcl::RectParaRequest req;
  amcl::RectParaResponse resp;
  req.rect_min_x = x - half_edge;
  req.rect_max_x = x + half_edge;
  req.rect_min_y = y - half_edge;
  req.rect_max_y = y + half_edge;
  if (ros::service::call("range_localization", req, resp)) {
    ROS_INFO("Relocation succeeds and new location is -> %.2f %.2f %.2f %.2f", req.rect_max_x,
             req.rect_min_x, req.rect_max_y, req.rect_min_y);
    std::string result = "Relocation succeeds and new location is ->"
        + std::to_string(req.rect_max_x) + " "
        + std::to_string(req.rect_min_x) + " "
        + std::to_string(req.rect_max_y) + " "
        + std::to_string(req.rect_min_y);
    QMessageBox::information(context_->getWindowManager()->getParentWindow(),
                             "Relocation result", QString(result.c_str()));
  }
  else {
    QMessageBox::warning(context_->getWindowManager()->getParentWindow(),
                         "Relocation result", "Relocation failed!");
  }
}

}   //namespace custom_tools
}   //namespace rock

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rock::custom_tools::RelocateAmclTool, rviz::Tool)
