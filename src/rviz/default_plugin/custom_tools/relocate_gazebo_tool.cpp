#include "rviz/default_plugin/custom_tools/relocate_gazebo_tool.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "rviz/display_context.h"
#include "rviz/properties/float_property.h"

using namespace rviz;

namespace rock {
namespace custom_tools {

RelocateGazeboTool::RelocateGazeboTool() : is_updated_(false) {
  ros::NodeHandle nh;
  location_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
  pose_sub_ = nh.subscribe("gazebo/model_states", 1000, &RelocateGazeboTool::OnPoseUpdated, this);
  std_dev_x_ = new FloatProperty("X std deviation", 0.5, "X standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_y_ = new FloatProperty("Y std deviation", 0.5, "Y standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_theta_ = new FloatProperty("Theta std deviation", M_PI / 12.0, "Theta standard deviation for initial pose [rad]", getPropertyContainer());
  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_theta_->setMin(0);
}

void RelocateGazeboTool::activate() {
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = current_pose_.header.frame_id;
  pose.header.stamp = ros::Time::now();
  pose.pose.pose = current_pose_.pose;
  pose.pose.covariance[6*0+0] = std::pow(std_dev_x_->getFloat(), 2);
  pose.pose.covariance[6*1+1] = std::pow(std_dev_y_->getFloat(), 2);
  pose.pose.covariance[6*5+5] = std::pow(std_dev_theta_->getFloat(), 2);
  location_pub_.publish(pose);
}

void RelocateGazeboTool::OnPoseUpdated(const gazebo_msgs::ModelStates& msg) {
  //Update current pose.
  current_pose_.header.frame_id = context_->getFixedFrame().toStdString();
  current_pose_.header.stamp = ros::Time::now();
  current_pose_.pose.position.x = msg.pose[2].position.x;
  current_pose_.pose.position.y = msg.pose[2].position.y;
  current_pose_.pose.position.z = msg.pose[2].position.z;
  current_pose_.pose.orientation.x = msg.pose[2].orientation.x;
  current_pose_.pose.orientation.y = msg.pose[2].orientation.y;
  current_pose_.pose.orientation.z = msg.pose[2].orientation.z;
  current_pose_.pose.orientation.w = msg.pose[2].orientation.w;
}

}   //namespace custom_tools
}   //namespace rock

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rock::custom_tools::RelocateGazeboTool, rviz::Tool)
