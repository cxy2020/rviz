#include "sync_pos_tool.h"

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <QFileDialog>
#include <QMessageBox>

#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/string_property.h"
#include "base_tools/base_math.h"

namespace rviz
{

double SyncPosTool::x_ = 0.0;
double SyncPosTool::y_ = 0.0;
double SyncPosTool::theta_ = 0.0;
bool SyncPosTool::is_synchronized_ = false;

SyncPosTool::SyncPosTool()
{
  shortcut_key_ = 's';
  pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>( "initialpose", 1 );

  sub_ = nh_.subscribe("/gazebo/model_states", 1000, &SyncPosTool::ModelStatesUpdated, this);
  std_dev_x_ = new FloatProperty("X std deviation", 0.5, "X standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_y_ = new FloatProperty("Y std deviation", 0.5, "Y standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_theta_ = new FloatProperty("Theta std deviation", M_PI / 12.0, "Theta standard deviation for initial pose [rad]", getPropertyContainer());
  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_theta_->setMin(0);
}

void SyncPosTool::onInitialize()
{
  Tool::onInitialize();
  setName( "SyncPosTool" );
}

void SyncPosTool::activate()
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = ros::Time::now();
  pose.pose.pose.position.x = x_;
  pose.pose.pose.position.y = y_;

  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta_);
  tf::quaternionTFToMsg(quat,
                        pose.pose.pose.orientation);
  pose.pose.covariance[6*0+0] = std::pow(std_dev_x_->getFloat(), 2);
  pose.pose.covariance[6*1+1] = std::pow(std_dev_y_->getFloat(), 2);
  pose.pose.covariance[6*5+5] = std::pow(std_dev_theta_->getFloat(), 2);
  ROS_INFO("Setting pose: %.3f %.3f %.3f [frame=%s]", x_, y_, theta_, fixed_frame.c_str());
  pub_.publish(pose);
}

void SyncPosTool::deactivate()
{
}

void SyncPosTool::ModelStatesUpdated(const gazebo_msgs::ModelStates &msg)
{
  //Get realtime position from gazebo and transform from quartenion to euler
  x_ = msg.pose[2].position.x;
  y_ = msg.pose[2].position.y;
  double sin_theta = msg.pose[2].orientation.z;
  double cos_theta = msg.pose[2].orientation.w;
  theta_ = traj_tools::common::Acos(cos_theta);
  if(sin_theta < 0) {
    theta_ = -theta_;
  }
  theta_ *= 2.0;

  //synchronize the position of the robot if this is never done before
  if(!is_synchronized_) {
    ros::Duration(0.5).sleep();
    activate();
    is_synchronized_ = true;
    ROS_WARN("Update rviz initial pose: %.3f %.3f %.3f ", x_, y_, theta_);
  }
}

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::SyncPosTool, rviz::Tool )
