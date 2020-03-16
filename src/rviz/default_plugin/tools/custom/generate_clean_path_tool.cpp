#include "generate_clean_path_tool.h"

#include <QMessageBox>
#include <tf/transform_listener.h>

#include "rviz/viewport_mouse_event.h"
#include "path/composite_clothoid.h"

#include "rviz/display_context.h"
#include "goal_custom_tool.h"
#include "sync_pos_tool.h"

namespace rviz
{

GenerateCleanPathTool::GenerateCleanPathTool()
{
  shortcut_key_ = 'g';
}

void GenerateCleanPathTool::onInitialize()
{
  PoseTool::onInitialize();
  setName( "GenerateCleanPath" );
}

int GenerateCleanPathTool::processMouseEvent( ViewportMouseEvent& event )
{
  int flags = PoseTool::processMouseEvent(event);

  if( event.leftUp() )
  {
    if( state_ == Orientation )
    {
      //Keep active state after left mouse up
      int finished_flags = ~Finished;
      flags &= finished_flags;
      PoseTool::activate();
    }
  }
  return flags;
}

void GenerateCleanPathTool::deactivate()
{
  PoseTool::deactivate();

  QString message_title = "Generate Tool";
  const int kExpectPointNumber = 2;
  const std::vector<geometry_msgs::PoseStamped>& goal_path = goal_path_;
  if(goal_path.size() < kExpectPointNumber) {
    QMessageBox::information(nullptr, message_title, "The input goal point number is not enough!");
    goal_path_.clear();
    return;
  }
  else if(goal_path.size() > kExpectPointNumber) {
    QMessageBox::information(nullptr, message_title, "Input more than 2 goal points, only take the first 2!");
  }

  const geometry_msgs::Pose& pose_start = goal_path[0].pose;
  const geometry_msgs::Pose& pose_end = goal_path[1].pose;

  Vec2 point_start(pose_start.position.x, pose_start.position.y);
  Vec2 point_end(pose_end.position.x, pose_end.position.y);

  if(abs(point_start.x() - point_end.x()) < 0.3 || abs(point_start.y() - point_end.y()) < 0.3) {
    QMessageBox::information(nullptr, message_title, "The 2 point2 ares too close!");
    goal_path_.clear();
    return;
  }

  //Get the param defining how many parallel lines to generate
  ros::NodeHandle private_nh("rviz");
  const int kMaxPathNumber = 10;
  int path_number = private_nh.param("/rviz/gen_path_num", 2);
  if(path_number < 2) {
    path_number = 2;
  }
  else if(path_number > kMaxPathNumber) {
    QString message = "Generate only" + QString::number(kMaxPathNumber) + "paths!";
    QMessageBox::information(nullptr, message_title, message);
    path_number = 10;
  }

  const int point_number = path_number + path_number;
  Vec2 point[kMaxPathNumber + kMaxPathNumber];

  //Set the first and last 2 points
  point[0] = point_start;

  Float dx = point_end.x() - point_start.x();
  Float dy = (point_end.y() - point_start.y()) / (path_number - 1);
  int point_index = 1;
  Float theta_start = 0.0;
  Float theta_end = traj_tools::common::kPi;
  Float dir_end_x3 = -1.0;
  Float dir_end_y3 = 0.0;

  Float max_yaw = traj_tools::common::kHalfPi;
  Float max_k = private_nh.param("/rviz/max_k", 2.0);
  Float max_c = private_nh.param("/rviz/max_c", 10.0);

  Float path_precision = 0.05;
  std::vector<geometry_msgs::PoseStamped> goal_path_to_send;

  Vec2 point0(SyncPosTool::x(), SyncPosTool::y());
  traj_tools::Line first_line(point0, point[0]);
  GoalCustomTool::GetPathPoint(first_line, path_precision, goal_path[0].header, goal_path_to_send);

  while (point_index < point_number - 2) {
    point[point_index] = Vec2(point[point_index - 1].x() + dx, point[point_index - 1].y());
    point[point_index + 1] = Vec2(point[point_index].x(), point[point_index].y() + dy);

    traj_tools::Line line(point[point_index - 1], point[point_index]);

    theta_start = 0.0;
    theta_end = traj_tools::common::kPi;
    dir_end_x3 = -1.0;
    dir_end_y3 = 0.0;
    if(point[point_index].x() < point[point_index - 1].x()) {
      theta_start = traj_tools::common::kPi;
      theta_end = 0.0;
      dir_end_x3 = 1.0;
    }
    traj_tools::CompositeClothoid clothoid(point[point_index], theta_start, 0.0,
          point[point_index + 1].x(), point[point_index + 1].y(), theta_end, dir_end_x3, dir_end_y3, max_k, max_c, max_yaw);

    GoalCustomTool::GetPathPoint(line, path_precision, goal_path[0].header, goal_path_to_send);
    GoalCustomTool::GetPathPoint(clothoid, path_precision, goal_path[0].header, goal_path_to_send);

    dx = -dx;
    point_index += 2;
  }

  point[point_number - 1] = Vec2(point[point_number - 2].x() + dx, point[point_number - 2].y());

  //Add last line path
  traj_tools::Line last_line(point[point_number - 2], point[point_number - 1]);
  GoalCustomTool::GetPathPoint(last_line, path_precision, goal_path[0].header, goal_path_to_send);

  GoalCustomTool::SaveToFile(goal_path_to_send);
  GoalCustomTool::Publish(goal_path_to_send);

  goal_path_.clear();
}

void GenerateCleanPathTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal;
  tf::poseStampedTFToMsg(p, goal);
  ROS_INFO("Saving goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
      goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
  goal_path_.push_back(goal);
}
}   //namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::GenerateCleanPathTool, rviz::Tool )

