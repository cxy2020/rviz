#include "generate_clean_path_tool.h"

#include <QFileDialog>
#include <QMessageBox>
#include <tf/transform_listener.h>

#include "rviz/viewport_mouse_event.h"
#include "path/composite_clothoid.h"

#include "record_goal_tool.h"
#include "publish_goal_tool.h"

using namespace traj_tools;
namespace rviz
{

GenerateCleanPathTool::GenerateCleanPathTool()
{
  shortcut_key_ = 'g';
}

void rviz::GenerateCleanPathTool::onInitialize()
{
  RecordGoalTool::onInitialize();
  setName( "GenerateCleanPath" );
}

void GenerateCleanPathTool::deactivate()
{
  PoseTool::deactivate();

  QString message_title = "Generate Tool";
  const int kExpectPointNumber = 2;
  const std::vector<geometry_msgs::PoseStamped>& goal_path = this->goal_path();
  if(goal_path.size() < kExpectPointNumber) {
    QMessageBox::information(nullptr, message_title, "The input goal point number is not enough!");
    clear_goal_path();
    return;
  }
  else if(goal_path.size() > kExpectPointNumber) {
    QMessageBox::information(nullptr, message_title, "Input more than 2 goal points, only take the first 2!");
  }

//  const geometry_msgs::Pose& pose0 = goal_path[0].pose;
  const geometry_msgs::Pose& pose1 = goal_path[0].pose;
  const geometry_msgs::Pose& pose3 = goal_path[1].pose;
//  Vec2 point0(pose0.position.x, pose0.position.y);
  Vec2 point0(0.0, 0.0);
  Vec2 point1(pose1.position.x, pose1.position.y);
  Vec2 point3(pose3.position.x, pose3.position.y);

  if(abs(point1.x() - point3.x()) < 0.3) {
    QMessageBox::information(nullptr, message_title, "The 2 point2 ares too close!");
    clear_goal_path();
    return;
  }

  Vec2 point2(point3.x(), point1.y());

  Float theta1 = 0.0;
  Float theta_t = common::kPi;
  Float dir_x3 = -1.0;
  Float dir_y3 = 0.0;
  if(point3.x() < point1.x()) {
    theta1 = common::kPi;
    theta_t = 0.0;
    dir_x3 = 1.0;
  }

  Float max_yaw = common::kHalfPi;
  Float max_k = 1.0;
  Float max_c = 5.0;

  Float path_precision = 0.05;
  std::vector<geometry_msgs::PoseStamped> goal_path_to_send;

  //Compute line0 from point0 to point1
  Line line0(point0, point1);
  GetPathPoint(line0, path_precision, goal_path[0].header, goal_path_to_send);

  //Compute line1 from point1 to point2
  Line line1(point1, point2);
  GetPathPoint(line1, path_precision, goal_path[0].header, goal_path_to_send);

  //Compute clothoid1 from point2 to point3
  CompositeClothoid clothoid1(point2, theta1, 0.0, point3.x(), point3.y(), theta_t, dir_x3, dir_y3, max_k, max_c, max_yaw);
  GetPathPoint(clothoid1, path_precision, goal_path[0].header, goal_path_to_send);

  //Compute line2 from point3 to point4
  Vec2 point4(point1.x(), point3.y());
  Line line2(point3, point4);
  GetPathPoint(line2, path_precision, goal_path[0].header, goal_path_to_send);

  RecordGoalTool::SaveToFile(goal_path_to_send);
  PublishGoalTool::Publish(goal_path_to_send);

  clear_goal_path();
}

void GenerateCleanPathTool::GetPathPoint(const Path2d &path,
                                         Float precision,
                                         const std_msgs::Header& header,
                                         std::vector<geometry_msgs::PoseStamped> &path_points)
{
  Float length = path.length();
  Float cur_pos = 0.0;

  geometry_msgs::PoseStamped pose_stamped;
  geometry_msgs::Pose pose_msg;
  while(cur_pos < length) {
    Pose init_pose = path.GetPose(cur_pos);

    pose_msg.position.x = init_pose.position.x;
    pose_msg.position.y = init_pose.position.y;
    pose_msg.position.z = init_pose.position.z;
    pose_msg.orientation.x = init_pose.orientation.x;
    pose_msg.orientation.y = init_pose.orientation.y;
    pose_msg.orientation.z = init_pose.orientation.z;
    pose_msg.orientation.w = init_pose.orientation.w;

    pose_stamped.header = header;
    pose_stamped.pose = pose_msg;
    path_points.push_back(pose_stamped);
    cur_pos += precision;
  }
}

//double GenerateCleanPathTool::judge_target_theta(const Eigen::Vector2d &point0, double theta0, const Eigen::Vector2d &point1, double theta1)
//{
//  Vec2 delta = point1 - point0;
//  Vec2 target_dir(cos(theta1), sin(theta1));
//  Float cross = delta.x() * target_dir.y() - delta.y() * target_dir.x();


} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::GenerateCleanPathTool, rviz::Tool )

