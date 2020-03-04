#include "publish_goal_tool.h"

#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>

#include <QFileDialog>
#include <QMessageBox>

namespace rviz
{
typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction> PathClient;

PublishGoalTool::PublishGoalTool()
{
  shortcut_key_ = 'p';
}

void PublishGoalTool::onInitialize() {
  Tool::onInitialize();
  setName( "PublishGoal" );
}

void PublishGoalTool::activate() {
  QFileDialog file_diag;
  file_diag.setWindowTitle("Publish Goal");
  file_diag.setNameFilter("*.gpath");
  file_diag.setFileMode(QFileDialog::ExistingFile);
  int result = file_diag.exec();
  if(QFileDialog::FileName == result || QFileDialog::Accept == result) {
    QString filename = file_diag.selectedFiles()[0];
    QFile goal_path_file(filename);
    goal_path_file.open(QFile::ReadOnly);

    nav_msgs::Path path;
    mbf_msgs::ExePathGoal target_path;

    geometry_msgs::PoseStamped goal_pose;
    while (!goal_path_file.atEnd()) {
      QByteArray line = goal_path_file.readLine();
      QList<QByteArray> pose_data_list = line.split('\t');
      if(8 != pose_data_list.size()) {
        QMessageBox::information(nullptr, "Process Result", "Wrong data format!");
        continue;
      }

      QList<QByteArray>::iterator pose_iter = pose_data_list.begin();
      goal_pose.header.frame_id = (*pose_iter++).data();
      goal_pose.pose.position.x = (*pose_iter++).toDouble();
      goal_pose.pose.position.y = (*pose_iter++).toDouble();
      goal_pose.pose.position.z = (*pose_iter++).toDouble();
      goal_pose.pose.orientation.x = (*pose_iter++).toDouble();
      goal_pose.pose.orientation.y = (*pose_iter++).toDouble();
      goal_pose.pose.orientation.z = (*pose_iter++).toDouble();
      goal_pose.pose.orientation.w = (*pose_iter++).toDouble();
      path.poses.push_back(goal_pose);
    }
    goal_path_file.close();

    if(path.poses.size() > 0) {
      target_path.path = path;
      PathClient pc("mbf_costmap_nav/exe_path", true); // true doesnt need ros::spin

      int try_times = 0;
      while(!pc.waitForServer(ros::Duration(1.0))){
        ROS_INFO("Waiting for Move Base server to come up");
        if(++try_times > 2) {
          QMessageBox::information(nullptr, "connect mbf server", "cannot connect mbf server!");
          return;
        }
      }
      pc.sendGoal(target_path);

      pc.waitForResult();

      if(pc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Base moved %s", pc.getState().toString().c_str());
      }
      else if(pc.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("Goal aborted");
      }
      else {
        ROS_INFO("Base failed to move for some reason");
      }
    }
    else {
      QMessageBox::information(nullptr, "publish result", "No goal path to publish!");
    }
  }
}

void PublishGoalTool::deactivate()
{

}
} // end namespace rviz
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::PublishGoalTool, rviz::Tool )
