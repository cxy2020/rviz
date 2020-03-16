#include "publish_goal_tool.h"

#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Path.h>
#include <mbf_msgs/ExePathAction.h>

#include <QFileDialog>
#include <QMessageBox>

#include "goal_custom_tool.h"

namespace rviz
{

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
  file_diag.setOption(QFileDialog::DontUseNativeDialog, true);
  file_diag.setDirectory(GoalCustomTool::LoadLastFileDir());
  int result = file_diag.exec();
  if(QFileDialog::FileName == result || QFileDialog::Accept == result) {
    QString filename = file_diag.selectedFiles()[0];
    GoalCustomTool::UpdateLastFilePath(file_diag);
    file_diag.close();
    QFile goal_path_file(filename);
    goal_path_file.open(QFile::ReadOnly);

    nav_msgs::Path path;

    geometry_msgs::PoseStamped goal_pose;
    while (!goal_path_file.atEnd()) {
      QByteArray line = goal_path_file.readLine();
      line = line.trimmed();
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
      mbf_msgs::ExePathGoal target_path;
      target_path.path = path;
      PathClient pc("move_base_flex/exe_path", true); // true doesnt need ros::spin

      int try_times = 0;
      while(!pc.waitForServer(ros::Duration(1.0))){
        ROS_INFO("Waiting for Move Base server to come up");
        if(++try_times > 2) {
          QMessageBox::information(nullptr, "connect mbf server", "cannot connect mbf server!");
          return;
        }
      }
      pc.sendGoal(target_path);

      QMessageBox::information(nullptr, "publish result", "Path Goal sent to mbf server!");
    }
  }
  else {
    QMessageBox::information(nullptr, "publish result", "Choose no valid file!");
    return;
  }
}

void PublishGoalTool::deactivate()
{

}
} // end namespace rviz
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::PublishGoalTool, rviz::Tool )
