#include "goal_custom_tool.h"

#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/MoveBaseAction.h>

#include <QFileDialog>
#include <QMessageBox>

#include "base_tools/base_math.h"
#include "path/pose.h"
#include "path/path_2d.h"


namespace rviz {

GoalCustomTool::GoalCustomTool()
{

}

void GoalCustomTool::SaveToFile(const std::vector<geometry_msgs::PoseStamped> &goal_path, bool is_silent)
{
  QFileDialog file_diag;
  file_diag.setWindowTitle("Record Goal...");
  file_diag.setNameFilter("*.gpath");
  file_diag.setFileMode(QFileDialog::ExistingFile);
  file_diag.setOption(QFileDialog::DontUseNativeDialog, true);
  QString filename = LoadLastFilePath();
  QIODevice::OpenMode open_mode = QFile::Append;
  if(!(is_silent)) {
    file_diag.setDirectory(LoadLastFileDir());
    int result = file_diag.exec();
    open_mode = QFile::WriteOnly;
    if(QFileDialog::FileName == result || QFileDialog::Accept == result) {
      filename = file_diag.selectedFiles()[0];
      UpdateLastFilePath(file_diag);
      file_diag.close();
    }
    else {
//      QMessageBox::information(nullptr, "record result", "Choose no valid file!");
      return;
    }
  }

  QFile save_file(filename);
  save_file.open(open_mode);
  QString path_string;
  const int kDoublePrecision = 4;
  const QString kDilimiter = "\t";
  for(std::vector<geometry_msgs::PoseStamped>::const_iterator iter = goal_path.begin(); iter != goal_path.end(); ++iter) {
    path_string += (*iter).header.frame_id.data() + kDilimiter
        + QString::number((*iter).pose.position.x, 10, kDoublePrecision) + kDilimiter
        + QString::number((*iter).pose.position.y, 10, kDoublePrecision) + kDilimiter
        + QString::number((*iter).pose.position.z, 10, kDoublePrecision) + kDilimiter
        + QString::number((*iter).pose.orientation.x, 10, kDoublePrecision) + kDilimiter
        + QString::number((*iter).pose.orientation.y, 10, kDoublePrecision) + kDilimiter
        + QString::number((*iter).pose.orientation.z, 10, kDoublePrecision) + kDilimiter
        + QString::number((*iter).pose.orientation.w, 10, kDoublePrecision) + "\n";
  }
  save_file.write(path_string.toLatin1().data());
  save_file.close();
  if(!is_silent) {
    QMessageBox::information(nullptr, "record result", "Successfully record all goal path to file!");
  }
}

QString GoalCustomTool::LoadLastFilePath()
{
  QString last_file_path = QDir::homePath();
  QFile rviz_persitent_config_file(QDir::homePath() + "/.rviz/last_gpath_file.cfg");
  rviz_persitent_config_file.open(QFile::ReadOnly);
  QString file_path = rviz_persitent_config_file.readLine();
  QFileInfo file_info(file_path);
  if(file_info.exists()) {
    last_file_path = file_info.absoluteFilePath();
  }
  rviz_persitent_config_file.close();
  return last_file_path;
}

QString GoalCustomTool::LoadLastFileDir()
{
  QString last_file_path = QDir::homePath();
  QFile rviz_persitent_config_file(QDir::homePath() + "/.rviz/last_gpath_file.cfg");
  rviz_persitent_config_file.open(QFile::ReadOnly);
  QString file_path = rviz_persitent_config_file.readLine();
  QFileInfo file_info(file_path);
  if(file_info.exists()) {
    last_file_path = file_info.absolutePath();
  }
  rviz_persitent_config_file.close();
  return last_file_path;
}

void GoalCustomTool::UpdateLastFilePath(const QFileDialog &file_dialog)
{
  //Save the current open file path
  QString filename = file_dialog.selectedFiles().back();
  QFile rviz_persitent_config_file(QDir::homePath() + "/.rviz/last_gpath_file.cfg");
  rviz_persitent_config_file.open(QFile::WriteOnly);
  rviz_persitent_config_file.write(filename.toLatin1().data());
  rviz_persitent_config_file.close();
}

void GoalCustomTool::Publish(const std::vector<geometry_msgs::PoseStamped> &goal_path)
{
  mbf_msgs::ExePathGoal target_path;
  nav_msgs::Path path;
  path.poses = goal_path;
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
}

void GoalCustomTool::GetPathPoint(const traj_tools::Path2d &path,
                                  double precision,
                                  const std_msgs::Header &header,
                                  std::vector<geometry_msgs::PoseStamped> &path_points)
{
  Float length = path.length();
  Float cur_pos = 0.0;

  geometry_msgs::PoseStamped pose_stamped;
  geometry_msgs::Pose pose_msg;
  while(cur_pos < length) {
    traj_tools::Pose init_pose = path.GetPose(cur_pos);

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
} //namespace rviz
