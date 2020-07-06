#ifndef GOAL_CUSTOM_TOOL_H_
#define GOAL_CUSTOM_TOOL_H_

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/ExePathAction.h>

#include <QString>

class QFileDialog;

namespace traj_tools {
  class Path2d;
}

namespace rviz
{
typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction> PathClient;
class GoalCustomTool
{
public:
  GoalCustomTool();
  static void SaveToFile(const std::vector<geometry_msgs::PoseStamped>& goal_path, bool is_silent = false);

  static QString LoadLastFilePath();

  static QString LoadLastFileDir();

  static void UpdateLastFilePath(const QFileDialog &file_dialog);

  static void Publish(const std::vector<geometry_msgs::PoseStamped>& goal_path);

  static void GetPathPoint(const traj_tools::Path2d &path,
                           double precision,
                           const std_msgs::Header &header,
                           std::vector<geometry_msgs::PoseStamped> &path_points);
};
}

#endif // GOAL_CUSTOM_TOOL_H_
