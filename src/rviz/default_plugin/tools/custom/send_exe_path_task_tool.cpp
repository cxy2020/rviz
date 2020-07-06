#include "send_exe_path_task_tool.h"

#include "task_control_service.h"

namespace rviz {

void SendExePathTaskTool::onInitialize() {
  PoseTool::onInitialize();
  setName( "SendExePath" );
}

void SendExePathTaskTool::deactivate()
{
  PoseTool::deactivate();
  std::vector<geometry_msgs::PoseStamped> goal_path;
  GenerateGoalPath(goal_path, false);
  TaskControlService::get_instance()->AddExePathTask(goal_path);
}

}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::SendExePathTaskTool, rviz::Tool )
