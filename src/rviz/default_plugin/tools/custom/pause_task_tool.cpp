#include "pause_task_tool.h"

#include "task_control_service.h"

namespace rviz {

void PauseTaskTool::onInitialize() {
  Tool::onInitialize();
  setName( "Pause" );
}

void PauseTaskTool::activate()
{
  TaskControlService::get_instance()->Pause();
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::PauseTaskTool, rviz::Tool )
