#include "cancel_task_tool.h"

#include "task_control_service.h"

namespace rviz {

void CancelTaskTool::onInitialize() {
  Tool::onInitialize();
  setName( "Cancel" );
}

void CancelTaskTool::activate()
{
  TaskControlService::get_instance()->Cancel();
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::CancelTaskTool, rviz::Tool )
