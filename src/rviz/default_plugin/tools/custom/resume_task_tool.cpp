#include "resume_task_tool.h"

#include "task_control_service.h"

namespace rviz {

void ResumeTaskTool::onInitialize() {
  Tool::onInitialize();
  setName( "Resume" );
}

void ResumeTaskTool::activate()
{
  TaskControlService::get_instance()->Resume();
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::ResumeTaskTool, rviz::Tool )
