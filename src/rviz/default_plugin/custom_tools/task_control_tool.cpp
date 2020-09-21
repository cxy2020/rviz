#include "rviz/default_plugin/custom_tools/task_control_tool.h"

#include <QKeyEvent>
#include <std_srvs/Empty.h>

#include "rviz/render_panel.h"
#include "rviz/tool.h"

namespace rock {
namespace custom_tools {

TaskControlTool::TaskControlTool() {
  ros::NodeHandle nh("task_control");
  start_task_client_ = nh.serviceClient<std_srvs::Empty>("resume_task");
  pause_task_client_ = nh.serviceClient<std_srvs::Empty>("pause_task");
  cancel_task_client_ = nh.serviceClient<std_srvs::Empty>("cancel_task");
  emergency_stop_client_ = nh.serviceClient<std_srvs::Empty>("emergency_stop");
}

int TaskControlTool::ProcessControlTaskEvent(QKeyEvent* event, rviz::RenderPanel* /*panel*/) {
  switch (event->key()) {
  //Start the task control process.
  case Qt::Key_R: {
    std_srvs::EmptyRequest req;
    std_srvs::EmptyResponse res;
    start_task_client_.call(req, res);
  }
    break;
  //Pause the task control process.
  case Qt::Key_P: {
    std_srvs::EmptyRequest req;
    std_srvs::EmptyResponse res;
    pause_task_client_.call(req, res);
  }
    break;
  //Stop the task control immediately.
  case Qt::Key_E: {
    std_srvs::EmptyRequest req;
    std_srvs::EmptyResponse res;
    emergency_stop_client_.call(req, res);
  }
    break;
  //Can the task control tasks.
  case Qt::Key_C: {
    std_srvs::EmptyRequest req;
    std_srvs::EmptyResponse res;
    cancel_task_client_.call(req, res);
  }
    break;
  default:
    break;
  }
  return rviz::Tool::Render;
}

}   //namespace custom_tools
}   //namespace rock
