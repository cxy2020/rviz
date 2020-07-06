#include "task_control_service.h"

#include <QMessageBox>

#include "sync_pos_tool.h"

#include "nav_task_msgs/Path.h"
#include "task_msgs/SetTaskInfo.h"
#include "task_msgs/GetBool.h"

namespace rviz
{

TaskControlService* TaskControlService::instance_ = nullptr;

TaskControlService::TaskControlService()
{
  add_task_client_ = nh_.serviceClient<task_msgs::SetTaskInfo>("/task_control/add_task");
  set_task_client_ = nh_.serviceClient<task_msgs::SetTaskInfo>("/task_control/set_task");
  pause_client_ = nh_.serviceClient<task_msgs::GetBool>("/task_control/pause_task");
  resume_client_ = nh_.serviceClient<task_msgs::GetBool>("/task_control/resume_task");
  cancel_client_ = nh_.serviceClient<task_msgs::GetBool>("/task_control/cancel_task");
  start_pose_ = SyncPosTool::GetRobotPose();
}

bool TaskControlService::AddGotoTask(const geometry_msgs::PoseStamped& goal) {
  task_msgs::SetTaskInfoRequest req;
  task_msgs::SetTaskInfoResponse res;

  req.path.poses.push_back(goal);
  req.path.path_type = nav_task_msgs::Path::GOTO;
  req.config_id = 0;

  if (!add_task_client_.call(req, res)) {
    ROS_ERROR("Call add_task(goto) service failed!");
    QMessageBox::information(nullptr, "error", "Call add_task(goto) service failed!");
    return false;
  }

  return res.result;
}

bool TaskControlService::SetGotoTask(const geometry_msgs::PoseStamped& goal) {
  task_msgs::SetTaskInfoRequest req;
  task_msgs::SetTaskInfoResponse res;

  req.path.poses.push_back(goal);
  req.path.path_type = nav_task_msgs::Path::GOTO;
  req.config_id = 0;

  if (!set_task_client_.call(req, res)) {
    QMessageBox::information(nullptr, "error", "Call set_task(goto) service failed!");
    ROS_ERROR("Call set_task(goto) service failed!");
    return false;
  }

  return res.result;
}

bool TaskControlService::SetExePathTask(const PoseVec& poses)
{
  task_msgs::SetTaskInfoRequest req;
  task_msgs::SetTaskInfoResponse res;

  req.path.poses = poses;
  req.path.path_type = nav_task_msgs::Path::EXE_PATH;
  req.config_id = 0;

  if (!set_task_client_.call(req, res)) {
    QMessageBox::information(nullptr, "error", "Call set_task(goto) service failed!");
    ROS_ERROR("Call set_task(exe_path) service failed!");
    return false;
  }

  return res.result;
}

bool TaskControlService::AddExePathTask(const PoseVec& poses)
{
  task_msgs::SetTaskInfoRequest req;
  task_msgs::SetTaskInfoResponse res;

  req.path.poses = poses;
  req.path.path_type = nav_task_msgs::Path::EXE_PATH;
  req.config_id = 0;

  if (!add_task_client_.call(req, res)) {
    QMessageBox::information(nullptr, "error", "Call add_task(exe_path) service failed!");
    ROS_ERROR("Call add_task(exe_path) service failed!");
    return false;
  }

  return res.result;
}

bool TaskControlService::Pause()
{
  task_msgs::GetBoolRequest req;
  task_msgs::GetBoolResponse res;

  if (!pause_client_.call(req, res)) {
    QMessageBox::information(nullptr, "error", "Call Pause service failed!");
    ROS_ERROR("Call Pause service failed!");
    return false;
  }

  return res.result;
}

bool TaskControlService::Resume()
{
  task_msgs::GetBoolRequest req;
  task_msgs::GetBoolResponse res;

  if (!resume_client_.call(req, res)) {
    QMessageBox::information(nullptr, "error", "Call Resume service failed!");
    ROS_ERROR("Call Resume service failed!");
    return false;
  }

  return res.result;
}

bool TaskControlService::Cancel()
{
  task_msgs::GetBoolRequest req;
  task_msgs::GetBoolResponse res;

  if (!cancel_client_.call(req, res)) {
    QMessageBox::information(nullptr, "error", "Call Cancel service failed!");
    ROS_ERROR("Call Cancel service failed!");
    return false;
  }

  return res.result;
}

}
