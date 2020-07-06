#ifndef TASK_CONTROL_SERVICE_H
#define TASK_CONTROL_SERVICE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace rviz
{
typedef std::vector<geometry_msgs::PoseStamped> PoseVec;

class TaskControlService
{
public:
  static inline TaskControlService* get_instance() {
    if (nullptr == instance_) {
      instance_ = new TaskControlService();
    }
    return instance_;
  }

  void set_start_pose(const geometry_msgs::PoseStamped& pose) {
    start_pose_ = pose;
  }

  bool AddGotoTask(const geometry_msgs::PoseStamped& goal);

  bool SetGotoTask(const geometry_msgs::PoseStamped& goal);

  bool SetExePathTask(const PoseVec& poses);

  bool AddExePathTask(const PoseVec& poses);

  bool Pause();

  bool Resume();

  bool Cancel();

private:
  class SingletonDestructor {
    SingletonDestructor() {}
    ~SingletonDestructor() {
      delete instance_;
    }
  };
  TaskControlService();

  static TaskControlService* instance_;
  static SingletonDestructor destructor_;
  ros::NodeHandle nh_;
  ros::ServiceClient add_task_client_;
  ros::ServiceClient set_task_client_;
  ros::ServiceClient pause_client_;
  ros::ServiceClient resume_client_;
  ros::ServiceClient cancel_client_;

  geometry_msgs::PoseStamped start_pose_;
};
}
#endif // TASK_CONTROL_SERVICE_H
