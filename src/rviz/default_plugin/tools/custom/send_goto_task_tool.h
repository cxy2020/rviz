#ifndef SEND_GOTO_TASK_TOOL_H
#define SEND_GOTO_TASK_TOOL_H

# include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>
# include "rviz/default_plugin/tools/pose_tool.h"
#endif

namespace rviz
{
class Arrow;
class DisplayContext;
class StringProperty;

class SendGotoTaskTool: public PoseTool
{
Q_OBJECT
public:
  SendGotoTaskTool();
  virtual ~SendGotoTaskTool() {}
  virtual void onInitialize();

protected:
  virtual void onPoseSet(double x, double y, double theta);

private Q_SLOTS:
  void updateTopic();

private:
  void SaveGoalPath(const ros::TimerEvent& event);

  ros::NodeHandle nh_;
  ros::Publisher pub_;

  StringProperty* topic_property_;
  std::vector<geometry_msgs::PoseStamped> poses_to_save_;

  ros::Timer save_file_timer_;
  bool is_file_saved_;
};

}

#endif // SEND_GOTO_TASK_TOOL_H

