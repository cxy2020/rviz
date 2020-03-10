#ifndef RVIZ_CUSTOM_PUBLISH_GOAL_TOOL_H_
#define RVIZ_CUSTOM_PUBLISH_GOAL_TOOL_H_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>
# include "rviz/tool.h"
#endif

#include <geometry_msgs/PoseStamped.h>

namespace rviz
{

class PublishGoalTool: public Tool
{
Q_OBJECT
public:
  PublishGoalTool();
  virtual ~PublishGoalTool() {}
  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  static void Publish(const std::vector<geometry_msgs::PoseStamped>& goal_path);
};

}

#endif // RVIZ_CUSTOM_PUBLISH_GOAL_TOOL_H_
