#ifndef RVIZ_CUSTOM_GENERATE_CLEAN_PATH_TOOL_H_
#define RVIZ_CUSTOM_GENERATE_CLEAN_PATH_TOOL_H_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>
# include "rviz/default_plugin/tools/pose_tool.h"
#endif

#include <geometry_msgs/PoseStamped.h>

namespace rviz
{

class GenerateCleanPathTool : public PoseTool
{
Q_OBJECT
public:
  GenerateCleanPathTool();
  virtual ~GenerateCleanPathTool() {}
  virtual void onInitialize();

  virtual int processMouseEvent( ViewportMouseEvent& event );

  virtual void deactivate();

protected:
  virtual void onPoseSet(double x, double y, double theta);

private:
  std::vector<geometry_msgs::PoseStamped> goal_path_;
};

}

#endif // RVIZ_CUSTOM_GENERATE_CLEAN_PATH_TOOL_H_

