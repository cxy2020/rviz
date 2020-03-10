#ifndef RVIZ_CUSTOM_RECORD_GOAL_TOOL_H_
#define RVIZ_CUSTOM_RECORD_GOAL_TOOL_H_

#include <vector>
#include <geometry_msgs/PoseStamped.h>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>
# include "rviz/default_plugin/tools/pose_tool.h"
#endif

class QFileDialog;
namespace rviz
{

class RecordGoalTool: public PoseTool
{
Q_OBJECT
public:
  RecordGoalTool();
  virtual ~RecordGoalTool() {}
  virtual void onInitialize();

  virtual int processMouseEvent( ViewportMouseEvent& event );

  virtual void deactivate();

  static QString LoadLastFilePath();

  static void UpdateLastFilePath(const QFileDialog &file_dialog);

  static void SaveToFile(const std::vector<geometry_msgs::PoseStamped>& gola_path);

  inline const std::vector<geometry_msgs::PoseStamped>& goal_path() const {
    return goal_path_;
  }

  inline void clear_goal_path() {
    goal_path_.clear();
  }

protected:
  virtual void onPoseSet(double x, double y, double theta);

private:
  std::vector<geometry_msgs::PoseStamped> goal_path_;
};

}

#endif // RVIZ_CUSTOM_RECORD_GOAL_TOOL_H_
