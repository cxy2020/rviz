#ifndef RVIZ_CUSTOM_GENERATE_CLEAN_PATH_TOOL_H_
#define RVIZ_CUSTOM_GENERATE_CLEAN_PATH_TOOL_H_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>
# include "rviz/default_plugin/tools/custom/record_goal_tool.h"
#endif

//#include <Eigen/Dense>
//#include "base_tools/base_type.h"

namespace traj_tools {
  class Path2d;
}
namespace rviz
{

class GenerateCleanPathTool: public RecordGoalTool
{
Q_OBJECT
public:
  GenerateCleanPathTool();
  virtual ~GenerateCleanPathTool() {}
  virtual void onInitialize();

  virtual void deactivate();

  static void GetPathPoint(const traj_tools::Path2d &path,
                           double precision,
                           const std_msgs::Header &header,
                           std::vector<geometry_msgs::PoseStamped> &path_points);
//private:
//  static double judge_target_theta(const Vec2& point0,
//                                   Float theta0,
//                                   const Vec2& point1,
//                                   Float theta1);
};

}

#endif // RVIZ_CUSTOM_GENERATE_CLEAN_PATH_TOOL_H_

