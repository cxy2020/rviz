#ifndef RVIZ_CUSTOM_RECORD_GOAL_TOOL_H_
#define RVIZ_CUSTOM_RECORD_GOAL_TOOL_H_

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>
# include "rviz/tool.h"
#endif

class QFileDialog;
namespace rviz
{
class FloatProperty;
class StringProperty;
class SyncPosTool: public Tool
{
Q_OBJECT
public:
  SyncPosTool();
  virtual ~SyncPosTool() {}
  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  static inline double x() {
    return x_;
  }
  static inline double y() {
    return y_;
  }
  static double theta() {
    return theta_;
  }

  static const geometry_msgs::PoseStamped& GetRobotPose() {
    return current_pose_;
  }

private:
  void ModelStatesUpdated(const gazebo_msgs::ModelStates &msg);

  static double x_;
  static double y_;
  static double theta_;
  static geometry_msgs::PoseStamped current_pose_;

  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::NodeHandle nh_;

  FloatProperty* std_dev_x_;
  FloatProperty* std_dev_y_;
  FloatProperty* std_dev_theta_;
  static bool is_synchronized_;
};

}

#endif // RVIZ_CUSTOM_RECORD_GOAL_TOOL_H_
