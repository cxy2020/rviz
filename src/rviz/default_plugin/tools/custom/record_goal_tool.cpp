#include <tf/transform_listener.h>

#include <QFileDialog>
#include <QMessageBox>

#include "rviz/viewport_mouse_event.h"

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"

#include "record_goal_tool.h"

namespace rviz
{

RecordGoalTool::RecordGoalTool()
{
  shortcut_key_ = 's';
}

void rviz::RecordGoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName( "RecordGoal" );
}

int RecordGoalTool::processMouseEvent( ViewportMouseEvent& event )
{
  int flags = PoseTool::processMouseEvent(event);

  if( event.leftUp() )
  {
    if( state_ == Orientation )
    {
      //Keep active state after left mouse up
      int finished_flags = ~Finished;
      flags &= finished_flags;
      PoseTool::activate();
    }
  }
  return flags;
}

void RecordGoalTool::deactivate()
{
  PoseTool::deactivate();
  if(goal_path_.size() > 0) {
    QFileDialog file_diag;
    file_diag.setWindowTitle("Record Goal...");
    file_diag.setNameFilter("*.gpath");
    file_diag.setFileMode(QFileDialog::ExistingFile);
    int result = file_diag.exec();
    if(QFileDialog::FileName == result || QFileDialog::Accept == result) {
      QString filename = file_diag.selectedFiles()[0];
      QFile save_file(filename);
      save_file.open(QFile::WriteOnly);
      QString path_string;
      const int kDoublePrecision = 4;
      const QString kDilimiter = "\t";
      for(std::vector<geometry_msgs::PoseStamped>::iterator iter = goal_path_.begin(); iter != goal_path_.end(); ++iter) {
        path_string += (*iter).header.frame_id.data() + kDilimiter
            + QString::number((*iter).pose.position.x, 10, kDoublePrecision) + kDilimiter
            + QString::number((*iter).pose.position.y, 10, kDoublePrecision) + kDilimiter
            + QString::number((*iter).pose.position.z, 10, kDoublePrecision) + kDilimiter
            + QString::number((*iter).pose.orientation.x, 10, kDoublePrecision) + kDilimiter
            + QString::number((*iter).pose.orientation.y, 10, kDoublePrecision) + kDilimiter
            + QString::number((*iter).pose.orientation.z, 10, kDoublePrecision) + kDilimiter
            + QString::number((*iter).pose.orientation.w, 10, kDoublePrecision) + "\n";
      }
      save_file.write(path_string.toLatin1().data());
      save_file.close();
      QMessageBox::information(nullptr, "record result", "Successfully record all goal path to file!");
    }
  }
  goal_path_.clear();
}

void rviz::RecordGoalTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal;
  tf::poseStampedTFToMsg(p, goal);
  ROS_INFO("Saving goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
      goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
  goal_path_.push_back(goal);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::RecordGoalTool, rviz::Tool )
