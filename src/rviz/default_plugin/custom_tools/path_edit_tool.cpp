#include "rviz/default_plugin/custom_tools/path_edit_tool.h"

#include <ros/ros.h>

#include <QFileDialog>

#include "rviz/display_context.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/render_panel.h"
#include "rviz/default_plugin/custom_tools/path_manager.h"
#include "rviz/selection/selection_manager.h"

using namespace rock::nav_tools;

namespace rock {
namespace custom_tools {

PathEditTool::PathEditTool() : is_operation_key_pressed_(false) {}

int PathEditTool::processMouseEvent(rviz::ViewportMouseEvent& event) {
  if (is_operation_key_pressed_ && event.leftDown()) {
    deactivate();
    is_operation_key_pressed_ = false;
  }
  return MultiZoneSelectionTool::processMouseEvent(event);
}

int PathEditTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) {
  PathManager& path_manager = PathManager::get_instance();
  switch (event->key()) {
  case Qt::Key_Q: {
    deactivate();
  }
    break;
  //Generate the path in the selected zone.
  case Qt::Key_G: {
    Rectangle rectangle;
    std::vector<boost::shared_ptr<AnalyticPath2d> > paths;
    paths.clear();
    if (getSelectedRectangle(rectangle)) {
      path_manager.GeneratePath(rectangle, paths);
      path_manager.AddPath(paths, rectangle);
      path_manager.PublishPath(context_->getFixedFrame().toStdString());
    }
    is_operation_key_pressed_ = true;
  }
    break;
  //Delete the paths in the selected zone.
  case Qt::Key_D: {
    Rectangle rectangle;
    if (getSelectedRectangle(rectangle)) {
      path_manager.DeletePath(rectangle);
      path_manager.PublishPath(context_->getFixedFrame().toStdString());
    }
    is_operation_key_pressed_ = true;
  }
    break;
  //Save all the generated paths.
  case Qt::Key_S: {
    QFileDialog file_dialog(panel);
    file_dialog.setWindowTitle("Save global path");
    file_dialog.setNameFilter("*.gpath");
    file_dialog.setFileMode(QFileDialog::AnyFile);
    file_dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    QString dir = path_manager.default_file_dir().c_str();
    file_dialog.setDirectory(dir);
    int result = file_dialog.exec();
    QString file_path;
    if (QFileDialog::FileName == result) {
      file_path = file_dialog.selectedFiles()[0];
      if (!file_path.contains(".gpath")) {
        file_path +=  ".gpath";
      }
      path_manager.SavePath(file_path.toStdString());
    }
    else if(QFileDialog::Accept == result) {
      file_path = file_dialog.selectedFiles()[0];
      path_manager.SavePath(file_path.toStdString());
    }
    is_operation_key_pressed_ = true;
  }
    break;
  //Send the paths in the selected zone.
  case Qt::Key_T: {
    path_manager.SendPath(context_->getFixedFrame().toStdString());
    is_operation_key_pressed_ = true;
  }
    break;
  //Load global path from file.
  case Qt::Key_L: {
    QFileDialog file_dialog(panel);
    file_dialog.setWindowTitle("Load global path");
    file_dialog.setNameFilter("*.gpath");
    file_dialog.setFileMode(QFileDialog::ExistingFile);
    file_dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    QString dir = path_manager.default_file_dir().c_str();
    file_dialog.setDirectory(dir);
    int result = file_dialog.exec();
    QString file_path;
    if (QFileDialog::FileName == result || QFileDialog::Accept == result) {
      file_path = file_dialog.selectedFiles()[0];
      path_manager.Load(file_path.toStdString());
      path_manager.PublishPath(context_->getFixedFrame().toStdString());
    }
    is_operation_key_pressed_ = true;
  }
    break;
  default:
    break;
  }

  TaskControlTool::ProcessControlTaskEvent(event, panel);
  return Render;
}

}   //namespace custom_tools
}   //namespace rock

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rock::custom_tools::PathEditTool, rviz::Tool)
