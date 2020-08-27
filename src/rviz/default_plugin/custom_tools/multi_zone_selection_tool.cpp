#include "rviz/default_plugin/custom_tools/multi_zone_selection_tool.h"

#include <QKeyEvent>

#include <OgreRay.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>
#include <OgreMovableObject.h>
#include <OgreRectangle2D.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <OgreMaterialManager.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include <ros/time.h>

#include "tools/move_tool.h"

#include "rviz/ogre_helpers/camera_base.h"
#include "rviz/ogre_helpers/qt_ogre_render_window.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"
#include "rviz/geometry.h"

using namespace rviz;

namespace rock {
namespace custom_tools {

MultiZoneSelectionTool::MultiZoneSelectionTool()
  : ZoneSelectionTool() {
  shortcut_key_ = 'z';
  access_all_keys_ = true;
}

void MultiZoneSelectionTool::activate() {
  ZoneSelectionTool::activate();
  clear_selected();
}

void MultiZoneSelectionTool::deactivate() {
  clear_selected();
  ZoneSelectionTool::deactivate();
}

void MultiZoneSelectionTool::update(float wall_dt, float ros_dt) {
  ZoneSelectionTool::update(wall_dt, ros_dt);
}

void MultiZoneSelectionTool::clear_selected() {
  select_stage_ = NO_SELECTION;
  zones_.clear();
  SelectionManager* sel_manager = context_->getSelectionManager();
  sel_manager->removeHighlight();
}

int MultiZoneSelectionTool::processMouseEvent(ViewportMouseEvent& event) {
  SelectionManager* sel_manager = context_->getSelectionManager();
  int flags = 0;
  switch (select_stage_) {
  case NO_SELECTION: {
    if (event.leftDown()) {
      select_stage_ = SELECTING;
      sel_start_[0] = event.x;
      sel_start_[1] = event.y;
      sel_manager->addHighlight(event.viewport, sel_start_[0], sel_start_[1], event.x, event.y);
    }
  }
    break;
  case SELECTING: {
    flags = move_tool_->processMouseEvent(event);
    if (event.leftUp()) {
      if (sel_start_[0] == event.x && sel_start_[1] == event.y) {
        clear_selected();
        break;
      }
      else {
        select_stage_ = SELECTED;
        int vertex_2[2] = {event.x, event.y};
        calculate_map_vertexes(event, sel_start_, vertex_2, is_clockwise_, vertexes_);
        Rectangle rectangle;
        if (getSelectedRectangle(rectangle)) {
          zones_.emplace_back(rectangle);
        }
      }
    }
    sel_manager->highlight(event.viewport, sel_start_[0], sel_start_[1], event.x, event.y);
  }
    break;
  case SELECTED: {
    if (event.leftDown()) {
      select_stage_ = SELECTING;
      sel_start_[0] = event.x;
      sel_start_[1] = event.y;
      sel_manager->addHighlight(event.viewport, sel_start_[0], sel_start_[1], event.x, event.y);
    }
  }
    break;
  }
  return flags;
}

}   //namespace custom_tools
}   //namespace rock
