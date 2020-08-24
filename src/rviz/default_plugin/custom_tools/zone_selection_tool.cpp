#include "rviz/default_plugin/custom_tools/zone_selection_tool.h"

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

ZoneSelectionTool::ZoneSelectionTool()
  : Tool()
  , move_tool_(new MoveTool())
  , select_stage_(NO_SELECTION)
  , sel_start_x_(0)
  , sel_start_y_(0)
  , sel_end_x_(0)
  , sel_end_y_(0)
  , is_clockwise_(false) {
  shortcut_key_ = 'z';
  access_all_keys_ = true;
}

ZoneSelectionTool::~ZoneSelectionTool() {
  delete move_tool_;
}

void ZoneSelectionTool::onInitialize() {
  move_tool_->initialize(context_);
}

void ZoneSelectionTool::activate() {
  setStatus("Click and drag to select objects on the screen.");
  context_->getSelectionManager()->setTextureSize(512);
  select_stage_ = NO_SELECTION;
}

void ZoneSelectionTool::deactivate() {
  context_->getSelectionManager()->removeHighlight();
}

void ZoneSelectionTool::update(float /*wall_dt*/, float /*ros_dt*/) {
  SelectionManager* sel_manager = context_->getSelectionManager();
  if (NO_SELECTION == select_stage_) {
    sel_manager->removeHighlight();
  }
}

Zone ZoneSelectionTool::getSelectedZone() const {
  Zone zone;
  zone.start = start_;
  Vec2f v = end_ - start_;
  Vec2f v_x(v[0], 0.0);
  Vec2f v_y(0.0, v[1]);
  bool is_cross_positive = (v[0] > 0.0) ^ (v[1] > 0.0);
  if (is_cross_positive ^ is_clockwise_) {
    zone.v1 = v_x;
    zone.v2 = v_y;
  }
  else {
    zone.v1 = v_y;
    zone.v2 = v_x;
  }
  return zone;
}

bool ZoneSelectionTool::transform_to_map(ViewportMouseEvent& event, rFloat& x, rFloat& y) {
  Ogre::Vector3 point;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x, event.y, point)){
    x = point.x;
    y = point.y;
    return true;
  }
  return false;
}

int ZoneSelectionTool::processMouseEvent(ViewportMouseEvent& event) {
  SelectionManager* sel_manager = context_->getSelectionManager();

  int flags = 0;

  switch (select_stage_) {
  case NO_SELECTION: {
    if (event.leftDown()) {
      select_stage_ = SELECTING;
      sel_start_x_ = event.x;
      sel_start_y_ = event.y;
      sel_end_x_ = event.x;
      sel_end_y_ = event.y;
      transform_to_map(event, start_[0], start_[1]);
      end_ = start_;
    }
  }
    break;
  case SELECTING: {
    sel_manager->removeHighlight();
    flags = move_tool_->processMouseEvent(event);
    sel_end_x_ = event.x;
    sel_end_y_ = event.y;
    transform_to_map(event, end_[0], end_[1]);
    if (event.type == QEvent::MouseButtonRelease) {
      select_stage_ = SELECTED;
    }
  }
    break;
  case SELECTED: {
    if (event.leftDown()) {
      select_stage_ = SELECTING;
      sel_start_x_ = event.x;
      sel_start_y_ = event.y;
      sel_end_x_ = event.x;
      sel_end_y_ = event.y;
      transform_to_map(event, start_[0], start_[1]);
      end_ = start_;
    }
  }
    break;
  }

  if (NO_SELECTION != select_stage_) {
    sel_manager->highlight(event.viewport, sel_start_x_, sel_start_y_, sel_end_x_, sel_end_y_);
  }
  return flags;
}

int ZoneSelectionTool::processKeyEvent(QKeyEvent* event, RenderPanel* /*panel*/) {
  SelectionManager* sel_manager = context_->getSelectionManager();

  if (event->key() == Qt::Key_F) {
    sel_manager->focusOnSelection();
  }

  return Render;
}

}   //namespace custom_tools
}   //namespace rock
