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
  , is_clockwise_(true) {
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
  select_stage_ = NO_SELECTION;
}

void ZoneSelectionTool::update(float /*wall_dt*/, float /*ros_dt*/) {
  SelectionManager* sel_manager = context_->getSelectionManager();
  if (NO_SELECTION == select_stage_) {
    sel_manager->removeHighlight();
  }
}

bool ZoneSelectionTool::getSelectedRectangle(Rectangle& rectangle) const {
  if (NO_SELECTION == select_stage_)
    return false;

  for (int i = 0; i < 4; ++i) {
    rectangle.vertexes[i] = vertexes_[i];
  }

  rectangle.v1 = rectangle.vertexes[1] - rectangle.vertexes[0];
  rectangle.v2 = rectangle.vertexes[2] - rectangle.vertexes[1];
  return true;
}

int ZoneSelectionTool::processMouseEvent(ViewportMouseEvent& event) {
  SelectionManager* sel_manager = context_->getSelectionManager();
  int flags = 0;
  switch (select_stage_) {
  case NO_SELECTION: {
    if (event.leftDown()) {
      select_stage_ = SELECTING;
      sel_start_[0] = event.x;
      sel_start_[1] = event.y;
    }
  }
    break;
  case SELECTING: {
    sel_manager->removeHighlight();
    flags = move_tool_->processMouseEvent(event);
    if (event.leftUp()) {
      if (sel_start_[0] == event.x && sel_start_[1] == event.y) {
        select_stage_ = NO_SELECTION;
      }
      else {
        int vertex_2[2] = {event.x, event.y};
        calculate_map_vertexes(event, sel_start_, vertex_2, is_clockwise_, vertexes_);
        select_stage_ = SELECTED;
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
    }
  }
    break;
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

//Transform the coordinates of the mouse event to those on the map.
bool ZoneSelectionTool::transform_to_map(rviz::ViewportMouseEvent& event,
                                         int event_point[2],
                                         Vec2f& map_point) {
  Ogre::Vector3 point;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (getPointOnPlaneFromWindowXY(event.viewport, ground_plane,
                                  event_point[0], event_point[1], point)){
    map_point[0] = point.x;
    map_point[1] = point.y;
    return true;
  }
  return false;
}

void ZoneSelectionTool::calculate_vertexes(int vertex_0[2], int vertex_2[2],
                                           bool is_clock_wise, int vertex_1[2], int vertex_3[2]) {
  int dx = vertex_2[0] - vertex_0[0];
  int dy = vertex_2[1] - vertex_0[1];
  int vec_0_1[2];     //vector from vertex_0 to vertex_1
  int vec_1_2[2];     //vector from vertex_1 to vertex_2
  if ((dx > 0 ^ dy > 0) ^ is_clock_wise) {
    vec_0_1[0] = 0;
    vec_0_1[1] = dy;
    vec_1_2[0] = dx;
    vec_1_2[1] = 0;
  }
  else {
    vec_0_1[0] = dx;
    vec_0_1[1] = 0;
    vec_1_2[0] = 0;
    vec_1_2[1] = dy;
  }
  vertex_1[0] = vertex_0[0] + vec_0_1[0];
  vertex_1[1] = vertex_0[1] + vec_0_1[1];
  vertex_3[0] = vertex_2[0] - vec_0_1[0];
  vertex_3[1] = vertex_2[1] - vec_0_1[1];
}

//Calculate the vertexes of rectangle according to the selected points on the diagonal of the
//rectangle in the mouse event.
void ZoneSelectionTool::calculate_map_vertexes(rviz::ViewportMouseEvent& event,
                                               int vertex_0[2],
                                               int vertex_2[2],
                                               bool is_clock_wise,
                                               Vec2f map_vertexes[4]) {
  int vertex_1[2];
  int vertex_3[2];
  calculate_vertexes(vertex_0, vertex_2, is_clock_wise, vertex_1, vertex_3);
  transform_to_map(event, vertex_0, map_vertexes[0]);
  transform_to_map(event, vertex_1, map_vertexes[1]);
  transform_to_map(event, vertex_2, map_vertexes[2]);
  transform_to_map(event, vertex_3, map_vertexes[3]);
  //Adjust the vertexes to make them form a rectangle.
  //Adjust vertex_1 to make [vertex_0, vertex_1] perpendicular to [vertex_1, vertex_2]
  adjust_vertex(map_vertexes[0], map_vertexes[2], map_vertexes[3]);
  adjust_vertex(map_vertexes[2], map_vertexes[0], map_vertexes[1]);
}

void ZoneSelectionTool::adjust_vertex(const Vec2f& A, const Vec2f& C, Vec2f& B) {
  Vec2f dir_AB = B - A;
  dir_AB.normalize();
  Vec2f dir_BC = Vec2f(-dir_AB[1], dir_AB[0]);
  Vec2f BC = C - B;
  rFloat length = dir_BC.dot(BC);
  Vec2f adjust_BC = dir_BC * length;
  B = C - adjust_BC;
}

}   //namespace custom_tools
}   //namespace rock
