/***************************************************************************************************
 * Copyright (C) 2020, Roborock, Inc.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 *    and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************************************/

/**
 * @file zone_selection_tool.h
 * @author Xiaoying Chen
 */

#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

#include "rviz/tool.h"
#include "rviz/default_plugin/custom_tools/zone.h"

namespace Ogre {
class Viewport;
}

namespace rviz {
class MoveTool;
}

namespace rock {
namespace custom_tools {

/**
 * @class ZoneSelectionTool
 * @brief The base tool for zone selection. When the user select a zone with UI, this class
 *  finds the corners of the zone on the map. The zone can be rotated with pose direction.
 */
class ZoneSelectionTool : public rviz::Tool {
public:
  ZoneSelectionTool();
  ~ZoneSelectionTool() override;

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz::ViewportMouseEvent& event) override;
  int processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) override;

  void update(float wall_dt, float ros_dt) override;

  bool getSelectedRectangle(Rectangle& rectangle) const;

protected:
  /**
   * @enum SelectStage
   * @brief The stages of selection.
   */
  enum SelectStage {
    NO_SELECTION = 0,
    SELECTING,
    SELECTED
  };

  /**
   * @brief Transform the coordinates of the mouse event to those on the map.
   * @param event The mouse event containt the coordinates.
   * @param event_point The point from the view port event
   * @param map_point the output point on the map.
   * @return Return true if the coordinate is valid; return false otherwise.
   */
  static bool transform_to_map(rviz::ViewportMouseEvent& event,
                               int event_point[2],
                               V2f& map_point);

  /**
   * @brief Calculate the vertexes of rectangle according to the selected points on the diagonal of
   *  the rectangle in the mouse event.
   * @param event The mouse event.
   * @param vertex_0 The first vertex of the diagonal in the mouse event.
   * @param vertex_2 The second vertex of the diagonal in the mouse event.
   * @param is_clock_wise Indicates the generated rectangle is clockwise or not.
   * @param map_vertexes The output vertexes of the rectangle
   */
  static void calculate_map_vertexes(rviz::ViewportMouseEvent& event,
                                     int vertex_0[2],
                                     int vertex_2[2],
                                     bool is_clock_wise,
                                     V2f map_vertexes[4]);

  rviz::MoveTool* move_tool_;
  SelectStage select_stage_;
  int sel_start_[2];
  bool is_clockwise_;
  V2f vertexes_[4];

private:
  static void calculate_vertexes(int vertex_0[2], int vertex_2[2],
                                 bool is_clock_wise, int vertex_1[2], int vertex_3[2]);

  /**
   * @brief Given the neighbour points A, B, C to form the rectangle, adjust the second point B to
   *  make BA perpendicular to BC.
   * @param A The first point to form the rectangle.
   * @param C The second point to form the rectangle.
   * @param B The third point to form the rectangle and to be adjusted.
   */
  static void adjust_vertex(const V2f& A, const V2f& C, V2f& B);
};

}   //namespace custom_tools
}   //namespace rock
