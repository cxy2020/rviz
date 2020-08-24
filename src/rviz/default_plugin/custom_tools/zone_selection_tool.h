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
#include "rviz/selection/forwards.h"
#include "nav_tools/common/base_data_type.h"

namespace Ogre {
class Viewport;
}

namespace rviz {
class MoveTool;
}

namespace rock {
namespace custom_tools {

/**
 * @struct Zone
 * @brief The struct containing geometry information of zone.
 */
struct Zone {
  //The first vertex of the rectangle
  Vec2f start;
  //The vector starting from the first vertex and ending at the second vertex.
  Vec2f v1;
  //The vector starting from the second vertex and ending at the third vertex.
  Vec2f v2;
};

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

  Zone getSelectedZone() const;

private:
  /**
   * @enum SelectStage
   * @brief The stages of selection.
   */
  enum SelectStage {
    NO_SELECTION = 0,
    SELECTING,
    SELECTED
  };

  inline bool transform_to_map(rviz::ViewportMouseEvent& event, rFloat& x, rFloat& y);

  rviz::MoveTool* move_tool_;

  SelectStage select_stage_;
  int sel_start_x_;
  int sel_start_y_;
  int sel_end_x_;
  int sel_end_y_;
  bool is_clockwise_;
  Vec2f start_;
  Vec2f end_;
};

}   //namespace custom_tools
}   //namespace rock
