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
 * @file multi_zone_selection_tool.h
 * @author Xiaoying Chen
 */

#pragma once

#include "rviz/default_plugin/custom_tools/zone_selection_tool.h"
#include "rviz/default_plugin/custom_tools/zone.h"

namespace rock {
namespace custom_tools {

/**
 * @class MultiZoneSelectionTool
 * @brief The tool for selecting multiple zones. The zone selection starts with the tool button
 *  checked. The user can select multiple zones by pressing and moving the left mouse. When the
 *  selection is finished, the user can clicked on the tool button to deactivate it. If The user
 *  clicks on the empty space, the selection will be canceled and all the selection zones are cleared.
 */
class MultiZoneSelectionTool : public ZoneSelectionTool {
public:
  MultiZoneSelectionTool();

  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

  void update(float wall_dt, float ros_dt) override;

private:
  inline void clear_selected();
  std::vector<Rectangle> zones_;
};

}   //namespace custom_tools
}   //namespace rock
