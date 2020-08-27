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
 * @file zone.h
 * @author Xiaoying Chen
 */

#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include "nav_tools/common/base_data_type.h"
#include "nav_tools/path/global_point_2d.h"

namespace rock {
namespace custom_tools {

/**
 * @struct Rectangle
 * @brief The struct containing geometry information of rectangle zone.
 */
struct Rectangle {
  //The 4 vertex of the rectangle
  Vec2f vertexes[4];
  //The vector starting from the first vertex and ending at the second vertex.
  Vec2f v1;
  //The vector starting from the second vertex and ending at the third vertex.
  Vec2f v2;

  inline bool is_inside(const Vec2f& point) const {
    Vec2f vec1 = point - vertexes[0];
    if (vec1.dot(v2) < 0.0)
      return false;
    if (vec1.dot(v1) < 0.0)
      return false;
    vec1 = point - vertexes[2];
    if (vec1.dot(v1) > 0.0)
      return false;
    if (vec1.dot(v2) > 0.0)
      return false;
    return true;
  }

  inline bool is_intersect(const Rectangle& rectangle) const {
    for (int i = 0; i < 4; ++i) {
      if (is_inside(rectangle.vertexes[i]))
        return true;
      if (rectangle.is_inside(vertexes[i]))
        return true;
    }
    return false;
  }
};

/**
 * @struct GeneratedPath
 * @brief The struct containing path generated with UI.
 */
struct GeneratedPath {
  std::vector<nav_tools::GlobalPoint2d> path;
  Rectangle rectangle;
};

}   //namespace custom_tools
}   //namespace rock
