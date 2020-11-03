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
 * @file path_manager.h
 * @author Xiaoying Chen
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include "nav_tools/common/singleton_class.h"
#include "rviz/default_plugin/custom_tools/zone.h"

namespace rock {
namespace nav_tools {
class AnalyticPath2d;
struct FitParam;
}
}

namespace rock {
namespace custom_tools {

/**
 * @class PathManager
 * @brief Singleton class for path management.
 */
class PathManager : public nav_tools::SingletonClass<PathManager>  {
public:
  PathManager(const PathManager&) = delete;

  PathManager& operator = (const PathManager&) = delete;

  void AddPath(const std::vector<boost::shared_ptr<nav_tools::AnalyticPath2d> >& paths,
               const Rectangle& rectangle);

  void DeletePath(const Rectangle& rectangle);

  void PublishPath(const std::string& frame_id) const;

  bool SendPath(const std::string& frame_id);

  void SavePath(const std::string& file_path);

  void Load(const std::string& file_path);

  /**
   * @brief Generate cleaning path in the given rectangle.
   * @param rectangle The given rectangle.
   * @param paths The output paths.
   */
  void GeneratePath(const Rectangle& rectangle,
                    std::vector<boost::shared_ptr<nav_tools::AnalyticPath2d> >& paths) const;

  const std::string& default_file_dir() const {
    return default_file_dir_;
  }

private:
  friend class nav_tools::SingletonClass<PathManager>;

  PathManager();

  std::list<GeneratedPath> generated_paths_;
  rFloat max_distance_error_;
  rFloat square_max_distance_error_;
  rFloat max_angle_error_;
  rFloat path_latitude_spacing_;
  rFloat path_longitude_spacing_;
  rFloat max_linear_speed_;
  rFloat max_rotation_speed_;
  nav_tools::FitParam* fit_param_;
  std::string default_file_dir_;
  const std::string kDataDelimiter;
  const std::string kRectangleDelimeter;

  ros::Publisher path_pub_;
  ros::ServiceClient add_task_client_;
};

}   //namespace custom_tools
}   //namespace rock
