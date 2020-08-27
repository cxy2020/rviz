#include "rviz/default_plugin/custom_tools/path_manager.h"

#include <sys/stat.h>
#include <fstream>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

#include "nav_tools/math/base_math.h"
#include "nav_tools/path/line_2d.h"
#include "nav_tools/path/composite_clothoid_2d.h"
#include "nav_tools/path/local_point_2d.h"
#include "nav_tools/nav/navigation_config.h"
#include "nav_tools/common/string_tools.h"
#include "nav_tools/nav/path_type.h"
#include "task_msgs/SendTaskInfo.h"

using namespace rock::nav_tools;

namespace rock {
namespace custom_tools {

void PathManager::AddPath(const std::vector<boost::shared_ptr<nav_tools::AnalyticPath2d> >& paths,
                           const Rectangle& rectangle) {
  static const FitParam* fit_param = &NavigationConfig::get_instance().fit_param;
  const rFloat max_v = NavigationConfig::get_instance().max_wheel_speed;
  const rFloat max_w = (max_v + max_v) / fit_param->wheel_spacing();

  GeneratedPath generated_path;
  std::vector<LocalPoint2d> local_points;
  for (std::vector<boost::shared_ptr<nav_tools::AnalyticPath2d> >::const_iterator i = paths.begin();
       i != paths.end(); ++i) {
    (*i)->Discretize(*fit_param, local_points);
  }

  //Transform local point to global points
  for (LocalPoint2d::ConstIter i = local_points.begin(); i != local_points.end(); ++i) {
    GlobalPoint2d global_point(
          i->coord, i->dir, i->angle, i->k, i->s, max_v, max_w,
          max_angle_error_, max_distance_error_, square_max_distance_error_, true);
    generated_path.path.emplace_back(global_point);
  }
  generated_path.rectangle = rectangle;
  generated_paths_.emplace_back(generated_path);
}

void PathManager::DeletePath(const Rectangle& rectangle) {
  std::list<GeneratedPath>::iterator i = generated_paths_.begin();
  while (generated_paths_.end() != i) {
    if (i->rectangle.is_intersect(rectangle)) {
      if (generated_paths_.begin() == i) {
        generated_paths_.pop_front();
        i = generated_paths_.begin();
        continue;
      }
      else {
        std::list<GeneratedPath>::iterator last_i = i;
        --last_i;
        generated_paths_.erase(i);
        i = last_i;
      }
    }
    ++i;
  }
}

void PathManager::PublishPath(const std::string& frame_id) const {
  nav_msgs::Path nav_path;
  nav_path.header.stamp = ros::Time::now();
  nav_path.header.frame_id = frame_id;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.z = 0.0;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = frame_id;
  tf::Quaternion quat;
  for (std::list<GeneratedPath>::const_iterator i = generated_paths_.begin();
       i != generated_paths_.end(); ++i) {
    const GlobalPoint2d::Vec& path = i->path;
    for (GlobalPoint2d::ConstIter j = path.begin(); j != path.end(); ++j) {
      pose.pose.position.x = j->x();
      pose.pose.position.y = j->y();
      quat.setRPY(0.0, 0.0, j->angle);
      pose.pose.orientation.x = quat.x();
      pose.pose.orientation.y = quat.y();
      pose.pose.orientation.z = quat.z();
      pose.pose.orientation.w = quat.w();
      nav_path.poses.emplace_back(pose);
    }
  }
  path_pub_.publish(nav_path);
}

void PathManager::SendPath(const std::string& frame_id) {
  static const FitParam* fit_param = &NavigationConfig::get_instance().fit_param;
  const rFloat max_v = NavigationConfig::get_instance().max_wheel_speed;
  const rFloat max_w = (max_v + max_v) / fit_param->wheel_spacing();

  task_msgs::SendTaskInfoRequest req;
  task_msgs::SendTaskInfoResponse res;
  req.path.frame_id = frame_id;
  req.path.path_type = EXE_PATH;
  req.path.can_smooth = true;
  req.path.is_end_pose_set = false;
  req.path.is_path_reversed = false;
  req.path.max_linear_speed = max_v;
  req.path.max_rotation_speed = max_w;
  req.path.max_angular_error = max_angle_error_;
  req.path.max_distance_error = max_distance_error_;
  int path_id = 1;
  task_msgs::Point2d task_point_2d;
  ros::Duration duration(0.1);
  for (std::list<GeneratedPath>::const_iterator i = generated_paths_.begin();
       i != generated_paths_.end(); ++i) {
    const std::vector<nav_tools::GlobalPoint2d>& path = i->path;
    req.path.path_id = path_id;
    for (GlobalPoint2d::ConstIter p = path.begin(); p != path.end(); ++p) {
      task_point_2d.x = p->x();
      task_point_2d.y = p->y();
      task_point_2d.angle = p->angle;
      task_point_2d.dir_x = p->dir.x();
      task_point_2d.dir_y = p->dir.y();
      req.path.poses.emplace_back(task_point_2d);
    }
    ++path_id;
    add_task_client_.call(req, res);
    duration.sleep();
  }
}

void PathManager::SavePath(const std::string& file_path) {
  std::fstream file = std::fstream(file_path, std::ios::out);
  std::string line;
  for (std::list<GeneratedPath>::const_iterator i = generated_paths_.begin();
       i != generated_paths_.end(); ++i) {
    line = kRectangleDelimeter + std::to_string(i->rectangle.vertexes[0].x()) + kDataDelimiter
        + std::to_string(i->rectangle.vertexes[0].y()) + kDataDelimiter
        + std::to_string(i->rectangle.vertexes[1].x()) + kDataDelimiter
        + std::to_string(i->rectangle.vertexes[1].y()) + kDataDelimiter
        + std::to_string(i->rectangle.vertexes[2].x()) + kDataDelimiter
        + std::to_string(i->rectangle.vertexes[2].y()) + kDataDelimiter
        + std::to_string(i->rectangle.vertexes[3].x()) + kDataDelimiter
        + std::to_string(i->rectangle.vertexes[3].y()) + kDataDelimiter
        + std::to_string(i->rectangle.v1.x()) + kDataDelimiter
        + std::to_string(i->rectangle.v1.y()) + kDataDelimiter
        + std::to_string(i->rectangle.v2.x()) + kDataDelimiter
        + std::to_string(i->rectangle.v2.y()) + kDataDelimiter;
    file << line << std::endl;
    const GlobalPoint2d::Vec& path = i->path;
    for (GlobalPoint2d::ConstIter j = path.begin(); j != path.end(); ++j) {
      line = std::to_string(j->coord.x()) + kDataDelimiter
          + std::to_string(j->coord.y()) + kDataDelimiter
          + std::to_string(j->angle) + kDataDelimiter
          + std::to_string(j->k) + kDataDelimiter
          + std::to_string(j->s) + kDataDelimiter
          + std::to_string(j->max_v) + kDataDelimiter
          + std::to_string(j->max_w) + kDataDelimiter
          + std::to_string(j->max_angle_error) + kDataDelimiter
          + std::to_string(j->max_distance_error()) + kDataDelimiter
          + std::to_string(j->is_regular);
      file << line << std::endl;
    }
  }
  file.close();
}

void PathManager::Load(const std::string& file_path) {
  std::fstream file = std::fstream(file_path, std::ios::in);
  char data[300];
  std::string data_str;
  generated_paths_.clear();
  Rectangle rectangle;
  GlobalPoint2d point;
  GeneratedPath* generated_path_ptr = nullptr;
  StringList line_str_list;
  while (!file.eof()) {
    file.getline(data, 300);
    data_str = data;
    if ("" == data_str)
      continue;

    if (data_str.find(kRectangleDelimeter) != std::string::npos) {
      GeneratedPath generated_path;
      line_str_list = split_string(data_str, kRectangleDelimeter);
      line_str_list = split_string(line_str_list[0], kDataDelimiter);
      assert(line_str_list.size() == 12);
      rectangle.vertexes[0][0] = std::atof(line_str_list[0].c_str());
      rectangle.vertexes[0][1] = std::atof(line_str_list[1].c_str());
      rectangle.vertexes[1][0] = std::atof(line_str_list[2].c_str());
      rectangle.vertexes[1][1] = std::atof(line_str_list[3].c_str());
      rectangle.vertexes[2][0] = std::atof(line_str_list[4].c_str());
      rectangle.vertexes[2][1] = std::atof(line_str_list[5].c_str());
      rectangle.vertexes[3][0] = std::atof(line_str_list[6].c_str());
      rectangle.vertexes[3][1] = std::atof(line_str_list[7].c_str());
      rectangle.v1[0] = std::atof(line_str_list[8].c_str());
      rectangle.v1[1] = std::atof(line_str_list[9].c_str());
      rectangle.v1[0] = std::atof(line_str_list[10].c_str());
      rectangle.v1[1] = std::atof(line_str_list[11].c_str());
      generated_path.rectangle = rectangle;
      generated_paths_.emplace_back(generated_path);
      generated_path_ptr = &generated_paths_.back();
    }
    else {
      line_str_list = split_string(data_str, kDataDelimiter);
      assert(line_str_list.size() == 10);
      point.coord = Vec2f(std::atof(line_str_list[0].c_str()), std::atof(line_str_list[1].c_str()));
      point.angle = std::atof(line_str_list[2].c_str());
      point.k = std::atof(line_str_list[3].c_str());
      point.s = std::atof(line_str_list[4].c_str());
      point.max_v = std::atof(line_str_list[5].c_str());
      point.max_w = std::atof(line_str_list[6].c_str());
      point.max_angle_error = std::atof(line_str_list[7].c_str());
      point.set_max_distance_error(std::atof(line_str_list[8].c_str()));
      point.is_regular = ("1" == line_str_list[9]);
      assert(nullptr != generated_path_ptr);
      generated_path_ptr->path.emplace_back(point);
    }
  }
}

//Generate cleaning path in the given rectangle.
void PathManager::GeneratePath(
    const Rectangle& rectangle,
    std::vector<boost::shared_ptr<nav_tools::AnalyticPath2d> >& paths) const {
  Vec2f path_dir = rectangle.v1;
  rFloat length = path_dir.norm();
  Vec2f path_norm = rectangle.v2;
  rFloat width = path_norm.norm();
  if (length <= 0.0 || width <= 0.0) {
    boost::shared_ptr<AnalyticPath2d> line = boost::make_shared<Line2d>
        (rectangle.vertexes[0], rectangle.vertexes[2]);
    paths.emplace_back(line);
    return;
  }

  path_norm /= width;
  rFloat longitude_spacing_length = path_longitude_spacing_;
  if (multiply_2(longitude_spacing_length) > length) {
    longitude_spacing_length = 0.5 * length;
  }
  const Vec2f longitude_spacing_vec = path_dir * (longitude_spacing_length / length);
  const Vec2f longitude_vec = path_dir - (longitude_spacing_vec + longitude_spacing_vec);

  const rFloat path_number = ceil(width / path_latitude_spacing_);
  const rFloat latitude_spacing_length = width / path_number;
  const rFloat two_latitude_spacing = latitude_spacing_length + latitude_spacing_length;
  const rFloat half_latitude_spacing = latitude_spacing_length * 0.5;
  rFloat cur_latitude = 0.0;
  const rFloat max_latitude_length = width + half_latitude_spacing;
  Vec2f cur_start = rectangle.vertexes[0] + longitude_spacing_vec;
  Vec2f cur_longitude_vec = longitude_vec;
  Vec2f latitude_spacing_vec = path_norm * two_latitude_spacing;
  boost::shared_ptr<AnalyticPath2d> line1 = boost::make_shared<Line2d>
      (rectangle.vertexes[0], cur_start + cur_longitude_vec);
  cur_start += cur_longitude_vec;
  paths.emplace_back(line1);
  cur_latitude += two_latitude_spacing;
  boost::shared_ptr<AnalyticPath2d> line2;
  while (cur_latitude <= max_latitude_length) {
    cur_start += latitude_spacing_vec;
    cur_longitude_vec = -cur_longitude_vec;
    line2 = boost::make_shared<Line2d>(cur_start, cur_start + cur_longitude_vec);
    cur_start += cur_longitude_vec;
    boost::shared_ptr<CompositeClothoid2d> composite_clothoid =
        boost::make_shared<CompositeClothoid2d>();
    composite_clothoid->FitTarget(line2->start(), line1->end(), *fit_param_);
    paths.emplace_back(composite_clothoid);
    paths.emplace_back(line2);
    line1 = line2;
    cur_latitude += two_latitude_spacing;
  }

  //The last line path coincides with the edge of the rectangle.
  if (cur_latitude > width + half_latitude_spacing + latitude_spacing_length) {
    cur_start -= path_norm * latitude_spacing_length;
    cur_latitude = latitude_spacing_length + two_latitude_spacing;
  }
  else {
    cur_start += path_norm * latitude_spacing_length;
    cur_latitude = two_latitude_spacing;
  }
  cur_longitude_vec = -cur_longitude_vec;
  line2 = boost::make_shared<Line2d>(cur_start, cur_start + cur_longitude_vec);
  cur_start += cur_longitude_vec;
  boost::shared_ptr<CompositeClothoid2d> composite_clothoid =
      boost::make_shared<CompositeClothoid2d>();
  composite_clothoid->FitTarget(line2->start(), line1->end(), *fit_param_);
  paths.emplace_back(composite_clothoid);
  paths.emplace_back(line2);
  line1 = line2;
  latitude_spacing_vec = -latitude_spacing_vec;
  while (cur_latitude <= max_latitude_length) {
    cur_start += latitude_spacing_vec;
    cur_longitude_vec = -cur_longitude_vec;
    line2 = boost::make_shared<Line2d>(cur_start, cur_start + cur_longitude_vec);
    cur_start += cur_longitude_vec;
    boost::shared_ptr<CompositeClothoid2d> composite_clothoid =
        boost::make_shared<CompositeClothoid2d>();
    composite_clothoid->FitTarget(line2->start(), line1->end(), *fit_param_);
    paths.emplace_back(composite_clothoid);
    paths.emplace_back(line2);
    line1 = line2;
    cur_latitude += two_latitude_spacing;
  }
  line2 = boost::make_shared<Line2d>(
        cur_start, cur_start + cur_longitude_vec * (longitude_spacing_length
                               / (length - longitude_spacing_length - longitude_spacing_length)));
  paths.emplace_back(line2);
}

PathManager::PathManager()
  : max_distance_error_(0.05)
  , square_max_distance_error_(0.05 * 0.05)
  , max_angle_error_(0.05)
  , path_latitude_spacing_(0.6)
  , path_longitude_spacing_(0.5)
  , default_file_dir_("time_log")
  , kDataDelimiter(" ")
  , kRectangleDelimeter("rectangle:") {
  ros::NodeHandle private_nh("~");
  private_nh.param("max_distance_error", max_distance_error_, max_distance_error_);
  square_max_distance_error_ = max_distance_error_ * max_distance_error_;
  private_nh.param("path_latitude_spacing", path_latitude_spacing_, path_latitude_spacing_);
  private_nh.param("path_longitude_spacing", path_longitude_spacing_, path_longitude_spacing_);
  fit_param_ = &NavigationConfig::get_instance().fit_param;
  private_nh.param("max_angle_error", max_angle_error_, max_angle_error_);
  private_nh.param("default_file_dir", default_file_dir_, default_file_dir_);
  default_file_dir_ = getenv("HOME") + std::string("/") + default_file_dir_;

  path_pub_ = private_nh.advertise<nav_msgs::Path>("cleaning_path", 1, true);
  ros::NodeHandle nh;
  add_task_client_ = nh.serviceClient<task_msgs::SendTaskInfo>("task_control/add_task");
}

}   //namespace custom_tools
}   //namespace rock
