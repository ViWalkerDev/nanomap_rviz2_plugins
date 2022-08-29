/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NANOMAP_RVIZ2_PLUGINS__DISPLAYS__OPENVDB_GRID__OPENVDB_GRID_STRUCTS_HPP_
#define NANOMAP_RVIZ2_PLUGINS__DISPLAYS__OPENVDB_GRID__OPENVDB_GRID_STRUCTS_HPP_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <deque>
# include <list>
# include <map>
# include <memory>
# include <queue>
# include <vector>
# include <string>

# include <QObject>  // NOLINT
# include <QList>  // NOLINT


#include "pluginlib/class_loader.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

#include "nanomap_msgs/msg/openvdb_grid.hpp"

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "nanomap_rviz2_plugins/rendering/objects/openvdb_grid.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

#endif
#define GRIDDEPTH 4

namespace rviz_common
{

class Display;
class DisplayContext;

namespace properties
{

class BoolProperty;
class EnumProperty;
class FloatProperty;

}  // namespace properties

}  // namespace rviz_common

namespace rviz_default_plugins
{


struct RVIZ_DEFAULT_PLUGINS_PUBLIC NodeInfo
{
  NodeInfo();
  ~NodeInfo();


  // clear the point grid, but keep selection handler around
  void clear();
  Ogre::SceneManager * manager_;

  //nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr message_;

  Ogre::SceneNode * scene_node_;
  std::shared_ptr<rviz_rendering::OpenvdbGrid> grid_;
  //OpenvdbGridSelectionHandlerPtr selection_handler_;
  std::vector<rviz_rendering::OpenvdbGrid::Point> transformed_points_;

};

struct RVIZ_DEFAULT_PLUGINS_PUBLIC GridInfo
{
  GridInfo();
  ~GridInfo();
  void clear();
  std::vector<std::shared_ptr<NodeInfo>> node_infos_ = std::vector<std::shared_ptr<NodeInfo>>(GRIDDEPTH);
  nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr message_;

  rclcpp::Time receive_time_;
  Ogre::Quaternion orientation_ = Ogre::Quaternion(1.0,0.0,0.0,0.0);
  Ogre::Vector3 position_;

  std::vector<uint32_t> grid_exponents_ = {5,4,3,0};
  float voxel_size_ = 0.1f;
  std::vector<uint32_t> node_counts_ = std::vector<uint32_t>(GRIDDEPTH);
  std::vector<float> sizes_ = std::vector<float>(GRIDDEPTH);

};


}  // namespace rviz_default_plugins

#endif  // NANOMAP_RVIZ2_PLUGINS__DISPLAYS__OPENVDB_GRID__OPENVDB_GRID_COMMON_HPP_
