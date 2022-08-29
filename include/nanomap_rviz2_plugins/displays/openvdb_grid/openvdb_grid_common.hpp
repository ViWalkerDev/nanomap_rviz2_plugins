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

#ifndef NANOMAP_RVIZ2_PLUGINS__DISPLAYS__OPENVDB_GRID__OPENVDB_GRID_COMMON_HPP_
#define NANOMAP_RVIZ2_PLUGINS__DISPLAYS__OPENVDB_GRID__OPENVDB_GRID_COMMON_HPP_

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

//# include <message_filters/time_sequencer.h>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

#include "nanomap_msgs/msg/openvdb_grid.hpp"
#include "nanomap_msgs/msg/openvdb_grid.hpp"

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "nanomap_rviz2_plugins/rendering/objects/openvdb_grid.hpp"

#include "openvdb_grid_transformer.hpp"
//#include "openvdb_grid_selection_handler.hpp"

//#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_selection_handler.hpp"
#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_structs.hpp"
#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_transformer.hpp"
#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_transformer_factory.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#endif


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

//typedef std::shared_ptr<OpenvdbGridSelectionHandler> OpenvdbGridSelectionHandlerPtr;
class OpenvdbGridTransformer;
class OpenvdbGridTransformerFactory;
typedef std::shared_ptr<OpenvdbGridTransformer> OpenvdbGridTransformerPtr;

typedef std::vector<std::string> V_string;
/**
 * \class OpenvdbGridCommon
 * \brief Displays a point grid of type sensor_msgs::OpenvdbGrid
 *
 * By default it will assume channel 0 of the grid is an intensity value, and will color them by intensity.
 * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r, g and b
 * all being 8 bits.
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC OpenvdbGridCommon : public QObject
{
  Q_OBJECT

public:
  typedef std::shared_ptr<GridInfo> GridInfoPtr;
  typedef std::deque<GridInfoPtr> D_GridInfo;
  typedef std::vector<GridInfoPtr> V_GridInfo;
  typedef std::list<GridInfoPtr> L_GridInfo;

  explicit OpenvdbGridCommon(rviz_common::Display * display);

  void initialize(rviz_common::DisplayContext * context, Ogre::SceneNode * scene_node);

  void reset();
  void update(float wall_dt, float ros_dt);

  void addMessage(nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr grid);
  //void addMessage(nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr grid);

  rviz_common::Display * getDisplay() {return display_;}

  void onDisable();

  bool auto_size_;

  //rviz_common::properties::BoolProperty * selectable_property_;
  rviz_common::properties::FloatProperty * voxel_world_size_property_;
  rviz_common::properties::FloatProperty * voxel_pixel_size_property_;
  rviz_common::properties::EnumProperty * int1_mode_property_;
  rviz_common::properties::FloatProperty * int1_alpha_property_;
  rviz_common::properties::EnumProperty * int2_mode_property_;
  rviz_common::properties::FloatProperty * int2_alpha_property_;
  rviz_common::properties::EnumProperty * leaf_mode_property_;
  rviz_common::properties::FloatProperty * leaf_alpha_property_;
  rviz_common::properties::EnumProperty * voxel_mode_property_;
  rviz_common::properties::FloatProperty * voxel_alpha_property_;
  rviz_common::properties::EnumProperty * voxel_style_property_;
  rviz_common::properties::FloatProperty * decay_time_property_;
  rviz_common::properties::EnumProperty * xyz_transformer_property_;
  rviz_common::properties::EnumProperty * color_transformer_property_;
    rviz_common::properties::EnumProperty * color_transformer_property2_;
  std::vector<rviz_common::properties::EnumProperty*> color_transformer_properties_ = std::vector<rviz_common::properties::EnumProperty*>(GRIDDEPTH);
  std::vector<rviz_rendering::OpenvdbGrid::RenderMode> styles_ = std::vector<rviz_rendering::OpenvdbGrid::RenderMode>(GRIDDEPTH);
  std::vector<rviz_rendering::OpenvdbGrid::RenderMode> modes_ = std::vector<rviz_rendering::OpenvdbGrid::RenderMode>(GRIDDEPTH);
  std::vector<float> alphas_ = std::vector<float>(GRIDDEPTH);
  std::vector<float> sizes_;
  //std::vector<uint32_t> grid_exponents_ = {5,4,3,0};
  //float voxel_size_ = 0.1f;
// TODO(anhosi): check if still needed when migrating DepthGrid
//  void setAutoSize(bool auto_size);

public Q_SLOTS:
  void causeRetransform();

private Q_SLOTS:
  //void updateSelectable();
  void updateStyle();
  void updateStyleVector();
  void updateBillboardSize();
  void updateAlphaVector();
  void updateAlpha();
  void updateModeVector();
  void updateXyzTransformer();

  void updateInt1ColorTransformer();
  void updateInt2ColorTransformer();
  void updateLeafColorTransformer();
  void updateVoxelColorTransformer();
  void setXyzTransformerOptions(rviz_common::properties::EnumProperty * prop);
  void setColorTransformerOptions(rviz_common::properties::EnumProperty * prop);

private:
  bool transformGrid(const GridInfoPtr & grid, bool fully_update_transformers);
  void processMessage(nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr grid);
  bool transformPoints(
    const GridInfoPtr & grid_info, bool update_transformers);
  void setProblematicPointsToInfinity(const GridInfoPtr & grid_info);
  void updateStatus();

  OpenvdbGridTransformerPtr getXYZTransformer(
    const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr & grid);
  OpenvdbGridTransformerPtr getColorTransformer(
    const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr & grid);
  std::vector<OpenvdbGridTransformerPtr> getColorTransformers(
    const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr & grid);
  void updateTransformers(const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr & grid);
  void retransform();

  void loadTransformers();
  void loadTransformer(
    OpenvdbGridTransformerPtr trans,
    std::string name,
    const std::string & lookup_name);

  float getSelectionBoxSize();
  void setPropertiesHidden(const QList<rviz_common::properties::Property *> & props, bool hide);
  void fillTransformerOptions(rviz_common::properties::EnumProperty * prop, uint32_t mask);

  void insertNewGrids(float point_decay_time, const rclcpp::Time & now);
  float getSizeForRenderMode(const rviz_rendering::OpenvdbGrid::RenderMode & mode);

  void updateTransformerProperties();

  /**
   * Instead of deleting obsolete grid infos, we just clear them
   * and put them into obsolete_node_infos, so active selections are preserved
   *
   * If decay time == 0, clear the old grid when we get a new one.
   * Otherwise, clear all the outdated ones
   */
  void collectObsoleteGridInfos(float point_decay_time, const rclcpp::Time & now);

  /// Garbage-collect old point grids that don't have an active selection
  void removeObsoleteGridInfos();

  bool gridInfoIsDecayed(
    const GridInfoPtr grid_info, float point_decay_time, const rclcpp::Time & now);

  D_GridInfo grid_infos_;

  Ogre::SceneNode * scene_node_;

  V_GridInfo new_grid_infos_;
  std::mutex new_grids_mutex_;

  L_GridInfo obsolete_grid_infos_;

  struct TransformerInfo
  {
    OpenvdbGridTransformerPtr transformer;
    QList<rviz_common::properties::Property *> xyz_props;
    QList<rviz_common::properties::Property *> color_props;

    std::string readable_name;
    std::string lookup_name;
  };
  typedef std::map<std::string, TransformerInfo> M_TransformerInfo;

  std::recursive_mutex transformers_mutex_;
  M_TransformerInfo transformers_;
  bool new_xyz_transformer_;
  std::vector<bool> new_color_transformers_;
  bool new_color_transformer_;
  //bool new_color_transformer2_;
  bool needs_retransform_;

  std::unique_ptr<OpenvdbGridTransformerFactory> transformer_factory_;

  rviz_common::Display * display_;
  rviz_common::DisplayContext * context_;
  rclcpp::Clock::SharedPtr clock_;

  static const std::string message_status_name_;

  friend class OpenvdbGridSelectionHandler;
};

}  // namespace rviz_default_plugins

#endif  // NANOMAP_RVIZ2_PLUGINS__DISPLAYS__OPENVDB_GRID__OPENVDB_GRID_COMMON_HPP_
