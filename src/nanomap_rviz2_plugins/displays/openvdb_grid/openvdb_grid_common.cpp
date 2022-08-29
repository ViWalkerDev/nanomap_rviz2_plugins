/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_common.hpp"

#include <memory>
#include <set>
#include <string>
#include <vector>
#include <utility>

#include <OgreSceneNode.h>
#include <OgreWireBoundingBox.h>

#include "rclcpp/clock.hpp"

#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_common/validate_floats.hpp"

namespace rviz_default_plugins
{


const std::string OpenvdbGridCommon::message_status_name_ = "Message";  // NOLINT allow std::string

OpenvdbGridCommon::OpenvdbGridCommon(rviz_common::Display * display)
: auto_size_(false),
  new_xyz_transformer_(false),
  new_color_transformer_(false),
  needs_retransform_(false),
  transformer_factory_(std::make_unique<OpenvdbGridTransformerFactory>()),
  display_(display)
{

  int1_mode_property_ = new rviz_common::properties::EnumProperty(
    "Level 1 Render Mode", "Do not show",
    "Rendering mode to use for Level 1 Nodes, in order of computational complexity.",
    display_, SLOT(updateStyle()), this);
  int1_mode_property_->addOption("Do not show", rviz_rendering::OpenvdbGrid::DO_NOT_SHOW);
  int1_mode_property_->addOption("Show active level 1 nodes", rviz_rendering::OpenvdbGrid::SHOW_ACTIVE);
  int1_mode_property_->addOption("Show inactive level 1 nodes", rviz_rendering::OpenvdbGrid::SHOW_INACTIVE);

  int1_alpha_property_ = new rviz_common::properties::FloatProperty(
      "Level 1 Alpha", 1.0f,
      "Amount of transparency to apply to Level 1 Nodes.  Note that this is experimental "
      "and does not always look correct.",
      display_, SLOT(updateAlpha()), this);
  int1_alpha_property_->setMin(0);
  int1_alpha_property_->setMax(1);

  int2_mode_property_ = new rviz_common::properties::EnumProperty(
    "Level 2 Render Mode", "Do not show",
    "Rendering mode to use for Level 2 Nodes.",
    display_, SLOT(updateStyle()), this);
  int2_mode_property_->addOption("Do not show", rviz_rendering::OpenvdbGrid::DO_NOT_SHOW);
  int2_mode_property_->addOption("Show active level 2 nodes", rviz_rendering::OpenvdbGrid::SHOW_ACTIVE);
  int2_mode_property_->addOption("Show inactive level 2 nodes", rviz_rendering::OpenvdbGrid::SHOW_INACTIVE);

  int2_alpha_property_ = new rviz_common::properties::FloatProperty(
      "Level 2 Alpha", 1.0f,
      "Amount of transparency to apply to Level 2 Nodes.  Note that this is experimental "
      "and does not always look correct.",
      display_, SLOT(updateAlpha()), this);
  int2_alpha_property_->setMin(0);
  int2_alpha_property_->setMax(1);

  leaf_mode_property_ = new rviz_common::properties::EnumProperty(
    "Leaf Node Render Mode", "Do not show",
    "Rendering mode to use for Leaf Nodes.",
    display_, SLOT(updateStyle()), this);
  leaf_mode_property_->addOption("Do not show", rviz_rendering::OpenvdbGrid::DO_NOT_SHOW);
  leaf_mode_property_->addOption("Show active leaf nodes", rviz_rendering::OpenvdbGrid::SHOW_ACTIVE);
  leaf_mode_property_->addOption("Show inactive leaf nodes", rviz_rendering::OpenvdbGrid::SHOW_INACTIVE);

  leaf_alpha_property_ = new rviz_common::properties::FloatProperty(
      "Leaf Node Alpha", 1.0f,
      "Amount of transparency to apply to Leaf Nodes.  Note that this is experimental "
      "and does not always look correct.",
      display_, SLOT(updateAlpha()), this);
  leaf_alpha_property_->setMin(0);
  leaf_alpha_property_->setMax(1);

  voxel_mode_property_ = new rviz_common::properties::EnumProperty(
    "Voxel Render Mode", "Do not show",
    "Rendering mode to use for voxels.",
    display_, SLOT(updateStyle()), this);
  voxel_mode_property_->addOption("Do not show", rviz_rendering::OpenvdbGrid::DO_NOT_SHOW);
  voxel_mode_property_->addOption("Show active voxels", rviz_rendering::OpenvdbGrid::SHOW_ACTIVE);
  voxel_mode_property_->addOption("Show inactive voxels", rviz_rendering::OpenvdbGrid::SHOW_INACTIVE);

  voxel_alpha_property_ = new rviz_common::properties::FloatProperty(
      "Voxel Node Alpha", 1.0f,
      "Amount of transparency to apply to Voxels.  Note that this is experimental "
      "and does not always look correct.",
      display_, SLOT(updateAlpha()), this);
  voxel_alpha_property_->setMin(0);
  voxel_alpha_property_->setMax(1);

  voxel_style_property_ = new rviz_common::properties::EnumProperty(
    "Voxel Render Style", "Spheres",
    "Rendering mode to use, in order of computational complexity.",
    display_, SLOT(updateStyle()), this);
  voxel_style_property_->addOption("Points", rviz_rendering::OpenvdbGrid::RM_POINTS);
  voxel_style_property_->addOption("Squares", rviz_rendering::OpenvdbGrid::RM_SQUARES);
  voxel_style_property_->addOption("Flat Squares", rviz_rendering::OpenvdbGrid::RM_FLAT_SQUARES);
  voxel_style_property_->addOption("Points", rviz_rendering::OpenvdbGrid::RM_POINTS);
  voxel_style_property_->addOption("Tiles", rviz_rendering::OpenvdbGrid::RM_TILES);
  voxel_style_property_->addOption("Spheres",rviz_rendering::OpenvdbGrid::RM_SPHERES);
  voxel_style_property_->addOption("Boxes", rviz_rendering::OpenvdbGrid::RM_BOXES);


  voxel_world_size_property_ = new rviz_common::properties::FloatProperty(
    "Voxel Size (m)", 0.1f,
    "Voxel size in meters.",
    display_, SLOT(updateBillboardSize()), this);
  voxel_world_size_property_->setMin(0.001f);

  voxel_pixel_size_property_ = new rviz_common::properties::FloatProperty(
    "Voxel Size (Pixels)", 3,
    "Voxel size in pixels.",
    display_, SLOT(updateBillboardSize()), this);
  voxel_pixel_size_property_->setMin(1);

  decay_time_property_ = new rviz_common::properties::FloatProperty(
    "Decay Time", 0,
    "Duration, in seconds, to keep the incoming points.  0 means only show the latest points.",
    display_, SLOT(queueRender()));
  decay_time_property_->setMin(0);

  xyz_transformer_property_ = new rviz_common::properties::EnumProperty(
    "Position Transformer", "",
    "Set the transformer to use to set the position of the points.",
    display_, SLOT(updateXyzTransformer()), this);
  connect(
    xyz_transformer_property_, SIGNAL(
      requestOptions(
        rviz_common::properties::EnumProperty*)),
    this, SLOT(setXyzTransformerOptions(rviz_common::properties::EnumProperty*)));


  color_transformer_properties_[0] = new rviz_common::properties::EnumProperty(
    "Level 1 Color Transformer", "",
    "Set the transformer to use to set the color of the level 1 nodes.",
    display_, SLOT(updateInt1ColorTransformer()), this);
  connect(
    color_transformer_properties_[0],
    SIGNAL(requestOptions(rviz_common::properties::EnumProperty*)),
    this, SLOT(setColorTransformerOptions(rviz_common::properties::EnumProperty*)));

  color_transformer_properties_[1] = new rviz_common::properties::EnumProperty(
    "Level 2 Color Transformer", "",
    "Set the transformer to use to set the color of the Level 2  nodes.",
    display_, SLOT(updateInt2ColorTransformer()), this);
  connect(
    color_transformer_properties_[1],
    SIGNAL(requestOptions(rviz_common::properties::EnumProperty*)),
    this, SLOT(setColorTransformerOptions(rviz_common::properties::EnumProperty*)));

  color_transformer_properties_[2] = new rviz_common::properties::EnumProperty(
    "Leaf Node Color Transformer", "",
    "Set the transformer to use to set the color of the leaf nodes.",
    display_, SLOT(updateLeafColorTransformer()), this);
  connect(
    color_transformer_properties_[2],
    SIGNAL(requestOptions(rviz_common::properties::EnumProperty*)),
    this, SLOT(setColorTransformerOptions(rviz_common::properties::EnumProperty*)));

  color_transformer_properties_[3] = new rviz_common::properties::EnumProperty(
    "Voxel Color Transformer", "",
    "Set the transformer to use to set the color of the voxels.",
    display_, SLOT(updateVoxelColorTransformer()), this);
  connect(
    color_transformer_properties_[3],
    SIGNAL(requestOptions(rviz_common::properties::EnumProperty*)),
    this, SLOT(setColorTransformerOptions(rviz_common::properties::EnumProperty*)));
}

void OpenvdbGridCommon::initialize(
  rviz_common::DisplayContext * context,
  Ogre::SceneNode * scene_node)
{
  loadTransformers();

  context_ = context;
  scene_node_ = scene_node;
  clock_ = context->getClock();
  updateStyleVector();
  updateStyle();
  updateBillboardSize();
  updateAlphaVector();
  updateAlpha();
}

void OpenvdbGridCommon::loadTransformers()
{
  auto plugins = transformer_factory_->getDeclaredPlugins();
  for (auto const & plugin : plugins) {
    auto plugin_name_std = plugin.name.toStdString();
    if (transformers_.count(plugin_name_std) > 0) {
      RVIZ_COMMON_LOG_ERROR_STREAM("Transformer type " << plugin_name_std << " is already loaded.");
      continue;
    }

    OpenvdbGridTransformerPtr trans(transformer_factory_->make(plugin.id));
    loadTransformer(trans, plugin_name_std, plugin.id.toStdString());
  }
}

void OpenvdbGridCommon::loadTransformer(
  OpenvdbGridTransformerPtr trans,
  std::string name,
  const std::string & lookup_name)
{
  trans->init();
  connect(trans.get(), SIGNAL(needRetransform()), this, SLOT(causeRetransform()));

  TransformerInfo info;
  info.transformer = trans;
  info.readable_name = name;
  info.lookup_name = lookup_name;

  info.transformer->createProperties(
    display_, OpenvdbGridTransformer::Support_XYZ, info.xyz_props);
  setPropertiesHidden(info.xyz_props, true);

  info.transformer->createProperties(
    display_, OpenvdbGridTransformer::Support_Color, info.color_props);
  setPropertiesHidden(info.color_props, true);

  transformers_[name] = info;
}

void OpenvdbGridCommon::updateModeVector(){
  modes_[0] = static_cast<rviz_rendering::OpenvdbGrid::RenderMode>(int1_mode_property_->getOptionInt());
  modes_[1] = static_cast<rviz_rendering::OpenvdbGrid::RenderMode>(int2_mode_property_->getOptionInt());
  modes_[2] = static_cast<rviz_rendering::OpenvdbGrid::RenderMode>(leaf_mode_property_->getOptionInt());
  modes_[3] = static_cast<rviz_rendering::OpenvdbGrid::RenderMode>(voxel_mode_property_->getOptionInt());
}

void OpenvdbGridCommon::updateAlphaVector(){
  alphas_[0] = int1_alpha_property_->getFloat();
  alphas_[1] = int2_alpha_property_->getFloat();
  alphas_[2] = leaf_alpha_property_->getFloat();
  alphas_[3] = voxel_alpha_property_->getFloat();
}

void OpenvdbGridCommon::updateAlpha()
{
  updateAlphaVector();
  for(auto const & grid_info : grid_infos_){
    int d = 0;
    for (auto const & node_info : grid_info->node_infos_) {
      bool per_point_alpha = false;
      node_info->grid_->setAlpha(alphas_[d], per_point_alpha);
      d++;
    }
  }
}

void OpenvdbGridCommon::updateStyleVector(){
  styles_[0] = rviz_rendering::OpenvdbGrid::RM_BOXES;
  styles_[1] = rviz_rendering::OpenvdbGrid::RM_BOXES;
  styles_[2] = rviz_rendering::OpenvdbGrid::RM_BOXES;
  styles_[3] = static_cast<rviz_rendering::OpenvdbGrid::RenderMode>(voxel_style_property_->getOptionInt());
}

void OpenvdbGridCommon::updateStyle()
{
  updateStyleVector();
  if (styles_[3] == rviz_rendering::OpenvdbGrid::RM_POINTS) {
    voxel_world_size_property_->hide();
    voxel_pixel_size_property_->show();
  } else {
    voxel_world_size_property_->show();
    voxel_pixel_size_property_->hide();
  }
  for (auto const & grid_info : grid_infos_) {
    int d = 0;
    for(auto const & node_info : grid_info->node_infos_) {
        node_info->grid_->setRenderMode(styles_[d]);
        d++;
    }
  }
  causeRetransform();
  updateBillboardSize();
}

void OpenvdbGridCommon::updateBillboardSize()
{
  float size;
  updateStyleVector();
  if (styles_[3] == rviz_rendering::OpenvdbGrid::RM_POINTS) {
    size = voxel_pixel_size_property_->getFloat();
  } else {
    size = voxel_world_size_property_->getFloat();
  }
  for (auto & grid_info : grid_infos_) {
    int d = 0;
    grid_info->sizes_[3] = size;
    for(auto & node_info : grid_info->node_infos_){
      node_info->grid_->setDimensions(grid_info->sizes_[d], grid_info->sizes_[d], grid_info->sizes_[d]);
      d++;
    }
  }
  context_->queueRender();
}

void OpenvdbGridCommon::reset()
{
  std::unique_lock<std::mutex> lock(new_grids_mutex_);
  grid_infos_.clear();
  new_grid_infos_.clear();
}

void OpenvdbGridCommon::causeRetransform()
{
  needs_retransform_ = true;
}

void OpenvdbGridCommon::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;
  float point_decay_time = decay_time_property_->getFloat();
  rclcpp::Time now = clock_->now();
  if (needs_retransform_) {
    retransform();
    needs_retransform_ = false;
  }
  collectObsoleteGridInfos(point_decay_time, now);
  removeObsoleteGridInfos();
  insertNewGrids(point_decay_time, now);
  updateTransformerProperties();
  updateStatus();
}

void OpenvdbGridCommon::insertNewGrids(float point_decay_time, const rclcpp::Time & now)
{
  std::unique_lock<std::mutex> lock(new_grids_mutex_);
  if (!new_grid_infos_.empty()) {
    auto it = new_grid_infos_.begin();
    auto end = new_grid_infos_.end();
    for (; it != end; ++it) {
      GridInfoPtr grid_info = *it;
      updateStyleVector();
      float size;
      if (styles_[3] == rviz_rendering::OpenvdbGrid::RM_POINTS) {
        size = voxel_pixel_size_property_->getFloat();
      } else {
        size = voxel_world_size_property_->getFloat();
      }
      grid_info->sizes_[3] = size;
      auto next = it; next++;
      if (next != end && gridInfoIsDecayed(grid_info, point_decay_time, now)) {
        continue;
      }
      int d = 0;
      for(auto & node_info : grid_info->node_infos_){
        bool per_point_alpha = false;
        node_info->grid_.reset(new rviz_rendering::OpenvdbGrid());
        node_info->grid_->setRenderMode(styles_[d]);
        node_info->grid_->addPoints(
        node_info->transformed_points_.begin(), node_info->transformed_points_.end());
        node_info->grid_->setAlpha(alphas_[d], per_point_alpha);
        node_info->grid_->setDimensions(grid_info->sizes_[d],
                                        grid_info->sizes_[d],
                                        grid_info->sizes_[d]);
        node_info->grid_->setAutoSize(auto_size_);
        node_info->manager_ = context_->getSceneManager();
        node_info->scene_node_ = scene_node_->createChildSceneNode(
                                                grid_info->position_,
                                                grid_info->orientation_);
        node_info->scene_node_->attachObject(node_info->grid_.get());
        d++;
      }
      grid_infos_.push_back(*it);
    }
    new_grid_infos_.clear();
  }
}

void OpenvdbGridCommon::updateTransformerProperties()
{
  std::unique_lock<std::mutex> lock(new_grids_mutex_);
  bool new_color_transformer = false;
  for(int x = 0; x < GRIDDEPTH; x++)
  {
    if(color_transformer_properties_[x]){
      new_color_transformer = true;
    }
  }
  if (new_xyz_transformer_ || new_color_transformer) {
    for (auto transformer : transformers_) {
      const std::string & name = transformer.first;
      TransformerInfo & info = transformer.second;

      setPropertiesHidden(info.xyz_props, name != xyz_transformer_property_->getStdString());
      bool name_check =false;
      for(int x = 0; x < GRIDDEPTH; x++){
        setPropertiesHidden(
          info.color_props, name != color_transformer_properties_[x]->getStdString());
          if(name == color_transformer_properties_[x]->getStdString()){
            name_check = true;
          }
      }


      if (name == xyz_transformer_property_->getStdString() || name_check)
      {
        info.transformer->hideUnusedProperties();
      }
    }
  }

  new_xyz_transformer_ = false;
  new_color_transformer_ = false;
}

void OpenvdbGridCommon::collectObsoleteGridInfos(float point_decay_time, const rclcpp::Time & now)
{
  std::unique_lock<std::mutex> lock(new_grids_mutex_);

  if (point_decay_time > 0.0 || !new_grid_infos_.empty()) {
    while (!grid_infos_.empty() &&
      gridInfoIsDecayed(grid_infos_.front(), point_decay_time, now))
    {
      grid_infos_.front()->clear();
      obsolete_grid_infos_.push_back(grid_infos_.front());
      grid_infos_.pop_front();
      context_->queueRender();
    }
  }
}

void OpenvdbGridCommon::removeObsoleteGridInfos()
{
  auto it = obsolete_grid_infos_.begin();
  auto end = obsolete_grid_infos_.end();
  while (it != end) {
      it = obsolete_grid_infos_.erase(it);
    if (it != end) {
      ++it;
    }
  }
}

bool OpenvdbGridCommon::gridInfoIsDecayed(
  GridInfoPtr grid_info, float point_decay_time, const rclcpp::Time & now)
{
  return (now.nanoseconds() - grid_info->receive_time_.nanoseconds()) / 1000000000.0 >
         point_decay_time;
}

void OpenvdbGridCommon::setPropertiesHidden(
  const QList<rviz_common::properties::Property *> & props,
  bool hide)
{
  for (auto prop : props) {
    prop->setHidden(hide);
  }
}

void OpenvdbGridCommon::updateTransformers(
  const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr & grid)
{
  std::string xyz_name = xyz_transformer_property_->getStdString();
  std::vector<std::string> color_names;
  for(int x = 0; x<GRIDDEPTH ; x++){
    color_names.push_back(color_transformer_properties_[x]->getStdString());
    color_transformer_properties_[x]->clearOptions();
  }
  xyz_transformer_property_->clearOptions();


  // Get the channels that we could potentially render
  typedef std::set<std::pair<uint8_t, std::string>> S_string;
  S_string valid_xyz, valid_color;
  bool cur_xyz_valid = false;
  std::vector<bool> cur_colors_valid = std::vector<bool>(GRIDDEPTH,false);
  bool has_rgb_transformer = false;
  for (auto transformer : transformers_) {
    const std::string & name = transformer.first;
    const OpenvdbGridTransformerPtr & trans = transformer.second.transformer;
    uint32_t mask = trans->supports(grid);
    if (mask & OpenvdbGridTransformer::Support_XYZ) {
      valid_xyz.insert(std::make_pair(trans->score(grid), name));
      if (name == xyz_name) {
        cur_xyz_valid = true;
      }
      xyz_transformer_property_->addOptionStd(name);
    }

    if (mask & OpenvdbGridTransformer::Support_Color) {
      valid_color.insert(std::make_pair(trans->score(grid), name));
      for(int i = 0; i < GRIDDEPTH; i++){
        if (name == color_names[i]) {
          cur_colors_valid[i] = true;
        }
        color_transformer_properties_[i]->addOptionStd(name);
        if (!cur_colors_valid[i]) {
          if (!valid_color.empty()) {
              color_transformer_properties_[i]->setStringStd(valid_color.rbegin()->second);
          }
        }
      }
    }
  }

  if (!cur_xyz_valid) {
    if (!valid_xyz.empty()) {
      xyz_transformer_property_->setStringStd(valid_xyz.rbegin()->second);
    }
  }


}

void OpenvdbGridCommon::updateStatus()
{
  std::stringstream ss;
  uint64_t total_point_count = 0;
  for(const auto & grid_info : grid_infos_){
    for (const auto & node_info : grid_info->node_infos_) {
      total_point_count += node_info->transformed_points_.size();
    }
  }
  ss << "Showing [" << total_point_count << "] points from [" << grid_infos_.size() <<
    "] messages";
  display_->setStatusStd(rviz_common::properties::StatusProperty::Ok, "Points", ss.str());

}

void OpenvdbGridCommon::processMessage(const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr grid)
{
  GridInfoPtr info(new GridInfo);
  info->message_ = grid;
  info->receive_time_ = clock_->now();
    if (transformGrid(info, true)) {
    std::unique_lock<std::mutex> lock(new_grids_mutex_);
    new_grid_infos_.push_back(info);
    display_->emitTimeSignal(grid->header.stamp);
  }
}

void OpenvdbGridCommon::updateXyzTransformer()
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  if (transformers_.count(xyz_transformer_property_->getStdString()) == 0) {
    return;
  }
  new_xyz_transformer_ = true;
  causeRetransform();
}

void OpenvdbGridCommon::updateInt1ColorTransformer()
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  if (transformers_.count(color_transformer_properties_[0]->getStdString()) == 0) {
    return;
  }
  new_color_transformer_ = true;
  causeRetransform();
}

void OpenvdbGridCommon::updateInt2ColorTransformer()
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  if (transformers_.count(color_transformer_properties_[1]->getStdString()) == 0) {
    return;
  }
  new_color_transformer_ = true;
  causeRetransform();
}
void OpenvdbGridCommon::updateLeafColorTransformer()
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  if (transformers_.count(color_transformer_properties_[2]->getStdString()) == 0) {
    return;
  }
  new_color_transformer_ = true;
  causeRetransform();
}

void OpenvdbGridCommon::updateVoxelColorTransformer()
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  if (transformers_.count(color_transformer_properties_[3]->getStdString()) == 0) {
    return;
  }
  new_color_transformer_ = true;
  causeRetransform();
}

OpenvdbGridTransformerPtr OpenvdbGridCommon::getXYZTransformer(
  const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr & grid)
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  auto it = transformers_.find(xyz_transformer_property_->getStdString());
  if (it != transformers_.end()) {
    const OpenvdbGridTransformerPtr & trans = it->second.transformer;
    if (trans->supports(grid) & OpenvdbGridTransformer::Support_XYZ) {
      return trans;
    }
  }

  return OpenvdbGridTransformerPtr();
}

OpenvdbGridTransformerPtr OpenvdbGridCommon::getColorTransformer(
  const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr & grid)
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  auto it = transformers_.find(color_transformer_property_->getStdString());
  if (it != transformers_.end()) {
    const OpenvdbGridTransformerPtr & trans = it->second.transformer;
    if (trans->supports(grid) & OpenvdbGridTransformer::Support_Color) {
      return trans;
    }
  }

  return OpenvdbGridTransformerPtr();
}
std::vector<OpenvdbGridTransformerPtr> OpenvdbGridCommon::getColorTransformers(
  const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr & grid)
{
  std::vector<OpenvdbGridTransformerPtr> color_transformers;
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  for(int x = 0; x < GRIDDEPTH ; x ++){
    auto it = transformers_.find(color_transformer_properties_[x]->getStdString());
    if (it != transformers_.end()) {
      const OpenvdbGridTransformerPtr & trans = it->second.transformer;
      if (trans->supports(grid) & OpenvdbGridTransformer::Support_Color) {
        color_transformers.push_back(trans);
      }
    }else{
      return std::vector<OpenvdbGridTransformerPtr>(GRIDDEPTH);
    }
  }
  return color_transformers;

}

void OpenvdbGridCommon::retransform()
{
  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);

  for (auto const & grid_info : grid_infos_) {
    transformGrid(grid_info, false);
    for(auto const & node_info : grid_info->node_infos_){
      node_info->grid_->clear();
      node_info->grid_->addPoints(
      node_info->transformed_points_.begin(), node_info->transformed_points_.end());
    }
  }
}

bool OpenvdbGridCommon::transformGrid(const GridInfoPtr & grid_info, bool update_transformers)
{
  for(auto const & node_info : grid_info->node_infos_){
    if (!node_info->scene_node_) {
      if (!context_->getFrameManager()->getTransform(
          grid_info->message_->header,
          grid_info->position_,
          grid_info->orientation_))
        {
        display_->setMissingTransformToFixedFrame(grid_info->message_->header.frame_id);
        return false;
      }
    }
  }
    display_->setTransformOk();
  // Remove outdated error message
    display_->deleteStatusStd(message_status_name_);
    updateModeVector();
    if (!transformPoints(grid_info, update_transformers)) {
      return false;
    }
    setProblematicPointsToInfinity(grid_info);
    return true;

}

bool OpenvdbGridCommon::transformPoints(
  const GridInfoPtr & grid_info, bool update_transformers)
{
  Ogre::Matrix4 transform;
  transform.makeTransform(grid_info->position_, Ogre::Vector3(1, 1, 1), grid_info->orientation_);

  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);
  if (update_transformers) {
    updateTransformers(grid_info->message_);
  }
  OpenvdbGridTransformerPtr xyz_trans = getXYZTransformer(grid_info->message_);
  std::vector<OpenvdbGridTransformerPtr> color_trans = getColorTransformers(grid_info->message_);

  if (!xyz_trans) {
    std::string status = "No position transformer available for grid";
    display_->setStatusStd(
      rviz_common::properties::StatusProperty::Error, message_status_name_, status);
    return false;
  }

  if (!color_trans[0]) {
    std::string status = "No color transformer available for level 1 nodes";
    display_->setStatusStd(
      rviz_common::properties::StatusProperty::Error, message_status_name_, status);
    return false;
  }
  if (!color_trans[1]) {
    std::string status = "No color transformer available for level 2 nodes";
    display_->setStatusStd(
      rviz_common::properties::StatusProperty::Error, message_status_name_, status);
    return false;
  }
  if (!color_trans[2]) {
    std::string status = "No color transformer available for leaf nodes";
    display_->setStatusStd(
      rviz_common::properties::StatusProperty::Error, message_status_name_, status);
    return false;
  }
  if (!color_trans[3]) {
    std::string status = "No color transformer available for voxels";
    display_->setStatusStd(
      rviz_common::properties::StatusProperty::Error, message_status_name_, status);
    return false;
  }
  xyz_trans->transform(0,
    grid_info, OpenvdbGridTransformer::Support_XYZ, modes_, transform);
  color_trans[0]->transform(0,
    grid_info, OpenvdbGridTransformer::Support_Color, modes_, transform);
  color_trans[1]->transform(1,
    grid_info, OpenvdbGridTransformer::Support_Color, modes_, transform);
  color_trans[2]->transform(2,
    grid_info, OpenvdbGridTransformer::Support_Color, modes_, transform);
  color_trans[3]->transform(3,
    grid_info, OpenvdbGridTransformer::Support_Color, modes_, transform);
  return true;
}

void OpenvdbGridCommon::setProblematicPointsToInfinity(const GridInfoPtr & grid_info)
{
  for(auto & node_info : grid_info->node_infos_){
  for (auto & grid_point : node_info->transformed_points_) {
      if (!rviz_common::validateFloats(grid_point.position)) {
        grid_point.position.x = 999999.0f;
        grid_point.position.y = 999999.0f;
        grid_point.position.z = 999999.0f;
      }
    }
  }
}

void OpenvdbGridCommon::addMessage(const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr grid)
{
  processMessage(grid);
}

void OpenvdbGridCommon::setXyzTransformerOptions(rviz_common::properties::EnumProperty * prop)
{
  fillTransformerOptions(prop, OpenvdbGridTransformer::Support_XYZ);
}

void OpenvdbGridCommon::setColorTransformerOptions(rviz_common::properties::EnumProperty * prop)
{
  fillTransformerOptions(prop, OpenvdbGridTransformer::Support_Color);
}

void OpenvdbGridCommon::fillTransformerOptions(
  rviz_common::properties::EnumProperty * prop,
  uint32_t mask)
{
  prop->clearOptions();

  if (grid_infos_.empty()) {
    return;
  }

  std::unique_lock<std::recursive_mutex> lock(transformers_mutex_);

  const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr & msg = grid_infos_.front()->message_;

  for (auto transformer : transformers_) {
    const OpenvdbGridTransformerPtr & trans = transformer.second.transformer;
    if ((trans->supports(msg) & mask) == mask) {
      prop->addOption(QString::fromStdString(transformer.first));
    }
  }
}

void OpenvdbGridCommon::onDisable()
{
}

float OpenvdbGridCommon::getSelectionBoxSize()
{
  return voxel_style_property_->getOptionInt() != rviz_rendering::OpenvdbGrid::RM_POINTS ?
         voxel_world_size_property_->getFloat() : 0.004f;
}

}  // namespace rviz_default_plugins
