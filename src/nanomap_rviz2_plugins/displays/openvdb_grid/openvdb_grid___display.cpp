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

#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_display.hpp"

#include <memory>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_common.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "nanomap_rviz2_plugins/rendering/objects/openvdb_grid.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#ifdef foreach
  #undef foreach
#endif
#include "openvdb/openvdb.h"
#include "openvdb/io/Stream.h"

namespace rviz_default_plugins
{
namespace displays
{

OpenvdbGridDisplay::OpenvdbGridDisplay()
: openvdb_grid_common_(new OpenvdbGridCommon(this))
{}

void OpenvdbGridDisplay::onInitialize()
{
  MFDClass::onInitialize();
  openvdb_grid_common_->initialize(context_, scene_node_);
  openvdb::initialize();
}

void OpenvdbGridDisplay::processMessage(const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr grid)
{

  openvdb_grid_common_->addMessage(grid);
}

bool OpenvdbGridDisplay::gridDataMatchesDimensions(
  const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr grid) const
{
  return grid->width * grid->height * grid->point_step == grid->data.size();
}

nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr OpenvdbGridDisplay::filterOutInvalidPoints(
  const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr grid) const
{
  auto filtered = std::make_shared<nanomap_msgs::msg::OpenvdbGrid>();

  if (grid->width * grid->height > 0) {
    filtered->data = filterData(grid);
  }

  filtered->header = grid->header;
  filtered->fields = grid->fields;
  filtered->height = 1;
  filtered->width = static_cast<uint32_t>(filtered->data.size() / grid->point_step);
  filtered->is_bigendian = grid->is_bigendian;
  filtered->point_step = grid->point_step;
  filtered->row_step = filtered->width;

  return filtered;
}

nanomap_msgs::msg::OpenvdbGrid::_data_type
OpenvdbGridDisplay::filterData(nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr grid) const
{
  nanomap_msgs::msg::OpenvdbGrid::_data_type filteredData;
  filteredData.reserve(grid->data.size());

  Offsets offsets = determineOffsets(grid);
  size_t points_to_copy = 0;
  nanomap_msgs::msg::OpenvdbGrid::_data_type::const_iterator copy_start_pos;
  for (auto it = grid->data.begin(); it < grid->data.end(); it += grid->point_step) {
    if (validateFloatsAtPosition(it, offsets)) {
      if (points_to_copy == 0) {
        copy_start_pos = it;
      }
      ++points_to_copy;
    } else if (points_to_copy > 0) {
      filteredData.insert(
        filteredData.end(),
        copy_start_pos,
        copy_start_pos + points_to_copy * grid->point_step);
      points_to_copy = 0;
    }
  }
  // Don't forget to flush what needs to be copied
  if (points_to_copy > 0) {
    filteredData.insert(
      filteredData.end(),
      copy_start_pos,
      copy_start_pos + points_to_copy * grid->point_step);
  }

  return filteredData;
}

Offsets OpenvdbGridDisplay::determineOffsets(
  const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr grid) const
{
  Offsets offsets{
    grid->fields[findChannelIndex(grid, "x")].offset,
    grid->fields[findChannelIndex(grid, "y")].offset,
    grid->fields[findChannelIndex(grid, "z")].offset
  };
  return offsets;
}

bool OpenvdbGridDisplay::validateFloatsAtPosition(
  nanomap_msgs::msg::OpenvdbGrid::_data_type::const_iterator position,
  const Offsets offsets) const
{
  float x = *reinterpret_cast<const float *>(&*(position + offsets.x));
  float y = *reinterpret_cast<const float *>(&*(position + offsets.y));
  float z = *reinterpret_cast<const float *>(&*(position + offsets.z));

  return rviz_common::validateFloats(x) &&
         rviz_common::validateFloats(y) &&
         rviz_common::validateFloats(z);
}

void OpenvdbGridDisplay::update(float wall_dt, float ros_dt)
{
  openvdb_grid_common_->update(wall_dt, ros_dt);
}

void OpenvdbGridDisplay::reset()
{
  MFDClass::reset();
  openvdb_grid_common_->reset();
}

void OpenvdbGridDisplay::onDisable()
{
  MFDClass::onDisable();
  openvdb_grid_common_->onDisable();
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::OpenvdbGridDisplay, rviz_common::Display)
