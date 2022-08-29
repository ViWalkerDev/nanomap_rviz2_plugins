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

#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_display.hpp"

#include <memory>
#include <utility>

#include <OgreSceneNode.h>

#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_common.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/int_property.hpp"
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
: openvdb_grid_common_(std::make_unique<OpenvdbGridCommon>(this))
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
