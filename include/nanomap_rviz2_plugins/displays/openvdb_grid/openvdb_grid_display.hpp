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

#ifndef NANOMAP_RVIZ2_PLUGINS__DISPLAYS__OPENVDB_GRID__OPENVDB_GRID_DISPLAY_HPP_
#define NANOMAP_RVIZ2_PLUGINS__DISPLAYS__OPENVDB_GRID__OPENVDB_GRID_DISPLAY_HPP_

#include <deque>
#include <memory>
#include <queue>
#include <vector>

#include "nanomap_msgs/msg/openvdb_grid.hpp"

#include "rviz_common/message_filter_display.hpp"

#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_common.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{

class IntProperty;

}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace displays
{

/**
 * \class OpenvdbGridDisplay
 * \brief Displays a point grid of type sensor_msgs::OpenvdbGrid
 *
 * By default it will assume channel 0 of the grid is an intensity value, and will color them by intensity.
 * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r, g and b
 * all being 8 bits.
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC OpenvdbGridDisplay : public
  rviz_common::MessageFilterDisplay<nanomap_msgs::msg::OpenvdbGrid>
{
public:
  OpenvdbGridDisplay();

  void reset() override;

  void update(float wall_dt, float ros_dt) override;

  void onDisable() override;

protected:
  /** @brief Do initialization. Overridden from MessageFilterDisplay. */
  void onInitialize() override;

  /** @brief Process a single message.  Overridden from MessageFilterDisplay. */
  void processMessage(nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr grid) override;

  std::unique_ptr<OpenvdbGridCommon> openvdb_grid_common_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // NANOMAP_RVIZ2_PLUGINS__DISPLAYS__OPENVDB_GRID__OPENVDB_GRID_DISPLAY_HPP_
