/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__OPENVDB_GRID__TRANSFORMERS__XYZ_PC_TRANSFORMER_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__OPENVDB_GRID__TRANSFORMERS__XYZ_PC_TRANSFORMER_HPP_

#include <vector>
#include <string>

#include "nanomap_msgs/msg/openvdb_grid.hpp"
#include "rviz_common/properties/property.hpp"

#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_transformer.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC XYZGridTransformer : public OpenvdbGridTransformer
{
public:
  uint8_t supports(const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr & grid) override;
  void getCounts(const std::shared_ptr<GridInfo> & grid_info, openvdb::FloatGrid::Ptr grid,
                                      std::vector<rviz_rendering::OpenvdbGrid::RenderMode> modes);

  void sizeVoxelVecs(const std::shared_ptr<GridInfo> & grid_info);
    // end namespace rviz_default_plugins
  void getNodeSizes(const std::shared_ptr<GridInfo> & grid_info);

  bool transform(int depth,
    const std::shared_ptr<GridInfo> & grid,
    uint32_t mask,
    std::vector<rviz_rendering::OpenvdbGrid::RenderMode> modes,
    const Ogre::Matrix4 & transform) override;
};
}

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__OPENVDB_GRID__TRANSFORMERS__XYZ_PC_TRANSFORMER_HPP_
