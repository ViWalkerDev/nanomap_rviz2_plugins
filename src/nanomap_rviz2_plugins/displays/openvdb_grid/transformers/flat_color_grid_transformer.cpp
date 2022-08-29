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

#include "rviz_common/properties/color_property.hpp"
//#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_helpers.hpp"

#include "nanomap_rviz2_plugins/displays/openvdb_grid/transformers/flat_color_grid_transformer.hpp"

namespace rviz_default_plugins
{

uint8_t FlatColorGridTransformer::supports(
  const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr & grid)
{
  (void) grid;
  return OpenvdbGridTransformer::Support_Color;
}

uint8_t FlatColorGridTransformer::score(const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr & grid)
{
  (void) grid;
  return 0;
}

bool FlatColorGridTransformer::transform(
  int depth,
  const std::shared_ptr<GridInfo> & grid_info,
  uint32_t mask,
  std::vector<rviz_rendering::OpenvdbGrid::RenderMode> modes,
  const Ogre::Matrix4 & transform)
{
  (void) transform;
  if (!(mask & OpenvdbGridTransformer::Support_Color)) {
    return false;
  }

  //for(auto & node_info : grid_info->node_infos_){
    Ogre::ColourValue color = color_property_[depth]->getOgreColor();
    const uint32_t num_points = grid_info->node_infos_[depth]->transformed_points_.size();//grid->width * grid->height;
    for (uint32_t i = 0; i < num_points; ++i) {
      grid_info->node_infos_[depth]->transformed_points_[i].color = color;
    }
  //}
  return true;
}

void FlatColorGridTransformer::createProperties(
  rviz_common::properties::Property * parent_property,
  uint32_t mask,
  QList<rviz_common::properties::Property *> & out_props)
{
  std::vector<std::string> names = {"Level 1 Color","Level 2 Color","Leaf Color","Voxel Color"};
  for(int x = 0; x < 4; x++){
    if (mask & OpenvdbGridTransformer::Support_Color) {
      color_property_[x] = new rviz_common::properties::ColorProperty(
        names[x].c_str(), Qt::white,
        "Color to assign to every coord.",
        parent_property, SIGNAL(needRetransform()),
        this);
      out_props.push_back(color_property_[x]);
    }
  }
}

}  // end namespace rviz_default_plugins
