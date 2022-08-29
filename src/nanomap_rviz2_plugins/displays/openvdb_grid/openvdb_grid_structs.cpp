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

#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_structs.hpp"

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

NodeInfo::NodeInfo()
: manager_(nullptr),
  scene_node_(nullptr)
{}

NodeInfo::~NodeInfo()
{
  clear();
}

void NodeInfo::clear()
{
  if (scene_node_) {
    manager_->destroySceneNode(scene_node_);
    scene_node_ = nullptr;
  }
}

GridInfo::GridInfo()
: position_(Ogre::Vector3::ZERO)
{
  for(auto & node_info : node_infos_){
    node_info = std::make_shared<NodeInfo>();
  }
}

GridInfo::~GridInfo()
{
  clear();
}

void GridInfo::clear()
{
  for(auto node : node_infos_){
    node->clear();
  }
}

}
