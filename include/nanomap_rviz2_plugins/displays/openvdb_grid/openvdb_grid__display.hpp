/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2018, TNG Technology Consulting GmbH.
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

#ifndef NANOMAP_RVIZ2_PLUGINS__DISPLAYS__OPENVDB_GRID__OPENVDB_GRID_SCALAR_DISPLAY_HPP_
#define NANOMAP_RVIZ2_PLUGINS__DISPLAYS__OPENVDB_GRID__OPENVDB_GRID_SCALAR_DISPLAY_HPP_

#include <memory>
#include <string>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/validate_floats.hpp"
#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_common.hpp"
#include "rviz_default_plugins/visibility_control.hpp"
#include "nanomap_rviz2_plugins/rendering/objects/openvdb_grid.hpp"

#include "nanomap_msgs/msg/openvdb_grid.hpp"
#include "std_msgs/msg/header.hpp"

namespace rviz_default_plugins
{

namespace displays
{

/// This is the parent class for several scalar type message displays
/// that use a point grid to display their data
/**
 * \class OpenvdbGridScalarDisplay
 */

template<typename MessageType>
class RVIZ_DEFAULT_PLUGINS_PUBLIC OpenvdbGridScalarDisplay
  : public rviz_common::MessageFilterDisplay<MessageType>
{
public:
  OpenvdbGridScalarDisplay()
  : openvdb_grid_common_(std::make_shared<OpenvdbGridCommon>(this))
  {}

  ~OpenvdbGridScalarDisplay() override = default;

  std::shared_ptr<nanomap_msgs::msg::OpenvdbGrid> createOpenvdbGridMessage(
    const std_msgs::msg::Header & header, double scalar_value, const std::string & channelName)
  {
    auto openvdb_grid_message = std::make_shared<nanomap_msgs::msg::OpenvdbGrid>();
    unsigned int field_size_total = 0;

    openvdb_grid_message->header = header;

    field_size_total = addFieldsAndReturnSize(openvdb_grid_message, channelName);
    openvdb_grid_message->data.resize(field_size_total);

    copyCoordinates(openvdb_grid_message);
    copyScalarValue(openvdb_grid_message, scalar_value);

    openvdb_grid_message->height = 1;
    openvdb_grid_message->width = 1;
    openvdb_grid_message->is_bigendian = false;
    openvdb_grid_message->point_step = field_size_total;
    openvdb_grid_message->row_step = 1;

    return openvdb_grid_message;
  }

protected:
  virtual void setInitialValues() = 0;
  virtual void hideUnneededProperties() = 0;

  std::shared_ptr<rviz_default_plugins::OpenvdbGridCommon> openvdb_grid_common_;

private:
  void onInitialize() override
  {
    rviz_common::MessageFilterDisplay<MessageType>::onInitialize();
    openvdb_grid_common_->initialize(
      this->context_, this->scene_node_);
    setInitialValues();
  }

  void update(float wall_dt, float ros_dt) override
  {
    openvdb_grid_common_->update(wall_dt, ros_dt);
    hideUnneededProperties();
  }

  void onEnable() override
  {
    rviz_common::MessageFilterDisplay<MessageType>::onEnable();
  }

  void onDisable() override
  {
    rviz_common::MessageFilterDisplay<MessageType>::onDisable();
    openvdb_grid_common_->onDisable();
  }

  void reset() override
  {
    rviz_common::MessageFilterDisplay<MessageType>::reset();
    openvdb_grid_common_->reset();
  }

  int addFieldsAndReturnSize(
    std::shared_ptr<nanomap_msgs::msg::OpenvdbGrid> openvdb_grid_message,
    const std::string & channelName)
  {
    unsigned int field_size_increment = 0;

    field_size_increment =
      addField32andReturnOffset(openvdb_grid_message, field_size_increment, "x");
    field_size_increment =
      addField32andReturnOffset(openvdb_grid_message, field_size_increment, "y");
    field_size_increment =
      addField32andReturnOffset(openvdb_grid_message, field_size_increment, "z");
    field_size_increment =
      addField64andReturnOffset(openvdb_grid_message, field_size_increment, channelName);
    return field_size_increment;
  }

  int addField32andReturnOffset(
    std::shared_ptr<nanomap_msgs::msg::OpenvdbGrid> openvdb_grid_message,
    unsigned int offset, std::string field_name)
  {
    sensor_msgs::msg::PointField field;
    field.name = field_name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    openvdb_grid_message->fields.push_back(field);
    offset += field_size_32_;

    return offset;
  }

  int addField64andReturnOffset(
    std::shared_ptr<nanomap_msgs::msg::OpenvdbGrid> openvdb_grid_message,
    unsigned int offset, std::string field_name)
  {
    sensor_msgs::msg::PointField field;
    field.name = field_name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT64;
    field.count = 1;
    openvdb_grid_message->fields.push_back(field);
    offset += field_size_64_;

    return offset;
  }

  void copyCoordinates(std::shared_ptr<nanomap_msgs::msg::OpenvdbGrid> openvdb_grid_message)
  {
    float coordinate_value = 0.0;

    for (int i = 0; i < 3; i++) {
      memcpy(
        &openvdb_grid_message->data[openvdb_grid_message->fields[i].offset],
        &coordinate_value, field_size_32_);
    }
  }

  void copyScalarValue(
    std::shared_ptr<nanomap_msgs::msg::OpenvdbGrid> openvdb_grid_message, double scalar_value)
  {
    memcpy(
      &openvdb_grid_message->data[openvdb_grid_message->fields[3].offset],
      &scalar_value, field_size_64_);
  }

  const unsigned int field_size_32_ = 4;
  const unsigned int field_size_64_ = 8;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // NANOMAP_RVIZ2_PLUGINS__DISPLAYS__OPENVDB_GRID__OPENVDB_GRID_SCALAR_DISPLAY_HPP_
