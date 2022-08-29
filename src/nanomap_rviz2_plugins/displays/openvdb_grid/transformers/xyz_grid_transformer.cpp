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

//#include "nanomap_rviz2_plugins/displays/openvdb_grid/openvdb_grid_helpers.hpp"
#include "nanomap_rviz2_plugins/displays/openvdb_grid/transformers/xyz_grid_transformer.hpp"
#ifdef foreach
  #undef foreach
#endif
#include "openvdb/openvdb.h"
#include "openvdb/io/Stream.h"

namespace rviz_default_plugins
{

uint8_t XYZGridTransformer::supports(const nanomap_msgs::msg::OpenvdbGrid::ConstSharedPtr & grid)
{

  return OpenvdbGridTransformer::Support_XYZ;
}

void XYZGridTransformer::getCounts(const std::shared_ptr<GridInfo> & grid_info, openvdb::FloatGrid::Ptr grid,
                                    std::vector<rviz_rendering::OpenvdbGrid::RenderMode> modes)
{
  using GridType = openvdb::FloatGrid;
  using TreeType = GridType::TreeType;
  using RootType = TreeType::RootNodeType;   // level 3 RootNode
  assert(RootType::LEVEL == 3);
  using Int1Type = RootType::ChildNodeType;  // level 2 InternalNode
  using Int2Type = Int1Type::ChildNodeType;  // level 1 InternalNode
  using LeafType = TreeType::LeafNodeType;
  //for (auto & node_info : node_infos_) {
    //node_info->active_counts_.clear();
    //node_info->inactive_counts_.clear();

  TreeType::NodeIter iter{grid->tree()};
  int depth = grid->tree().treeDepth();
  grid_info->node_counts_.clear();
  grid_info->node_counts_.resize(depth,0);
  //node_info->active_counts_.resize(depth,0);
  //node_info->inactive_counts_.resize(depth,0);
  for ( ; iter; ++iter) {
    ////std::cout << "1" << std::endl;
      switch (iter.getDepth()) {
        case 0:{
          RootType* node = nullptr;
          iter.getNode(node);
          if (node){
            //std::cout << (int)modes[depth-1] << std::endl;
            //std::cout << (int)rviz_rendering::OpenvdbGrid::DO_NOT_SHOW << std::endl;
            //std::cout << (int)rviz_rendering::OpenvdbGrid::SHOW_ACTIVE << std::endl;
            switch(modes[depth-1]){
              case rviz_rendering::OpenvdbGrid::DO_NOT_SHOW:
                //std::cout << "do not show voxels" << std::endl;
                grid_info->node_counts_[depth-1] = 0;
                break;
              case rviz_rendering::OpenvdbGrid::SHOW_ACTIVE:
              //std::cout << "show voxels" << std::endl;
                grid_info->node_counts_[depth-1] = node->onVoxelCount();
                break;
              case rviz_rendering::OpenvdbGrid::SHOW_INACTIVE:
              //std::cout << "show inactive voxels" << std::endl;
                grid_info->node_counts_[depth-1] = node->offVoxelCount();
                break;
            }
          }
          break;
        }
        case 1:
        {
          Int1Type* node = nullptr;
          iter.getNode(node);
          if (node){
            switch(modes[iter.getDepth()-1]){
              case rviz_rendering::OpenvdbGrid::DO_NOT_SHOW:
                grid_info->node_counts_[iter.getDepth()-1] = 0;
                break;
              case rviz_rendering::OpenvdbGrid::SHOW_ACTIVE:
                if(node->onVoxelCount()>0){
                  grid_info->node_counts_[iter.getDepth()-1] += 1;
                }
                break;
              case rviz_rendering::OpenvdbGrid::SHOW_INACTIVE:
                if(node->onVoxelCount()==0){
                  grid_info->node_counts_[iter.getDepth()-1] += 1;
                }
                break;
            }
          }
          break;
        }
        case 2:
        {
          Int2Type* node = nullptr;
          iter.getNode(node);
          if (node){
            switch(modes[iter.getDepth()-1]){
              case rviz_rendering::OpenvdbGrid::DO_NOT_SHOW:
                grid_info->node_counts_[iter.getDepth()-1] = 0;
                break;
              case rviz_rendering::OpenvdbGrid::SHOW_ACTIVE:
                if(node->onVoxelCount()>0){
                  grid_info->node_counts_[iter.getDepth()-1] += 1;
                }
                break;
              case rviz_rendering::OpenvdbGrid::SHOW_INACTIVE:
                if(node->onVoxelCount()==0){
                  grid_info->node_counts_[iter.getDepth()-1] += 1;
                }
                break;
            }
          }
          break;
        }
        case 3:
        {
          LeafType* node = nullptr;
          iter.getNode(node);
          if (node){
            switch(modes[iter.getDepth()-1]){
              case rviz_rendering::OpenvdbGrid::DO_NOT_SHOW:
                grid_info->node_counts_[iter.getDepth()-1] = 0;
                break;
              case rviz_rendering::OpenvdbGrid::SHOW_ACTIVE:
                if(node->onVoxelCount()>0){
                  grid_info->node_counts_[iter.getDepth()-1] += 1;
                }
                break;
              case rviz_rendering::OpenvdbGrid::SHOW_INACTIVE:
                if(node->onVoxelCount()==0){
                  grid_info->node_counts_[iter.getDepth()-1] += 1;
                }
                break;
            }
          }
          break;
        }
      }
    }
  }


void XYZGridTransformer::sizeVoxelVecs(const std::shared_ptr<GridInfo> & grid_info){
  rviz_rendering::OpenvdbGrid::Point default_pt = {Ogre::Vector3::ZERO, Ogre::ColourValue(1, 1, 1)};
  int d = 0;
  for(auto & node_info : grid_info->node_infos_){
    node_info->transformed_points_.resize(grid_info->node_counts_[d], default_pt);
    d++;
  }
}

void XYZGridTransformer::getNodeSizes(const std::shared_ptr<GridInfo> & grid_info){
  //std::vector<float> sizes;
  //for(auto & node_info : node_infos_){
  for(int a = 0; a < GRIDDEPTH-1; a++){
    float size = 1;
    for(int b = a; b < GRIDDEPTH-1;b++){
      size *= std::pow(2, grid_info->grid_exponents_[b]);
    }
    size *= grid_info->voxel_size_;
    //std::cout << "sizes = " << size << std::endl;
    //std::vector<float> vsize = std::vector<float>(3,size);
    grid_info->sizes_[a] = (size);
  }
}

bool XYZGridTransformer::transform(
  int depth,
  const std::shared_ptr<GridInfo> & grid_info,
  uint32_t mask,
  std::vector<rviz_rendering::OpenvdbGrid::RenderMode> modes,
  const Ogre::Matrix4 & transform)
{
  (void) depth;
  (void) transform;
  using GridType = openvdb::FloatGrid;
  using TreeType = GridType::TreeType;
  using RootType = TreeType::RootNodeType;   // level 3 RootNode
  assert(RootType::LEVEL == 3);
  using Int1Type = RootType::ChildNodeType;  // level 2 InternalNode
  using Int2Type = Int1Type::ChildNodeType;  // level 1 InternalNode
  using LeafType = TreeType::LeafNodeType;
  //std::cout << "creating stringstream object" << std::endl;
  //std::cout << grid_info->message_->size << std::endl;
  std::istringstream istr(std::string((reinterpret_cast<const char*>(grid_info->message_->data.data())),
                                        grid_info->message_->size), std::ios_base::binary);
  //if(openvdb::io::Stream strm(istr));
  //std::cout << "creating grid stream object " << std::endl;
  //std::cout << grid_info->message_->size << std::endl;
  std::shared_ptr<openvdb::io::Stream> strmptr;
  openvdb::GridPtrVecPtr grids;
  try{
    //std::cout << grid_info->message_->size << std::endl;
    strmptr = std::make_shared<openvdb::io::Stream>(istr);
    grids = strmptr->getGrids();
  }
  catch(const std::runtime_error& e){
    return false;
  }
  ////std::cout << "1" << std::endl;
  //std::cout << "getting grids from stream" << std::endl;
  //auto
  ////std::cout << "2" << std::endl;
  //node_info->openvdb_grid_->clear();
  ////std::cout << "3" << std::endl;
  openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>((*grids)[0]);
  int grid_depth = grid->tree().treeDepth();
  getCounts(grid_info, grid, modes);
  sizeVoxelVecs(grid_info);
  grid_info->voxel_size_ = grid->voxelSize()[0];
  std::vector<std::string> v_s;
  std::istringstream iss(grid->tree().type());
  std::string s;
  ////std::cout << "VOXEL COUNT! = " << grid_info->node_counts_[3] << std::endl;
  grid_info->grid_exponents_.clear();
  while(std::getline(iss, s, '_')){
      v_s.push_back(s);
  }
  for(int i = 2; i<v_s.size(); i++){
    grid_info->grid_exponents_.push_back(stoi(v_s[i]));
  }

  grid_info->grid_exponents_.push_back(0);
  getNodeSizes(grid_info);
  std::vector<std::vector<rviz_rendering::OpenvdbGrid::Point>::iterator> voxel_iterators;
  for(auto & node_info : grid_info->node_infos_){
    voxel_iterators.push_back(node_info->transformed_points_.begin());
  }
  TreeType::NodeIter iter{grid->tree()};
  openvdb::Vec3d worldCoord;
  for ( ; iter; ++iter) {
    switch (iter.getDepth()) {
      case 1:
      {
        Int1Type* node = nullptr;
        iter.getNode(node);
        if (node){
          switch(modes[iter.getDepth()-1]){
            case rviz_rendering::OpenvdbGrid::DO_NOT_SHOW:
              //do nothing, we are not displaying this node
              break;
            case rviz_rendering::OpenvdbGrid::SHOW_ACTIVE:
              if(node->onVoxelCount() > 0){
              //if node is active, add to display vector
                worldCoord = grid->indexToWorld(iter.getCoord());
                voxel_iterators[iter.getDepth()-1]->position.x = worldCoord.x()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]->position.y = worldCoord.y()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]->position.z = worldCoord.z()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]++;
                ////std::cout << "xCoord = " << worldCoord.z() << "yCoord = " << worldCoord.y() << "zCoord = " << worldCoord.z() << std::endl;
              }
              break;
            case rviz_rendering::OpenvdbGrid::SHOW_INACTIVE:
              if(node->onVoxelCount() == 0){
              //if node is inactive add to display
                worldCoord = grid->indexToWorld(iter.getCoord());
                voxel_iterators[iter.getDepth()-1]->position.x = worldCoord.x()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]->position.y = worldCoord.y()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]->position.z = worldCoord.z()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]++;
              }
              break;
          }
        }
       break;
      }
      case 2:
      {
        Int2Type* node = nullptr;
        iter.getNode(node);
        if (node){
          switch(modes[iter.getDepth()-1]){
            case rviz_rendering::OpenvdbGrid::DO_NOT_SHOW:
              //do nothing, we are not displaying this node
              break;
            case rviz_rendering::OpenvdbGrid::SHOW_ACTIVE:
              if(node->onVoxelCount() > 0){
              //if node is active, add to display vector
                worldCoord = grid->indexToWorld(iter.getCoord());
                voxel_iterators[iter.getDepth()-1]->position.x = worldCoord.x()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]->position.y = worldCoord.y()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]->position.z = worldCoord.z()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]++;
                ////std::cout << "xCoord = " << worldCoord.z() << "yCoord = " << worldCoord.y() << "zCoord = " << worldCoord.z() << std::endl;

              }
              break;
            case rviz_rendering::OpenvdbGrid::SHOW_INACTIVE:
              if(node->onVoxelCount() == 0){
              //if node is inactive add to display
                worldCoord = grid->indexToWorld(iter.getCoord());
                voxel_iterators[iter.getDepth()-1]->position.x = worldCoord.x()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]->position.y = worldCoord.y()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]->position.z = worldCoord.z()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]++;
              }
              break;
          }
        }
       break;
      }
      case 3:
      {
        LeafType* node = nullptr;
        iter.getNode(node);
        if (node){
          switch(modes[iter.getDepth()-1]){
            case rviz_rendering::OpenvdbGrid::DO_NOT_SHOW:
              //do nothing, we are not displaying this node
              break;
            case rviz_rendering::OpenvdbGrid::SHOW_ACTIVE:
              if(node->onVoxelCount() > 0){
              //if node is active, add to display vector
                worldCoord = grid->indexToWorld(iter.getCoord());
                voxel_iterators[iter.getDepth()-1]->position.x = worldCoord.x()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]->position.y = worldCoord.y()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]->position.z = worldCoord.z()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]++;
              }
              break;
            case rviz_rendering::OpenvdbGrid::SHOW_INACTIVE:
              if(node->onVoxelCount() == 0){
              //if node is inactive add to display
                worldCoord = grid->indexToWorld(iter.getCoord());
                voxel_iterators[iter.getDepth()-1]->position.x = worldCoord.x()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]->position.y = worldCoord.y()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]->position.z = worldCoord.z()+grid_info->sizes_[iter.getDepth()-1]/2;
                voxel_iterators[iter.getDepth()-1]++;
              }
              break;
          }
        }
        break;
      }
    }
  }
  ////std::cout << "6" << std::endl;
  ////std::cout << "activity_mask_size" << activity_mask.size() << std::endl;
  //openvdb::FloatGrid::IteratorvIter
  switch(modes[grid_depth-1]){
    case rviz_rendering::OpenvdbGrid::DO_NOT_SHOW:
    ////std::cout << "7" << std::endl;
      break;
    case rviz_rendering::OpenvdbGrid::SHOW_ACTIVE:
    {
      ////std::cout << "8" << std::endl;
      auto vIter = grid->beginValueOn();
      for(; vIter; ++vIter, ++voxel_iterators[grid_depth-1])
      {
        if(vIter.isVoxelValue()){
          worldCoord = grid->indexToWorld(vIter.getCoord());
          voxel_iterators[grid_depth-1]->position.x = worldCoord.x()+grid_info->sizes_[grid_depth-1]/2;
          voxel_iterators[grid_depth-1]->position.y = worldCoord.y()+grid_info->sizes_[grid_depth-1]/2;
          voxel_iterators[grid_depth-1]->position.z = worldCoord.z()+grid_info->sizes_[grid_depth-1]/2;
        }
      }
      break;
    }
    case rviz_rendering::OpenvdbGrid::SHOW_INACTIVE:
    {
      ////std::cout << "9 " << std::endl;
      auto vIter = grid->beginValueOff();
      for(; vIter; ++vIter, ++voxel_iterators[grid_depth-1])
      {
        if(vIter.isVoxelValue()){
          worldCoord = grid->indexToWorld(vIter.getCoord());
          voxel_iterators[grid_depth-1]->position.x = worldCoord.x()+grid_info->sizes_[grid_depth-1]/2;
          voxel_iterators[grid_depth-1]->position.y = worldCoord.y()+grid_info->sizes_[grid_depth-1]/2;
          voxel_iterators[grid_depth-1]->position.z = worldCoord.z()+grid_info->sizes_[grid_depth-1]/2;
        }
      }
      break;
    }
  }
  return true;
}









//
//   auto vIter = openvdb_grid->beginValueOn();
//   auto points_out_iter = points_out.begin();
//   openvdb::Vec3d worldCoord;
//   for(; vIter; ++vIter, ++points_out_iter)
//   {
//     if(vIter.isVoxelValue()){
//       worldCoord = openvdb_grid->indexToWorld(vIter.getCoord());
//       points_out_iter->position.x = worldCoord.x();
//       points_out_iter->position.y = worldCoord.y();
//       points_out_iter->position.z = worldCoord.z();
//     }
//   }
//   openvdb_grid->clear();
//   //for (V_OpenvdbGridPoint::iterator iter = points_out.begin(); iter != points_out.end();
//   //  ++iter, point_x += point_step,
//   //  point_y += point_step, point_z += point_step)
//   //{
//   //  iter->position.x = *reinterpret_cast<const float *>(point_x);
//   //  iter->position.y = *reinterpret_cast<const float *>(point_y);
//   //  iter->position.z = *reinterpret_cast<const float *>(point_z);
//   //}
//
//   return true;
// }

}  // end namespace rviz_default_plugins
