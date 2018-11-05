/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Jon Binney */

#ifndef MOVEIT_MAP_OCCUPANCY_MAP_
#define MOVEIT_MAP_OCCUPANCY_MAP_

#include <octomap/octomap.h>
#include <boost/thread/shared_mutex.hpp>
#include <boost/function.hpp>
#include <memory>
#include <moveit/map/moveit_map.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>
namespace map
{
typedef octomap::OcTreeNode OccMapNode;

class OccMapTree : public octomap::OcTree, public map::MoveitMap {
public:

  OccMapTree(double resolution) : octomap::OcTree(resolution) {}

  OccMapTree(const std::string& filename) : octomap::OcTree(filename) {}

  virtual ~OccMapTree() = default;

#ifdef NEW_MSG_FORMAT
  virtual void useDistanceFieldMessage(const voxblox_msgs::Layer& layer) override{
    ROS_WARN("octomap can not use voxblox message!");
  }
  virtual bool getMapMsg(moveit_msgs::PlanningSceneWorld& world) const override{
    //TODO
  }
#endif

  inline virtual bool writeBinary(const std::string& filename) override{
    octomap::OcTree::writeBinary(filename);
  }
  inline virtual bool readBinary(const std::string& filename) override{
    octomap::OcTree::readBinary(filename);
  }

  inline virtual void clear() override {
    octomap::OcTree::clear();
  }
  inline static std::string name(){
    return "map::OccMapTree";
  }

  //this is the obvious case
  virtual void getOctreeMessage(octomap_msgs::Octomap* msg)const override{
    octomap_msgs::fullMapToMsg(*this, *msg);
  }

 /* inline virtual void updateScene(const std::shared_ptr<const map::MoveitMap>& me, planning_scene::PlanningScenePtr& scene, const Eigen::Affine3d& t) override{
    scene->processOctomapPtr(std::dynamic_pointer_cast(me), t);
  }*/
private:
};

  typedef std::shared_ptr<OccMapTree> OccMapTreePtr;
typedef std::shared_ptr<const OccMapTree> OccMapTreeConstPtr;
}

#endif
