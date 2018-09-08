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

/* Author: Magnus Gärtner */

#ifndef MOVEIT_OCCUPANCY_MAP_MONITOR_ESDF_MAP_
#define MOVEIT_OCCUPANCY_MAP_MONITOR_ESDF_MAP_

//#include <octomap/octomap.h>
#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>
#include <moveit/collision_detection/moveit_map.h>

namespace occupancy_map_monitor {

  class EsdfMap : public collision_detection::MoveitMap, public voxblox::EsdfServer {
  public:
    EsdfMap(double resolution):voxblox::EsdfServer(ros::NodeHandle(), ros::NodeHandle("~")) { init(); }

    EsdfMap(const std::string &filename):voxblox::EsdfServer(ros::NodeHandle(), ros::NodeHandle("~")) { init(); }

    void init() {
     // ros::NodeHandle nh;
     // ros::NodeHandle nh_private("~");
     // vxblx.reset(new voxblox::EsdfServer(nh, nh_private));
    }

    inline virtual bool writeBinary(const std::string &filename) override {
      saveMap(filename);
      return true;
    }

    inline virtual bool readBinary(const std::string &filename) override {
      loadMap(filename);
      return true;
    }
    inline virtual void clear() override {
      voxblox::EsdfServer::clear();
    }

    inline static std::string name(){
      return "occupancy_map_monitor::EsdfMap";
    }

   /* inline virtual void updateScene(const std::shared_ptr<const collision_detection::MoveitMap>& me, planning_scene::PlanningScenePtr& scene, const Eigen::Affine3d& t) override{
      ROS_ERROR("UpdateScene called");

      collision_detection::MoveitMapConstPtr map = scene->getWorld()->getMapPtr();
      // if the octree pointer changed, update the structure
      if(map!= me)scene->getWorld()->setMapPtr(me);
      scene->getWorld()->setMapPose(t);
    }*/
  };

  typedef std::shared_ptr<EsdfMap> EsdfMapPtr;
  typedef std::shared_ptr<const EsdfMap> EsdfMapConstPtr;
}

#endif
