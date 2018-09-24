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
#include <moveit/distance_field/distance_field.h>
//#include "../../../../../../../voxblox/voxblox_ros/include/voxblox_ros/esdf_server.h"

namespace occupancy_map_monitor {

  class EsdfMap : public voxblox::EsdfServer , public distance_field::DistanceField, public collision_detection::MoveitMap {
  public:
    EsdfMap(double resolution):voxblox::EsdfServer(ros::NodeHandle(), ros::NodeHandle("~")), DistanceField(0,0,0,0,0,0,0) { init(); }

    EsdfMap(const std::string &filename):voxblox::EsdfServer(ros::NodeHandle(), ros::NodeHandle("~")), DistanceField(0,0,0,0,0,0,0) { init(); }
    virtual ~EsdfMap() = default;
    void init() {
     // ros::NodeHandle nh;
     // ros::NodeHandle nh_private("~");
     // vxblx.reset(new voxblox::EsdfServer(nh, nh_private));
    }

    inline virtual bool writeBinary(const std::string &filename) override {
      return saveMap(filename);
    }

    inline virtual bool readBinary(const std::string &filename) override {
      return loadMap(filename);
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

   /*bool getDistanceReq(my_collision_detection::DistanceQuerry::Request& req, my_collision_detection::DistanceQuerry::Response& res) override {
     double distance= vxblx->getEsdfMaxDistance();
     bool success = vxblx->getEsdfMapPtr()->getDistanceAtPosition(Eigen::Vector3d(req.x,req.y,req.z), &distance);
     res.distance = distance;
     return success;

   }*/


    double getUninitializedDistance() const override {
      return getEsdfMaxDistance();
    }

    double getDistance(int x, int y, int z) const override{
      ROS_ERROR("not supported");
      return 0;
      /*double distance;
      vxblx->getEsdfMapPtr()->getDistanceAtPosition(Eigen::Vector3d(x,y,z), &distance);
      return distance;*/
    }

    double getDistance(double x, double y , double z) const override{
      double distance = getEsdfMaxDistance();
      getEsdfMapPtr()->getDistanceAtPosition(Eigen::Vector3d(x,y,z), &distance);
      return distance;
    }

    int getXNumCells() const override{
      //TODO
      ROS_ERROR("not supported");
      return 0;
    }

    int getYNumCells() const override {
      //TODO
      ROS_ERROR("not supported");
      return 0;
    }

    int getZNumCells() const override{
      //TODO
      ROS_ERROR("not supported");
      return 0;
    }

    bool worldToGrid(double world_x, double world_y, double world_z, int &x, int &y,
                                                        int &z) const override{
      ROS_ERROR("not supported");
      x = world_x; y = world_y; z = world_z;
      return true;
    }


    double getDistanceGradient(double x, double y, double z, double &gradient_x,
                                                                  double &gradient_y, double &gradient_z,
                                                                  bool &in_bounds) const override{


      //ROS_INFO("%f, %f, %f distance : %f",2,0,1.5, getDistance(2.0, 0.0,1.5));

      //ROS_INFO("voxblox getDistanceGradient called");
      Eigen::Vector3d gradient(gradient_x, gradient_y, gradient_z);
      double distance = getEsdfMaxDistance();
      in_bounds = getEsdfMapPtr()->getDistanceAndGradientAtPosition(Eigen::Vector3d(x, y, z), &distance,&gradient);
      return distance;//DistanceField::getDistanceGradient(x, y, z, gradient_x, gradient_y, gradient_z, in_bounds);
    }



    void addPointsToField(const EigenSTL::vector_Vector3d &points) override{
      //TODO
      ROS_ERROR("not implemented");
      return;
    }

    void removePointsFromField(const EigenSTL::vector_Vector3d &points) override{
      //TODO
      ROS_ERROR("not implemented");
      return;
    }

    void updatePointsInField(const EigenSTL::vector_Vector3d &old_points,
                                                                const EigenSTL::vector_Vector3d &new_points) override{
      //TODO
      ROS_ERROR("not implemented");
      return;
    }

    void reset() override{
      clear();
    }

    bool isCellValid(int x, int y, int z) const override{
      getEsdfMapPtr()->isObserved(Eigen::Vector3d(x, y, z));
    }

    bool gridToWorld(int x, int y, int z, double &world_x, double &world_y,
                                                        double &world_z) const override{
      ROS_ERROR("not supported");
      world_x = x; world_y = y; world_z = z;
    }

    bool writeToStream(std::ostream &stream) const override{
      //vxblx->saveMap(stream)
      //TODO
      ROS_ERROR("not implemented");
      return false;
    }

    bool readFromStream(std::istream &stream) override{
      //return false;
      //TODO
      ROS_ERROR("not implemented");
      return false;
    }

  };

  typedef std::shared_ptr<EsdfMap> EsdfMapPtr;
  typedef std::shared_ptr<const EsdfMap> EsdfMapConstPtr;
}

#endif
