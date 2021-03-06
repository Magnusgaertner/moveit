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

#ifndef MOVEIT_MAP_ESDF_MAP_
#define MOVEIT_MAP_ESDF_MAP_


#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>
#include <moveit/map/moveit_map.h>
#include <moveit/distance_field/distance_field.h>
#include <voxblox_octomap_conversions/octomap_conversions.h>
namespace map {

  class EsdfMap : public voxblox::EsdfServer , public distance_field::DistanceField, public map::MoveitMap {
  public:
    EsdfMap(double resolution);

    EsdfMap(const std::string &filename);
    virtual ~EsdfMap() = default;
    void init() ;

#ifdef NEW_MSG_FORMAT
    virtual void useDistanceFieldMessage(const voxblox_msgs::Layer& layer) override;
    virtual bool getMapMsg(moveit_msgs::PlanningSceneWorld& world) const override;
#endif

    virtual bool writeBinary(const std::string &filename) override ;

    virtual bool readBinary(const std::string &filename) override ;
    virtual void clear() override ;

    static std::string name();


    virtual void getOctreeMessage(octomap_msgs::Octomap* msg)const override;


    double getUninitializedDistance() const override ;


    double getDistance(double x, double y , double z) const override;





    double getDistanceGradient(double x, double y, double z, double &gradient_x,
                                                                  double &gradient_y, double &gradient_z,
                                                                  bool &in_bounds) const override;



    void addPointsToField(const EigenSTL::vector_Vector3d &points) override;

    void removePointsFromField(const EigenSTL::vector_Vector3d &points) override;

    /***
     * Taken from PropagationDistanceField
     * @param old_points
     * @param new_points
     */
    void updatePointsInField(const EigenSTL::vector_Vector3d &old_points,
                                                                const EigenSTL::vector_Vector3d &new_points) override;

    void reset() override;

    bool isCellValid(int x, int y, int z) const override;

    bool gridToWorld(int x, int y, int z, double &world_x, double &world_y,
                                                        double &world_z) const override;

    bool writeToStream(std::ostream &stream) const override;

    bool readFromStream(std::istream &stream) override;

      double getDistance(int x, int y, int z) const override {
          return 0;
      }

      int getXNumCells() const override {
          return 0;
      }

      int getYNumCells() const override {
          return 0;
      }

      int getZNumCells() const override {
          return 0;
      }


  private:
      bool worldToGrid(double world_x, double world_y, double world_z, int &x, int &y,
                       int &z)const ;
      bool convert_to_octree = false;

      /** Typedef for set of integer indices, taken from PropagationDistanceField */
     // typedef std::set<Eigen::Vector3i, compareEigen_Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> VoxelSet;
  };

  typedef std::shared_ptr<EsdfMap> EsdfMapPtr;
  typedef std::shared_ptr<const EsdfMap> EsdfMapConstPtr;
}

#endif
