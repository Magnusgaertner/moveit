/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef MOVEIT_OCCUPANCY_MAP_MONITOR_OCCUPANCY_MAP_UPDATER_
#define MOVEIT_OCCUPANCY_MAP_MONITOR_OCCUPANCY_MAP_UPDATER_

#include <moveit/macros/class_forward.h>
#include <moveit/occupancy_map_monitor/occupancy_map.h>
#include <moveit/occupancy_map_monitor/map_updater.h>
#include <geometric_shapes/shapes.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace occupancy_map_monitor
{

  template<typename MapType>
class OccupancyMapMonitor;

/** \brief Base class for classes which update the occupancy map.
 */
  template <typename  MapType>
class OccupancyMapUpdater: public MapUpdater
{
  typedef std::shared_ptr<MapType> MapTypePtr;
  typedef std::shared_ptr<const MapType> MapTypeConstPtr;
public:
  OccupancyMapUpdater(const std::string& type);
  virtual ~OccupancyMapUpdater();

  /** \brief This is the first function to be called after construction */
  virtual void setMonitor(MapMonitor* monitor) override ;

  /** @brief Set updater params using struct that comes from parsing a yaml string. This must be called after
   * setMonitor() */
  virtual bool setParams(XmlRpc::XmlRpcValue& params) = 0;

  /** @brief Do any necessary setup (subscribe to ros topics, etc.). This call assumes setMonitor() and setParams() have
   * been previously called. */
  virtual bool initialize() = 0;

  virtual void start() = 0;

  virtual void stop() = 0;

  virtual ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape) = 0;

  virtual void forgetShape(ShapeHandle handle) = 0;

  virtual const std::string& getType() const override
  {
    return type_;
  }

  virtual void setTransformCacheCallback(const TransformCacheProvider& transform_callback) override
  {
    transform_provider_callback_ = transform_callback;
  }

  virtual void publishDebugInformation(bool flag) override
  {
    debug_info_ = flag;
  }

protected:
  OccupancyMapMonitor<MapType>* monitor_;
  std::string type_;
  MapTypePtr tree_;
  TransformCacheProvider transform_provider_callback_;
  ShapeTransformCache transform_cache_;
  bool debug_info_;

  virtual bool updateTransformCache(const std::string& target_frame, const ros::Time& target_time) override;
};
}
#include <moveit/occupancy_map_monitor/occupancy_map_updater.tpp>
#endif
