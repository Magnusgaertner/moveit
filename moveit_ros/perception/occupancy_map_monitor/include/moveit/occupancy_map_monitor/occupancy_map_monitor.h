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

#ifndef MOVEIT_PERCEPTION_OCCUPANCY_MAP_MONITOR_
#define MOVEIT_PERCEPTION_OCCUPANCY_MAP_MONITOR_

#include <vector>
#include <string>
#include <ros/ros.h>
#include <tf/tf.h>
#include <pluginlib/class_loader.hpp>

#include <moveit_msgs/SaveMap.h>
#include <moveit_msgs/LoadMap.h>
#include <moveit/occupancy_map_monitor/map_monitor.h>
#include <moveit/occupancy_map_monitor/occupancy_map.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>


#include <boost/thread/mutex.hpp>
#include <memory>

namespace occupancy_map_monitor
{
  template <typename MapType>
class OccupancyMapMonitor:public MapMonitor
{
  MOVEIT_DECLARE_PTR(MapType, MapType);
  MOVEIT_DECLARE_PTR(OccupancyMapUpdater, OccupancyMapUpdater<MapType>);
public:
  OccupancyMapMonitor(const boost::shared_ptr<tf::Transformer>& tf, const std::string& map_frame = "",
                      double map_resolution = 0.0);
  OccupancyMapMonitor(double map_resolution = 0.0);
  OccupancyMapMonitor(const boost::shared_ptr<tf::Transformer>& tf, ros::NodeHandle& nh,
                      const std::string& map_frame = "", double map_resolution = 0.0);

  ~OccupancyMapMonitor();

  /** @brief start the monitor (will begin updating the map */
  virtual void startMonitor() override ;

  virtual void stopMonitor() override ;

  /** @brief Get a pointer to the underlying map for this monitor. Lock the map before reading or writing using this
   *  pointer. The value of this pointer stays the same throughout the existance of the monitor instance. */
  //as this is part of a public interface the function is not renamed to i.e getMapPtr()
  virtual const MoveitMapPtr& getOcTreePtr() override
  {
    return base_map_;
  }


  /** @brief Get a const pointer to the underlying map for this monitor. Lock the
   *  map before reading this pointer */
  //as this is part of a public interface the function is not renamed to i.e getMapPtr()
  virtual const MoveitMapConstPtr& getOcTreePtr() const override
  {
    return base_map_const_;
  }

  const MapTypePtr getMapPtr(){
    return map_;
  }

  const MapTypeConstPtr getMapPtr() const{
    return map_const_;
  }


  virtual const std::string& getMapFrame() const override
  {
    return map_frame_;
  }

  virtual void setMapFrame(const std::string& frame) override;

  virtual double getMapResolution() const override
  {
    return map_resolution_;
  }

  virtual const boost::shared_ptr<tf::Transformer>& getTFClient() const override
  {
    return tf_;
  }

  virtual void addUpdater(const MapUpdaterPtr& updater) override;

  /** \brief Add this shape to the set of shapes to be filtered out from the map */
  virtual ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape) override;

  /** \brief Forget about this shape handle and the shapes it corresponds to */
  virtual void forgetShape(ShapeHandle handle) override;

  /** @brief Set the callback to trigger when updates to the maintained map are received */
  virtual void setUpdateCallback(const boost::function<void()>& update_callback) override
  {
    map_->setUpdateCallback(update_callback);
  }

  virtual void setTransformCacheCallback(const TransformCacheProvider& transform_cache_callback) override;

  virtual void publishDebugInformation(bool flag) override;

  virtual bool isActive() const override
  {
    return active_;
  }

private:
  virtual void initialize() override;

  /** @brief Save the current map to a binary file */
  virtual bool saveMapCallback(moveit_msgs::SaveMap::Request& request, moveit_msgs::SaveMap::Response& response) override;

  /** @brief Load map from a binary file (gets rid of current map data) */
  virtual bool loadMapCallback(moveit_msgs::LoadMap::Request& request, moveit_msgs::LoadMap::Response& response) override;

  virtual bool getShapeTransformCache(std::size_t index, const std::string& target_frame, const ros::Time& target_time,
                              ShapeTransformCache& cache) const override ;

  boost::shared_ptr<tf::Transformer> tf_;
  std::string map_frame_;
  double map_resolution_;
  boost::mutex parameters_lock_;

  MapTypePtr map_;
  MapTypeConstPtr map_const_;
  MoveitMapPtr base_map_;
  MoveitMapConstPtr base_map_const_;
  
  std::unique_ptr<pluginlib::ClassLoader<OccupancyMapUpdater<MapType> > > updater_plugin_loader_;
  std::vector<OccupancyMapUpdaterPtr> map_updaters_;
  std::vector<std::map<ShapeHandle, ShapeHandle> > mesh_handles_;
  TransformCacheProvider transform_cache_callback_;
  bool debug_info_;

  std::size_t mesh_handle_count_;

  ros::NodeHandle root_nh_;
  ros::NodeHandle nh_;
  ros::ServiceServer save_map_srv_;
  ros::ServiceServer load_map_srv_;

  bool active_;
};
}

#endif
