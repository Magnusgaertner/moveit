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

#ifndef MOVEIT_INDUSTRIAL_TOPLEVEL_MAP_MONITOR_H
#define MOVEIT_INDUSTRIAL_TOPLEVEL_MAP_MONITOR_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <tf/tf.h>
#include <pluginlib/class_loader.hpp>

#include <moveit_msgs/SaveMap.h>
#include <moveit_msgs/LoadMap.h>
#include <moveit/occupancy_map_monitor/map_updater.h>
#include <moveit/map/moveit_map.h>

#include <boost/thread/mutex.hpp>

#include <memory>

namespace occupancy_map_monitor
{
  using map::MoveitMapPtr;
  using map::MoveitMap;
  using map::MoveitMapConstPtr;
  MOVEIT_CLASS_FORWARD(MapMonitor);

  class MapMonitor
  {
  public:

    /** @brief start the monitor (will begin updating the map */
    virtual void startMonitor() = 0;

    virtual void stopMonitor() = 0;

    /** @brief Get a pointer to the underlying map for this monitor. Lock the map before reading or writing using this
     *  pointer. The value of this pointer stays the same throughout the existance of the monitor instance. */
    //as this is part of a public interface the function is not renamed to i.e getMapPtr()
    virtual const map::MoveitMapPtr& getOcTreePtr() = 0;


    /** @brief Get a const pointer to the underlying octree for this monitor. Lock the
     *  tree before reading this pointer */
    //as this is part of a public interface the function is not renamed to i.e getMapPtr()
    virtual const map::MoveitMapConstPtr& getOcTreePtr() const = 0;


    virtual const std::string& getMapFrame() const = 0;


    virtual void setMapFrame(const std::string& frame) = 0;

    virtual double getMapResolution() const = 0;

    virtual const boost::shared_ptr<tf::Transformer>& getTFClient() const = 0;

    virtual void addUpdater(const MapUpdaterPtr& updater) = 0;

    /** \brief Add this shape to the set of shapes to be filtered out from the map */
    virtual ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape) = 0;

    /** \brief Forget about this shape handle and the shapes it corresponds to */
    virtual void forgetShape(ShapeHandle handle) = 0;

    /** @brief Set the callback to trigger when updates to the maintained map are received */
    virtual void setUpdateCallback(const boost::function<void()>& update_callback) = 0;

    virtual void setTransformCacheCallback(const TransformCacheProvider& transform_cache_callback) = 0;

    virtual void publishDebugInformation(bool flag) = 0;

    virtual bool isActive() const = 0;

  protected:
    virtual void initialize() = 0;

    /** @brief Save the current map to a binary file */
    virtual bool saveMapCallback(moveit_msgs::SaveMap::Request& request, moveit_msgs::SaveMap::Response& response) = 0;

    /** @brief Load map from a binary file (gets rid of current map data) */
    virtual bool loadMapCallback(moveit_msgs::LoadMap::Request& request, moveit_msgs::LoadMap::Response& response) = 0;

    virtual bool getShapeTransformCache(std::size_t index, const std::string& target_frame, const ros::Time& target_time,
                                ShapeTransformCache& cache) const = 0;

  };
}

#endif
