//
// Created by magnus on 04.09.18.
//

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

/* Author: Jon Binney, Ioan Sucan */


#include <moveit/pointcloud_octomap_updater/pointcloud_esdf_updater.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <XmlRpcException.h>
#include <swri_profiler/profiler.h>
#include <memory>

namespace occupancy_map_monitor {
    bool PointCloudEsdfUpdater::setParams(XmlRpc::XmlRpcValue &params) {
        bool res = PointCloudMapUpdater::setParams(params);
        try {
            if (params.hasMember("filter_pointcloud"))
                MapUpdater::readXmlParam(params, "filter_pointcloud", &filter_pointcloud);
            if (params.hasMember("use_freespace_pointcloud"))
                MapUpdater::readXmlParam(params, "use_freespace_pointcloud", &use_freespace_pointcloud);
        }
        catch (XmlRpc::XmlRpcException &ex) {
            ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
            return false;
        }

        return res;
    }

    void PointCloudEsdfUpdater::cloudMsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
        SWRI_PROFILE("cloudMsgCallback");
        if (max_update_rate_ > 0) {
            // ensure we are not updating the octomap representation too often
            if (ros::Time::now() - last_update_time_ <= ros::Duration(1.0 / max_update_rate_))
                return;
            last_update_time_ = ros::Time::now();
        }

        if (monitor_->getMapFrame().empty())
            monitor_->setMapFrame(cloud_msg->header.frame_id);

        /* get transform for cloud into map frame */
        tf::StampedTransform map_H_sensor;
        if (monitor_->getMapFrame() == cloud_msg->header.frame_id)
            map_H_sensor.setIdentity();
        else
        {
            if (tf_)
            {
                try
                {
                    tf_->lookupTransform(monitor_->getMapFrame(), cloud_msg->header.frame_id, cloud_msg->header.stamp,
                                         map_H_sensor);
                }
                catch (tf::TransformException& ex)
                {
                    ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << "; quitting callback");
                    return;
                }
            }
            else
                return;
        }

        /* compute sensor origin in map frame */
        const tf::Vector3& sensor_origin_tf = map_H_sensor.getOrigin();
        Eigen::Vector3d sensor_origin_eigen(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());

        {
            SWRI_PROFILE("updateTransformCache");
            if (!updateTransformCache(cloud_msg->header.frame_id, cloud_msg->header.stamp)) {
                ROS_ERROR_THROTTLE(1, "Transform cache was not updated. Self-filtering may fail.");
                return;
            }
        }
        if (filter_pointcloud) {
            /* mask out points on the robot */
            {
                SWRI_PROFILE("shape mask update");
                shape_mask_->maskContainment(*cloud_msg, sensor_origin_eigen, 0.0, max_range_, mask_);
                updateMask(*cloud_msg, sensor_origin_eigen, mask_);
            }
            SWRI_PROFILE("2");
            sensor_msgs::PointCloud2::Ptr filtered_cloud, freespace_cloud;

            // We only use these iterators if we are creating a filtered_cloud for
            // publishing. We cannot default construct these, so we use unique_ptr's
            // to defer construction
            std::unique_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_x, iter_freespace_x;
            std::unique_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_y, iter_freespace_y;
            std::unique_ptr<sensor_msgs::PointCloud2Iterator<float> > iter_filtered_z, iter_freespace_z;


            filtered_cloud.reset(new sensor_msgs::PointCloud2());
            filtered_cloud->header = cloud_msg->header;
            sensor_msgs::PointCloud2Modifier pcd_modifier(*filtered_cloud);
            pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
            pcd_modifier.resize(cloud_msg->width * cloud_msg->height);

            // we have created a filtered_out, so we can create the iterators now
            iter_filtered_x.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "x"));
            iter_filtered_y.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "y"));
            iter_filtered_z.reset(new sensor_msgs::PointCloud2Iterator<float>(*filtered_cloud, "z"));


            freespace_cloud.reset(new sensor_msgs::PointCloud2());
            freespace_cloud->header = cloud_msg->header;
            sensor_msgs::PointCloud2Modifier pcd_modifier_freespace(*freespace_cloud);
            pcd_modifier_freespace.setPointCloud2FieldsByString(1, "xyz");
            pcd_modifier_freespace.resize(cloud_msg->width * cloud_msg->height);

            // we have created a filtered_out, so we can create the iterators now
            iter_freespace_x.reset(new sensor_msgs::PointCloud2Iterator<float>(*freespace_cloud, "x"));
            iter_freespace_y.reset(new sensor_msgs::PointCloud2Iterator<float>(*freespace_cloud, "y"));
            iter_freespace_z.reset(new sensor_msgs::PointCloud2Iterator<float>(*freespace_cloud, "z"));
            size_t filtered_cloud_size = 0, freespace_cloud_size = 0;


            {

                SWRI_PROFILE("split points into two pointclouds");
                /* do ray tracing to find which cells this point cloud indicates should be free, and which it indicates
                 * should be occupied */
                for (unsigned int row = 0; row < cloud_msg->height; row += point_subsample_) {
                    unsigned int row_c = row * cloud_msg->width;
                    sensor_msgs::PointCloud2ConstIterator<float> pt_iter(*cloud_msg, "x");
                    // set iterator to point at start of the current row
                    pt_iter += row_c;

                    for (unsigned int col = 0;
                         col < cloud_msg->width; col += point_subsample_, pt_iter += point_subsample_) {
                        // if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
                        //  continue;

                        /* check for NaN */
                        if (!std::isnan(pt_iter[0]) && !std::isnan(pt_iter[1]) && !std::isnan(pt_iter[2])) {

                            tf::Vector3 point_tf(pt_iter[0], pt_iter[1], pt_iter[2]);

                            /* occupied cell at ray endpoint if ray is shorter than max range and this point
                               isn't on a part of the robot*/
                            if (pt_iter[2] != 0.0 &&
                                (mask_[row_c + col] == point_containment_filter::ShapeMask::INSIDE ||
                                 mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)) {
                                **iter_freespace_x = pt_iter[0];
                                **iter_freespace_y = pt_iter[1];
                                **iter_freespace_z = pt_iter[2];
                                ++freespace_cloud_size;
                                ++*iter_freespace_x;
                                ++*iter_freespace_y;
                                ++*iter_freespace_z;
                            } //else if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP){}//clip_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
                            else {
                                **iter_filtered_x = pt_iter[0];
                                **iter_filtered_y = pt_iter[1];
                                **iter_filtered_z = pt_iter[2];
                                ++filtered_cloud_size;
                                ++*iter_filtered_x;
                                ++*iter_filtered_y;
                                ++*iter_filtered_z;
                            }
                        }
                    }
                }
            }
            {
                SWRI_PROFILE("3");
                sensor_msgs::PointCloud2Modifier pcd_modifier_(*filtered_cloud);
                pcd_modifier.resize(filtered_cloud_size);

                sensor_msgs::PointCloud2Modifier pcd_modifier_freespace_(*freespace_cloud);
                pcd_modifier_freespace_.resize(freespace_cloud_size);
            }
            {
                SWRI_PROFILE("4");
                tree_->lockWrite();
                try {
                    SWRI_PROFILE("insertPointcloud_and_FreespacePointcloud");
                    tree_->insertPointcloud(filtered_cloud);
                    if (use_freespace_pointcloud) {
                        tree_->insertFreespacePointcloud(freespace_cloud);
                    }
                } catch (...) {
                    tree_->unlockWrite();
                    return;
                }
                tree_->unlockWrite();
            }
            {
                SWRI_PROFILE("5");
                if (filtered_cloud_publisher_.getNumSubscribers() != 0)
                    filtered_cloud_publisher_.publish(filtered_cloud);
                if (freespace_cloud_publisher_.getNumSubscribers() != 0)
                    freespace_cloud_publisher_.publish(freespace_cloud);
            }
        } else {//no filtering
            tree_->lockWrite();
            try {
                SWRI_PROFILE("insertPointcloud_and_FreespacePointcloud");
                tree_->insertPointcloud(cloud_msg);
            } catch (...) {
                tree_->unlockWrite();
                return;
            }
            tree_->unlockWrite();

        }
        {
            SWRI_PROFILE("Update callback")
            tree_->triggerUpdateCallback();
        }


    }
}
