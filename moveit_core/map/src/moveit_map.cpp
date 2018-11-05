//
// Created by magnus on 07.09.18.
//

#include <moveit/map/moveit_map.h>

namespace map{

  void MoveitMap::lockRead() { map_mutex_.lock_shared(); }


  void MoveitMap::unlockRead() { map_mutex_.unlock_shared(); }

  void MoveitMap::lockWrite() { map_mutex_.lock(); }

  void MoveitMap::unlockWrite() { map_mutex_.unlock(); }

  MoveitMap::ReadLock MoveitMap::reading() { return MoveitMap::ReadLock(map_mutex_); }

  MoveitMap::WriteLock MoveitMap::writing() { return MoveitMap::WriteLock(map_mutex_); }

  void MoveitMap::triggerUpdateCallback(void)
  {
    if (update_callback_)
      update_callback_(); ///default callbacks are planning_scene_monitor::esdf/octomapUpdateCallback
  }

  /** @brief Set the callback to trigger when updates are received */
  void MoveitMap::setUpdateCallback(const boost::function<void()>& update_callback) { update_callback_ = update_callback; }


}