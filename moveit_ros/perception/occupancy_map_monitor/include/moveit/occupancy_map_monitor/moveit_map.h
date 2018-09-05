//
// Created by magnus on 31.08.18.
//

#ifndef MOVEIT_INDUSTRIAL_TOPLEVEL_MOVEITMAP_H
#define MOVEIT_INDUSTRIAL_TOPLEVEL_MOVEITMAP_H

#include <octomap/octomap.h>

namespace occupancy_map_monitor {


  class MoveitMap {
  public:
    virtual bool writeBinary(const std::string& filename) = 0;

    virtual bool readBinary(const std::string& filename) = 0;

    virtual void clear() = 0;

    /** @brief lock the underlying map. it will not be read or written by the
     *  monitor until unlockTree() is called */
    void lockRead() { map_mutex_.lock_shared(); }

    /** @brief unlock the underlying map. */
    void unlockRead() { map_mutex_.unlock_shared(); }

    /** @brief lock the underlying map. it will not be read or written by the
     *  monitor until unlockTree() is called */
    void lockWrite() { map_mutex_.lock(); }

    /** @brief unlock the underlying octree. */
    void unlockWrite() { map_mutex_.unlock(); }

    typedef boost::shared_lock<boost::shared_mutex> ReadLock;
    typedef boost::unique_lock<boost::shared_mutex> WriteLock;

    ReadLock reading() { return ReadLock(map_mutex_); }

    WriteLock writing() { return WriteLock(map_mutex_); }

    void triggerUpdateCallback(void)
    {
      if (update_callback_)
        update_callback_();
    }

    /** @brief Set the callback to trigger when updates are received */
    void setUpdateCallback(const boost::function<void()>& update_callback) { update_callback_ = update_callback; }

  private:
    boost::shared_mutex map_mutex_;
    boost::function<void()> update_callback_;
  };

  typedef std::shared_ptr<MoveitMap> MoveitMapPtr;
  typedef std::shared_ptr<const MoveitMap> MoveitMapConstPtr;
}


#endif //MOVEIT_INDUSTRIAL_TOPLEVEL_MOVEITMAP_H
