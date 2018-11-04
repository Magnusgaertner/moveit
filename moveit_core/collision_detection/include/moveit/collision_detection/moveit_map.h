#ifndef MOVEIT_INDUSTRIAL_TOPLEVEL_MOVEIT_MAP_H
#define MOVEIT_INDUSTRIAL_TOPLEVEL_MOVEIT_MAP_H


//dont know how to include boost::function<void()> otherwise
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/function.hpp>
#include <octomap_msgs/Octomap.h>
namespace collision_detection {
  class MoveitMap {
  public:
    virtual ~MoveitMap() = default;
    virtual bool writeBinary(const std::string& filename) = 0;

    virtual bool readBinary(const std::string& filename) = 0;

    virtual void clear() = 0;

    /** @brief lock the underlying map. it will not be read or written by the
     *  monitor until unlockTree() is called */
    void lockRead();

    /** @brief unlock the underlying map. */
    void unlockRead();

    /** @brief lock the underlying map. it will not be read or written by the
     *  monitor until unlockTree() is called */
    void lockWrite();

    /** @brief unlock the underlying octree. */
    void unlockWrite();

    typedef boost::shared_lock<boost::shared_mutex> ReadLock;
    typedef boost::unique_lock<boost::shared_mutex> WriteLock;

    ReadLock reading();

    WriteLock writing();

    /**
     * @details default callbacks are planning_scene_monitor::esdf/octomapUpdateCallback
     */
    void triggerUpdateCallback(void);

    /** @brief Set the callback to trigger when updates are received */
    void setUpdateCallback(const boost::function<void()>& update_callback);

    //virtual void updateScene(const std::shared_ptr<const MoveitMap>& me, planning_scene::PlanningScenePtr &scene, const Eigen::Affine3d& t) = 0;

    virtual void getOctreeMessage(octomap_msgs::Octomap* msg) const = 0;
  private:
    boost::shared_mutex map_mutex_;
    boost::function<void()> update_callback_;
  };

  typedef std::shared_ptr<MoveitMap> MoveitMapPtr;
  typedef std::shared_ptr<const MoveitMap> MoveitMapConstPtr;
}


#endif //MOVEIT_INDUSTRIAL_TOPLEVEL_MOVEITMAP_H
