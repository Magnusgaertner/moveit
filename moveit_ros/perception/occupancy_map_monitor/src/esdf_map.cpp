


#include <moveit/occupancy_map_monitor/esdf_map.h>
#include <swri_profiler/profiler.h>

namespace occupancy_map_monitor {


    EsdfMap::EsdfMap(double resolution):voxblox::EsdfServer(ros::NodeHandle(), ros::NodeHandle("~voxblox")), DistanceField(0,0,0,0,0,0,0) { init(); }

    EsdfMap::EsdfMap(const std::string &filename):voxblox::EsdfServer(ros::NodeHandle(), ros::NodeHandle("~voxblox")), DistanceField(0,0,0,0,0,0,0) { init(); }

    void EsdfMap::init() {}

    bool EsdfMap::writeBinary(const std::string &filename)  {
      return saveMap(filename);
    }

    bool EsdfMap::readBinary(const std::string &filename)  {
      return loadMap(filename);
    }
    void EsdfMap::clear()  {
      voxblox::EsdfServer::clear();
    }

    std::string EsdfMap::name(){
      return "occupancy_map_monitor::EsdfMap";
    }


    void EsdfMap::getOctreeMessage(octomap_msgs::Octomap* msg)const {
        SWRI_PROFILE("EsdfMap/getOctreeMessage");
      voxblox::serializeLayerAsOctomapMsg(this->getTsdfMapPtr()->getTsdfLayer(),msg, 0.75);
    }


    double EsdfMap::getUninitializedDistance() const {
      return getEsdfMaxDistance();
    }

    double EsdfMap::getDistance(int x, int y, int z) const {
      ROS_ERROR("not supported");
      return 0;
      /*double distance;
      vxblx->getEsdfMapPtr()->getDistanceAtPosition(Eigen::Vector3d(x,y,z), &distance);
      return distance;*/
    }

    double EsdfMap::getDistance(double x, double y , double z) const {
      double distance = getEsdfMaxDistance();
      getEsdfMapPtr()->getDistanceAtPosition(Eigen::Vector3d(x,y,z), &distance);
      return distance;
    }

    int EsdfMap::getXNumCells() const {
      //TODO
      ROS_ERROR("not supported");
      return 0;
    }

    int EsdfMap::getYNumCells() const {
      //TODO
      ROS_ERROR("not supported");
      return 0;
    }

    int EsdfMap::getZNumCells() const {
      //TODO
      ROS_ERROR("not supported");
      return 0;
    }

    bool EsdfMap::worldToGrid(double world_x, double world_y, double world_z, int &x, int &y,
                                                        int &z) const {
      ROS_ERROR("not supported");
      x = world_x; y = world_y; z = world_z;
      return true;
    }


    double EsdfMap::getDistanceGradient(double x, double y, double z, double &gradient_x,
                                                                  double &gradient_y, double &gradient_z,
                                                                  bool &in_bounds) const {


      //ROS_INFO("%f, %f, %f distance : %f",2,0,1.5, getDistance(2.0, 0.0,1.5));

      //ROS_INFO("voxblox getDistanceGradient called");
      Eigen::Vector3d gradient(gradient_x, gradient_y, gradient_z);
      double distance = getEsdfMaxDistance();
      in_bounds = getEsdfMapPtr()->getDistanceAndGradientAtPosition(Eigen::Vector3d(x, y, z), &distance,&gradient);
      return distance;//DistanceField::getDistanceGradient(x, y, z, gradient_x, gradient_y, gradient_z, in_bounds);
    }



    void EsdfMap::addPointsToField(const EigenSTL::vector_Vector3d &points) {
      //TODO
      ROS_ERROR("not implemented");
      return;
    }

    void EsdfMap::removePointsFromField(const EigenSTL::vector_Vector3d &points) {
      //TODO
      ROS_ERROR("not implemented");
      return;
    }

    void EsdfMap::updatePointsInField(const EigenSTL::vector_Vector3d &old_points,
                                                                const EigenSTL::vector_Vector3d &new_points) {
      //TODO
      ROS_ERROR("not implemented");
      return;
    }

    void EsdfMap::reset() {
      clear();
    }

    bool EsdfMap::isCellValid(int x, int y, int z) const {
      getEsdfMapPtr()->isObserved(Eigen::Vector3d(x, y, z));
    }

    bool EsdfMap::gridToWorld(int x, int y, int z, double &world_x, double &world_y,
                                                        double &world_z) const {
      ROS_ERROR("not supported");
      world_x = x; world_y = y; world_z = z;
    }

    bool EsdfMap::writeToStream(std::ostream &stream) const {
      //vxblx->saveMap(stream)
      //TODO
      ROS_ERROR("not implemented");
      return false;
    }

    bool EsdfMap::readFromStream(std::istream &stream) {
      //return false;
      //TODO
      ROS_ERROR("not implemented");
      return false;
    }

  

  
}


