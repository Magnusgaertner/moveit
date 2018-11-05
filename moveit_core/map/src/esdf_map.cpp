


#include <moveit/map/esdf_map.h>
#include <swri_profiler/profiler.h>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <voxblox/utils/planning_utils.h>
#include <moveit/distance_field/propagation_distance_field.h>
namespace map {
    inline voxblox::EsdfMap::Config getEsdfMapConfig(double res, int block_size) {
        voxblox::EsdfMap::Config esdf_config;
        esdf_config.esdf_voxel_size = res;
        esdf_config.esdf_voxels_per_side = block_size;
        return esdf_config;
    }

    inline voxblox::TsdfMap::Config getTsdfMapConfig(double res, int block_size) {
        voxblox::TsdfMap::Config tsdf_config;
        tsdf_config.tsdf_voxel_size = res;
        tsdf_config.tsdf_voxels_per_side = block_size;
        return tsdf_config;
    }
    EsdfMap::EsdfMap(double resolution):
            voxblox::EsdfServer(ros::NodeHandle(),
                                ros::NodeHandle("~voxblox"),
                                getEsdfMapConfig(resolution, 16),
                                voxblox::EsdfIntegrator::Config{},
                                getTsdfMapConfig(resolution, 16),
                                voxblox::TsdfIntegratorBase::Config{}),
            DistanceField(0,0,0,0,0,0,0) {


        init();
    }


    EsdfMap::EsdfMap(const std::string &filename):voxblox::EsdfServer(ros::NodeHandle(), ros::NodeHandle("~voxblox")), DistanceField(0,0,0,0,0,0,0) { init(); }

    void EsdfMap::init() {
        ros::NodeHandle nh("~voxblox");
        nh.param("convert_to_octree", convert_to_octree, false);
    }

#ifdef NEW_MSG_FORMAT
    void EsdfMap::useDistanceFieldMessage(const voxblox_msgs::Layer& layer){
        esdfMapCallback(layer);
    }

    bool EsdfMap::getMapMsg(moveit_msgs::PlanningSceneWorld& world) const{
        voxblox::serializeLayerAsMsg<voxblox::EsdfVoxel>(this->esdf_map_->getEsdfLayer(),
                                       false, &world.distancefield);

        world.distancefield.action = static_cast<uint8_t>(voxblox::MapDerializationAction::kReset);
    }
#endif

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
      return "map::EsdfMap";
    }


    void EsdfMap::getOctreeMessage(octomap_msgs::Octomap* msg)const {

        if(convert_to_octree) {
            SWRI_PROFILE("EsdfMap/getOctreeMessage_convert_to_octree");
            voxblox::serializeLayerAsOctomapMsg(this->getTsdfMapPtr()->getTsdfLayer(), msg, 0.75);
        }
    }


    double EsdfMap::getUninitializedDistance() const {
      return getEsdfMaxDistance();
    }


    double EsdfMap::getDistance(double x, double y , double z) const {
      double distance = getEsdfMaxDistance();
      getEsdfMapPtr()->getDistanceAtPosition(Eigen::Vector3d(x,y,z), &distance);
      return distance;
    }


    double EsdfMap::getDistanceGradient(double x, double y, double z, double &gradient_x,
                                                                  double &gradient_y, double &gradient_z,
                                                                  bool &in_bounds) const {
      Eigen::Vector3d gradient(gradient_x, gradient_y, gradient_z);
      double distance = getEsdfMaxDistance();
      in_bounds = getEsdfMapPtr()->getDistanceAndGradientAtPosition(Eigen::Vector3d(x, y, z), &distance,&gradient);
      return distance;//DistanceField::getDistanceGradient(x, y, z, gradient_x, gradient_y, gradient_z, in_bounds);
    }

    bool EsdfMap::worldToGrid(double world_x, double world_y, double world_z, int &x, int &y,
                     int &z) const {
        double scale = 1/getEsdfMapPtr()->voxel_size();
        x*=scale;
        y*=scale;
        z*=scale;
        return true;
    }



    void EsdfMap::addPointsToField(const EigenSTL::vector_Vector3d &points) {
        double max_distance = this->getEsdfMaxDistance();
        voxblox::Layer<voxblox::EsdfVoxel>* layer = this->getEsdfMapPtr()->getEsdfLayerPtr();
        for(auto& point: points)voxblox::utils::fillSphereAroundPoint(point.cast<float>(),0.0, max_distance, layer);
    }

    void EsdfMap::removePointsFromField(const EigenSTL::vector_Vector3d &points) {
        double max_distance = this->getEsdfMaxDistance();
        voxblox::Layer<voxblox::EsdfVoxel>* layer = this->getEsdfMapPtr()->getEsdfLayerPtr();
        for(auto& point: points)voxblox::utils::clearSphereAroundPoint(point.cast<float>(),0.0, max_distance, layer);
    }

    void EsdfMap::updatePointsInField(const EigenSTL::vector_Vector3d &old_points,
                                                                const EigenSTL::vector_Vector3d &new_points) {
        /*VoxelSet old_point_set;
        for (unsigned int i = 0; i < old_points.size(); i++)
        {
            Eigen::Vector3i voxel_loc;
            bool valid = worldToGrid(old_points[i].x(), old_points[i].y(), old_points[i].z(), voxel_loc.x(), voxel_loc.y(),
                                     voxel_loc.z());
            if (valid)
            {
                old_point_set.insert(voxel_loc);
            }
        }

        VoxelSet new_point_set;
        for (unsigned int i = 0; i < new_points.size(); i++)
        {
            Eigen::Vector3i voxel_loc;
            bool valid = worldToGrid(new_points[i].x(), new_points[i].y(), new_points[i].z(), voxel_loc.x(), voxel_loc.y(),
                                     voxel_loc.z());
            if (valid)
            {
                new_point_set.insert(voxel_loc);
            }
        }
        distance_field::compareEigen_Vector3i comp;

        EigenSTL::vector_Vector3i old_not_new;
        std::set_difference(old_point_set.begin(), old_point_set.end(), new_point_set.begin(), new_point_set.end(),
                            std::inserter(old_not_new, old_not_new.end()), comp);

        EigenSTL::vector_Vector3i new_not_old;
        std::set_difference(new_point_set.begin(), new_point_set.end(), old_point_set.begin(), old_point_set.end(),
                            std::inserter(new_not_old, new_not_old.end()), comp);


        removePointsFromField(old_not_new);
        addPointsToField(new_not_in_current);*/
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


