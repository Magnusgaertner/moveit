set(MOVEIT_LIB_NAME moveit_pointcloud_octomap_updater)

add_library(${MOVEIT_LIB_NAME}_core src/pointcloud_octomap_updater.cpp src/pointcloud_map_updater.cpp)
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME}_core moveit_point_containment_filter moveit_occupancy_map_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES LINK_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


set(MOVEIT_LIB_NAME_2 moveit_pointcloud_esdf_updater)

add_library(${MOVEIT_LIB_NAME_2}_core src/pointcloud_esdf_updater.cpp src/pointcloud_map_updater.cpp)
set_target_properties(${MOVEIT_LIB_NAME_2}_core PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME_2}_core moveit_point_containment_filter moveit_occupancy_map_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME_2}_core PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(${MOVEIT_LIB_NAME_2}_core PROPERTIES LINK_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

add_library(${MOVEIT_LIB_NAME_2} src/plugin_esdf_init.cpp)
set_target_properties(${MOVEIT_LIB_NAME_2} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME_2} ${MOVEIT_LIB_NAME_2}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME_2} ${MOVEIT_LIB_NAME_2}_core
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
