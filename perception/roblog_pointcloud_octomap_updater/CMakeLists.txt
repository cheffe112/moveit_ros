set(MOVEIT_LIB_NAME moveit_roblog_pointcloud_octomap_updater)

add_library(${MOVEIT_LIB_NAME}_core src/roblog_pointcloud_octomap_updater.cpp)
target_link_libraries(${MOVEIT_LIB_NAME}_core moveit_point_containment_filter moveit_occupancy_map_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES LINK_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(${MOVEIT_LIB_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
