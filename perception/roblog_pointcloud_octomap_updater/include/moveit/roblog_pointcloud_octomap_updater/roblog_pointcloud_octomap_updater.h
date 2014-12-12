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

#ifndef ROBLOG_MOVEIT_PERCEPTION_POINTCLOUD_OCTOMAP_UPDATER_
#define ROBLOG_MOVEIT_PERCEPTION_POINTCLOUD_OCTOMAP_UPDATER_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <moveit/point_containment_filter/shape_mask.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <moveit_ros_perception/UpdateCollisionObjects.h>
#include <moveit_ros_perception/MaskCollisionObjects.h>
#include <moveit_ros_perception/UpdateOctomap.h>
#include <moveit_ros_perception/IsAppliedUpdateOctomap.h>

namespace occupancy_map_monitor
{

class RoblogPointCloudOctomapUpdater : public OccupancyMapUpdater
{
public:

  RoblogPointCloudOctomapUpdater();
  virtual ~RoblogPointCloudOctomapUpdater();

  virtual bool setParams(XmlRpc::XmlRpcValue &params);

  virtual bool initialize();
  virtual void start();
  virtual void stop();
  virtual ShapeHandle excludeShape(const shapes::ShapeConstPtr &shape);
  virtual void forgetShape(ShapeHandle handle);

protected:

  virtual void updateMask(const pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Vector3d &sensor_origin, std::vector<int> &mask);

private:

  bool transformPointCloud(const std::string &fromFrame, const std::string &toFrame, const pcl::PointCloud<pcl::PointXYZ> &src_point_cloud, pcl::PointCloud<pcl::PointXYZ> &target_point_cloud, ros::Duration duration = ros::Duration(5.0));
  
  bool updateCollisionObjectsService(moveit_ros_perception::UpdateCollisionObjects::Request &req, moveit_ros_perception::UpdateCollisionObjects::Response &res);
  bool updateCollisionObjects(moveit_ros_perception::UpdateCollisionObjects::Request &req, moveit_ros_perception::UpdateCollisionObjects::Response &res, double shape_model_scale, std::vector<pcl::PointCloud<pcl::PointXYZ> > &collisionObjectsClouds);
  
  bool updateOctomap(moveit_ros_perception::UpdateOctomap::Request &req, moveit_ros_perception::UpdateOctomap::Response &res);
  bool maskCollisionObjects(moveit_ros_perception::MaskCollisionObjects::Request &req, moveit_ros_perception::MaskCollisionObjects::Response &res);
  //bool isUpdateApplied(moveit_ros_perception::IsAppliedUpdateOctomap::Request &req, moveit_ros_perception::IsAppliedUpdateOctomap::Response &res);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr generateBox(double lengthX, double lengthY, double lengthZ, double resolution);
  pcl::PointCloud<pcl::PointXYZ>::Ptr generateCylinder(double length, double radius, double resolution);
  bool getShapeTransform(ShapeHandle h, Eigen::Affine3d &transform) const;
  void cloudMsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);
  void stopHelper();

  ros::NodeHandle root_nh_;
  ros::NodeHandle private_nh_;
  boost::shared_ptr<tf::Transformer> tf_;
  boost::shared_ptr<tf::TransformListener> tf_listener_;
  
 ros::ServiceServer updateCollisionObjectsServer, maskCollisionObjectsServer, updateOctomapServer;
 //ros::ServiceServer isAppliedUpdateServer;

  /* params */
  std::string point_cloud_topic_;
  double scale_;
  double padding_;
  double max_range_;
  double shape_model_scale_;
  unsigned int point_subsample_;
  
  
  std::string update_collision_objects_service_name_;
  std::string mask_collision_objects_service_name_;
  std::string update_octomap_service_name_;
  std::string is_applied_update_service_name_;
  std::string filtered_cloud_topic_;
  ros::Publisher filtered_cloud_publisher_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> *point_cloud_subscriber_;
  tf::MessageFilter<sensor_msgs::PointCloud2> *point_cloud_filter_;

  /* used to store all cells in the map which a given ray passes through during raycasting.
     we cache this here because it dynamically pre-allocates a lot of memory in its contsructor */
  octomap::KeyRay key_ray_;

  boost::scoped_ptr<point_containment_filter::ShapeMask> shape_mask_;
  std::vector<int> mask_;
  
  std::string sensorFrameId;
  std::vector<moveit_msgs::CollisionObject> collisionObjects;
  std::vector<pcl::PointCloud<pcl::PointXYZ> > collisionObjectsClouds;
  std::vector<pcl::PointCloud<pcl::PointXYZ> > collisionObjectsCloudsScaled;
  std::vector<bool> maskCollisionObject;
  

  //We need this since we don't know when we will receive an octomap update callback. Hence we need to have track on state-changes due to service calls so that we can check whether they have been applied to the octomap by the callback 
  //int update_request_counter;
  //boost::mutex is_update_applied_mtx_;
  //bool requireUpdate(bool do_add = true);
  //bool existUpdateRequest();
};

}

#endif
