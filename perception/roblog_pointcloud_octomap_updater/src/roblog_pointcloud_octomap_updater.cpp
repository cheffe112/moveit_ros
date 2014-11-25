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

#include <cmath>
#include <moveit/roblog_pointcloud_octomap_updater/roblog_pointcloud_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <message_filters/subscriber.h>
#include <pcl_conversions/pcl_conversions.h>
#include <XmlRpcException.h>

namespace occupancy_map_monitor
{

RoblogPointCloudOctomapUpdater::RoblogPointCloudOctomapUpdater() : OccupancyMapUpdater("RoblogPointCloudUpdater"),
                                                       private_nh_("~"),
                                                       scale_(1.0),
                                                       padding_(0.0),
                                                       max_range_(std::numeric_limits<double>::infinity()),
                                                       point_subsample_(1),
                                                       point_cloud_subscriber_(NULL),
                                                       point_cloud_filter_(NULL)
{
}

RoblogPointCloudOctomapUpdater::~RoblogPointCloudOctomapUpdater()
{
  stopHelper();
}

bool RoblogPointCloudOctomapUpdater::setParams(XmlRpc::XmlRpcValue &params)
{
  try
  {
    if (!params.hasMember("point_cloud_topic"))
      return false;
    point_cloud_topic_ = static_cast<const std::string&>(params["point_cloud_topic"]);

    readXmlParam(params, "max_range", &max_range_);
    readXmlParam(params, "padding_offset", &padding_);
    readXmlParam(params, "padding_scale", &scale_);
    readXmlParam(params, "point_subsample", &point_subsample_);
    if (params.hasMember("update_collision_objects_service_name"))
      update_collision_objects_service_name_ = static_cast<const std::string&>(params["update_collision_objects_service_name"]);
    if (params.hasMember("mask_collision_objects_service_name"))
      mask_collision_objects_service_name_ = static_cast<const std::string&>(params["mask_collision_objects_service_name"]);
    if (params.hasMember("filtered_cloud_topic"))
      filtered_cloud_topic_ = static_cast<const std::string&>(params["filtered_cloud_topic"]);
  }
  catch (XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
    return false;
  }

  return true;
}

bool RoblogPointCloudOctomapUpdater::initialize()
{
  tf_ = monitor_->getTFClient();
  tf_listener_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener(ros::Duration(1000)));
  shape_mask_.reset(new point_containment_filter::ShapeMask());
  shape_mask_->setTransformCallback(boost::bind(&RoblogPointCloudOctomapUpdater::getShapeTransform, this, _1, _2));
  if (!filtered_cloud_topic_.empty())
    filtered_cloud_publisher_ = private_nh_.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic_, 10, false);
    
  updateCollisionObjectsServer = private_nh_.advertiseService(update_collision_objects_service_name_, &occupancy_map_monitor::RoblogPointCloudOctomapUpdater::updateCollisionObjects, this);
  maskCollisionObjectsServer = private_nh_.advertiseService(mask_collision_objects_service_name_, &occupancy_map_monitor::RoblogPointCloudOctomapUpdater::maskCollisionObjects, this);

  return true;
}

void RoblogPointCloudOctomapUpdater::start()
{
  if (point_cloud_subscriber_)
    return;
  /* subscribe to point cloud topic using tf filter*/
  point_cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(root_nh_, point_cloud_topic_, 5);
  if (tf_ && !monitor_->getMapFrame().empty())
  {
    point_cloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(*point_cloud_subscriber_, *tf_, monitor_->getMapFrame(), 5);
    point_cloud_filter_->registerCallback(boost::bind(&RoblogPointCloudOctomapUpdater::cloudMsgCallback, this, _1));
    ROS_INFO("Listening to '%s' using message filter with target frame '%s'", point_cloud_topic_.c_str(), point_cloud_filter_->getTargetFramesString().c_str());
  }
  else
  {
    point_cloud_subscriber_->registerCallback(boost::bind(&RoblogPointCloudOctomapUpdater::cloudMsgCallback, this, _1));
    ROS_INFO("Listening to '%s'", point_cloud_topic_.c_str());
  }
}

void RoblogPointCloudOctomapUpdater::stopHelper()
{
  delete point_cloud_filter_;
  delete point_cloud_subscriber_;
}

void RoblogPointCloudOctomapUpdater::stop()
{
  stopHelper();
  point_cloud_filter_ = NULL;
  point_cloud_subscriber_ = NULL;
}

ShapeHandle RoblogPointCloudOctomapUpdater::excludeShape(const shapes::ShapeConstPtr &shape)
{
  ShapeHandle h = 0;
  if (shape_mask_)
    h = shape_mask_->addShape(shape, scale_, padding_);
  else
    ROS_ERROR("Shape filter not yet initialized!");
  return h;
}

void RoblogPointCloudOctomapUpdater::forgetShape(ShapeHandle handle)
{
  if (shape_mask_)
    shape_mask_->removeShape(handle);
}

bool RoblogPointCloudOctomapUpdater::getShapeTransform(ShapeHandle h, Eigen::Affine3d &transform) const
{
  ShapeTransformCache::const_iterator it = transform_cache_.find(h);
  if (it == transform_cache_.end())
  {
    ROS_ERROR("Internal error. Shape filter handle %u not found", h);
    return false;
  }
  transform = it->second;
  return true;
}

void RoblogPointCloudOctomapUpdater::updateMask(const pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Vector3d &sensor_origin, std::vector<int> &mask)
{
}

bool RoblogPointCloudOctomapUpdater::transformPointCloud(const std::string &fromFrame, const std::string &toFrame, const pcl::PointCloud<pcl::PointXYZ> &src_point_cloud, pcl::PointCloud<pcl::PointXYZ> &target_point_cloud, ros::Duration duration)
{
    bool success_tf = false;

    sensor_msgs::PointCloud2 src_point_cloud_msg;
    sensor_msgs::PointCloud pointCloudMsgConvert;
    sensor_msgs::PointCloud pointCloudMsgTransformed;

    pcl::toROSMsg(src_point_cloud,src_point_cloud_msg);
    sensor_msgs::convertPointCloud2ToPointCloud(src_point_cloud_msg, pointCloudMsgConvert);

    try
    {
        tf_listener_->waitForTransform(toFrame, fromFrame, src_point_cloud_msg.header.stamp, duration);
        tf_listener_->transformPointCloud(std::string(toFrame), pointCloudMsgConvert, pointCloudMsgTransformed);
        success_tf = true;
    } catch (tf::TransformException &ex)
    {
        ROS_ERROR_STREAM("RoblogPointCloudOctomapUpdater::transformPointCloud ... transformation failed with error "<<ex.what()<<", timestamp was "<<src_point_cloud_msg.header.stamp);
        success_tf = false;
        return success_tf;
    }

    sensor_msgs::PointCloud2 transformedPointCloud;
    sensor_msgs::convertPointCloudToPointCloud2(pointCloudMsgTransformed, transformedPointCloud);
    pcl::fromROSMsg(transformedPointCloud,target_point_cloud);

    return success_tf;
}

bool RoblogPointCloudOctomapUpdater::updateCollisionObjects(moveit_ros_perception::UpdateCollisionObjects::Request &req, moveit_ros_perception::UpdateCollisionObjects::Response &res)
{
    ROS_INFO_STREAM("RoblogPointCloudOctomapUpdater::updateCollisionObjects with " << req.collision_objects.size() << " objects.");
    collisionObjects.clear();
    collisionObjectsClouds.clear();
    maskCollisionObject.clear();
    for(std::vector<moveit_msgs::CollisionObject>::iterator it = req.collision_objects.begin(); it != req.collision_objects.end(); ++it){
        // save collision object
        collisionObjects.push_back(*it);
        // generate point cloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for(unsigned int i = 0; i < it->primitives.size(); ++i){
            if(it->primitives[i].type == shape_msgs::SolidPrimitive::BOX){
                // create box
                pcl::PointCloud<pcl::PointXYZ> box = *generateBox(
                    it->primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_X],
                    it->primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Y],
                    it->primitives[i].dimensions[shape_msgs::SolidPrimitive::BOX_Z],
                    0.03);
                // rotate/translate
                Eigen::Quaternionf rotation(it->primitive_poses[i].orientation.w,it->primitive_poses[i].orientation.x,it->primitive_poses[i].orientation.y,it->primitive_poses[i].orientation.z);
                Eigen::Translation3f translation(it->primitive_poses[i].position.x,it->primitive_poses[i].position.y,it->primitive_poses[i].position.z);
                Eigen::Affine3f t;
                t = translation * rotation;
                pcl::transformPointCloud(box, box, t);
                cloud += box;
            } else if(it->primitives[i].type == shape_msgs::SolidPrimitive::CYLINDER){
                // create cylinder
                pcl::PointCloud<pcl::PointXYZ> cylinder = *generateCylinder(
                    it->primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT],
                    it->primitives[i].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS],
                    0.03);
                // rotate/translate
                Eigen::Quaternionf rotation(it->primitive_poses[i].orientation.w,it->primitive_poses[i].orientation.x,it->primitive_poses[i].orientation.y,it->primitive_poses[i].orientation.z);
                Eigen::Translation3f translation(it->primitive_poses[i].position.x,it->primitive_poses[i].position.y,it->primitive_poses[i].position.z);
                Eigen::Affine3f t;
                t = translation * rotation;
                pcl::transformPointCloud(cylinder, cylinder, t);
                // add cylinder shape to cloud
                cloud += cylinder;
            } else {
                ROS_WARN_STREAM("Encountered unknown collision object shape type " << it->primitives[i].type << "!");
            }
        }
        ROS_INFO_STREAM("Transforming point cloud from "<<it->header.frame_id<<" to "<<sensorFrameId);
        cloud.header = pcl_conversions::toPCL(it->header);
        if(!transformPointCloud(cloud.header.frame_id, sensorFrameId, cloud, cloud))
        {
            return false;
        }
        collisionObjectsClouds.push_back(cloud);
    }
    maskCollisionObject.assign(collisionObjects.size(), false);
    
    return true;
}

bool RoblogPointCloudOctomapUpdater::maskCollisionObjects(moveit_ros_perception::MaskCollisionObjects::Request &req, moveit_ros_perception::MaskCollisionObjects::Response &res)
{
    for(unsigned int i = 0; i < collisionObjects.size(); ++i)
    {
        std::vector<unsigned int>::iterator itIDs;
        itIDs = std::find(req.tracked_instance_ids.begin(), req.tracked_instance_ids.end(), boost::lexical_cast<int>(collisionObjects[i].id));
        if (itIDs != req.tracked_instance_ids.end())
        {
            maskCollisionObject[i] = req.do_masking;
        }
    }
    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RoblogPointCloudOctomapUpdater::generateBox(double lengthX, double lengthY, double lengthZ, double resolution){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cube(new pcl::PointCloud<pcl::PointXYZ>);

/*
    for(double x = -lengthX/2; x < lengthX/2 + 1e-8; x += resolution){
        for(double y = -lengthY/2/2; y < lengthY/2/2 + 1e-8; y += resolution){
            for(double z = -lengthZ/2/2; z < lengthZ/2/2 + 1e-8; z += resolution){
            pcl::PointXYZ p;
            p.x = x;
            p.y = y;
            p.z = z;
            cube->push_back(p);
            }
        }
    }
 */   
    
    for(double x = -lengthX/2; x < lengthX/2 + 1e-8; x +=resolution){
        for(double y = -lengthY/2; y < lengthY/2+ 1e-8; y+=resolution){
            pcl::PointXYZ p;
            p.x = x;
            p.y = y;
            p.z = -lengthZ/2;
            cube->push_back(p);
        }
    }
    for(double x = -lengthX/2; x < lengthX/2 + 1e-8; x +=resolution){
        for(double y = -lengthY/2; y < lengthY/2+ 1e-8; y+=resolution){
            pcl::PointXYZ p;
            p.x = x;
            p.y = y;
            p.z = lengthZ/2;
            cube->push_back(p);
        }
    }
    for(double x = -lengthX/2; x < lengthX/2 + 1e-8; x +=resolution){
        for(double z = -lengthZ/2; z < lengthZ/2+ 1e-8; z+=resolution){
            pcl::PointXYZ p;
            p.x = x;
            p.y = -lengthY/2;
            p.z = z;
            cube->push_back(p);
        }
    }
    for(double x = -lengthX/2; x < lengthX/2 + 1e-8; x +=resolution){
        for(double z = -lengthZ/2; z < lengthZ/2+ 1e-8; z+=resolution){
            pcl::PointXYZ p;
            p.x = x;
            p.y = lengthY/2;
            p.z = z;
            cube->push_back(p);
        }
    }
    for(double y = -lengthY/2; y < lengthY/2+ 1e-8; y+=resolution){
        for(double z = -lengthZ/2; z < lengthZ/2+ 1e-8; z+=resolution){
            pcl::PointXYZ p;
            p.x = -lengthX/2;
            p.y = y;
            p.z = z;
            cube->push_back(p);
        }
    }
    for(double y = -lengthY/2; y < lengthY/2+ 1e-8; y+=resolution){
        for(double z = -lengthZ/2; z < lengthZ/2+ 1e-8; z+=resolution){
            pcl::PointXYZ p;
            p.x = lengthX/2;
            p.y = y;
            p.z = z;
            cube->push_back(p);
        }
    }

    return cube;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RoblogPointCloudOctomapUpdater::generateCylinder(double length, double radius, double resolution){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder(new pcl::PointCloud<pcl::PointXYZ>);

    double angular_resolution_rad = 0.1;

    for(double z = -length/2; z < length/2; z += resolution){
        for (double angle = 0.0; angle < 2*M_PI; angle += angular_resolution_rad) {
            pcl::PointXYZ p;
            p.x = cos(angle)*radius;
            p.y = sin(angle)*radius;
            p.z = z;
            cylinder->push_back(p);
        }
    }

    double zz[] = {-length/2, length/2};
    for (int i = 0; i < 2; ++i) {
        double z = zz[i];
        for (double r = radius - resolution; r > 0; r -= resolution) {
            for (double angle = 0.0; angle < 2*M_PI; angle += angular_resolution_rad) {
                pcl::PointXYZ p;
                p.x = cos(angle)*r;
                p.y = sin(angle)*r;
                p.z = z;
                cylinder->push_back(p);
            }
        }
    }


    return cylinder;
}

void RoblogPointCloudOctomapUpdater::cloudMsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
  ROS_DEBUG("Received a new point cloud message");
  ros::WallTime start = ros::WallTime::now();

  if (monitor_->getMapFrame().empty())
    monitor_->setMapFrame(cloud_msg->header.frame_id);

  /* get transform for cloud into map frame */
  tf::StampedTransform map_H_sensor;
  if (monitor_->getMapFrame() == cloud_msg->header.frame_id)
    map_H_sensor.setIdentity();
  else
  {
    sensorFrameId = cloud_msg->header.frame_id;
    if (tf_)
    {
      try
      {
        tf_->lookupTransform(monitor_->getMapFrame(), cloud_msg->header.frame_id, cloud_msg->header.stamp, map_H_sensor);
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

  /* convert cloud message to pcl cloud object */
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);

  /* compute sensor origin in map frame */
  const tf::Vector3 &sensor_origin_tf = map_H_sensor.getOrigin();
  octomap::point3d sensor_origin(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());
  Eigen::Vector3d sensor_origin_eigen(sensor_origin_tf.getX(), sensor_origin_tf.getY(), sensor_origin_tf.getZ());

  if (!updateTransformCache(cloud_msg->header.frame_id, cloud_msg->header.stamp))
  {
    ROS_ERROR_THROTTLE(1, "Transform cache was not updated. Self-filtering may fail.");
    return;
  }

  /* mask out points on the robot */
  shape_mask_->maskContainment(cloud, sensor_origin_eigen, 0.0, max_range_, mask_);
  
  // mask out respective collision objects
/*  for(unsigned int i = 0; i < maskCollisionObject.size(); ++i)
  {
    if(maskCollisionObject[i]){
        std::vector<int> maskRemove;
        shape_mask_->maskContainment(collisionObjectsClouds[i], sensor_origin_eigen, 0.0, max_range_, maskRemove);
        mask_.insert(mask_.end(), maskRemove.begin(), maskRemove.end());
    }
  }
  */
  updateMask(cloud, sensor_origin_eigen, mask_);
  
  octomap::KeySet free_cells, occupied_cells, model_cells, clip_cells, solid_cells;
  boost::scoped_ptr<pcl::PointCloud<pcl::PointXYZ> > filtered_cloud;
  if (!filtered_cloud_topic_.empty())
    filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

  tree_->lockRead();

  try
  {
      
    // do ray tracing to find which cells this point cloud indicates should be free, and which it indicates
    // should be occupied
    for (unsigned int row = 0; row < cloud.height; row += point_subsample_)
    {
      unsigned int row_c = row * cloud.width;
      for (unsigned int col = 0; col < cloud.width; col += point_subsample_)
      {
        //if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
        //  continue;
        const pcl::PointXYZ &p = cloud(col, row);

        // check for NaN
        if (!isnan(p.x) && !isnan(p.y) && !isnan(p.z))
    {
      // transform to map frame
      tf::Vector3 point_tf = map_H_sensor * tf::Vector3(p.x, p.y, p.z);

      // occupied cell at ray endpoint if ray is shorter than max range and this point
      //   isn't on a part of the robot
      if (mask_[row_c + col] == point_containment_filter::ShapeMask::INSIDE)
        model_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
      else if (mask_[row_c + col] == point_containment_filter::ShapeMask::CLIP)
        clip_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
      else
          {
            occupied_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
           // if (filtered_cloud)
              filtered_cloud->push_back(p);
          }
        }
      }
    }
    //ROS_INFO_STREAM("filtered_cloud->size = "<<filtered_cloud->points.size());

    // add voxels for all existing collision objects    
    //for(std::vector<pcl::PointCloud<pcl::PointXYZ> >::iterator cloudIt = collisionObjectsClouds.begin(); cloudIt != collisionObjectsClouds.end(); ++cloudIt)
    for(unsigned int i = 0; i < collisionObjectsClouds.size(); ++i)
    {
        //if(!maskCollisionObject[i]){
            for(pcl::PointCloud<pcl::PointXYZ>::iterator pointIt = collisionObjectsClouds[i].begin(); pointIt != collisionObjectsClouds[i].end(); ++pointIt)
            {
                tf::Vector3 point_tf = map_H_sensor * tf::Vector3(pointIt->x, pointIt->y, pointIt->z);
                solid_cells.insert(tree_->coordToKey(point_tf.getX(), point_tf.getY(), point_tf.getZ()));
                //occupied_cells.insert(tree_->coordToKey(pointIt->x, pointIt->y, pointIt->z));
               // if (filtered_cloud)
                  filtered_cloud->push_back(*pointIt);
            }
        //}
    }
    //ROS_INFO_STREAM("filtered_cloud->size = "<<filtered_cloud->points.size());
  
    // compute the free cells along each ray that ends at an occupied cell
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
      if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
        free_cells.insert(key_ray_.begin(), key_ray_.end());

    // compute the free cells along each ray that ends at a model cell
    for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
      if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
        free_cells.insert(key_ray_.begin(), key_ray_.end());

    // compute the free cells along each ray that ends at a clipped cell
    for (octomap::KeySet::iterator it = clip_cells.begin(), end = clip_cells.end(); it != end; ++it)
      if (tree_->computeRayKeys(sensor_origin, tree_->keyToCoord(*it), key_ray_))
        free_cells.insert(key_ray_.begin(), key_ray_.end());
        
  }
  catch (...)
  {
    tree_->unlockRead();
    return;
  }

  tree_->unlockRead();

  // cells that overlap with the model are not occupied
  for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
    occupied_cells.erase(*it);

  // occupied cells are not free
  for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
    free_cells.erase(*it);

  // solid cells are not free either
  for (octomap::KeySet::iterator it = solid_cells.begin(), end = solid_cells.end(); it != end; ++it)
    free_cells.erase(*it);

  tree_->lockWrite();

  try
  {
    // mark free cells only if not seen occupied in this cloud
    for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
      tree_->updateNode(*it, false);

    // now mark all occupied cells
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it)
      tree_->updateNode(*it, true);

    // mark the solidification cells as occupied
    for (octomap::KeySet::iterator it = solid_cells.begin(), end = solid_cells.end(); it != end; ++it)
      tree_->updateNode(*it, true);

    // set the logodds to the minimum for the cells that are part of the model
    const float lg = tree_->getClampingThresMinLog() - tree_->getClampingThresMaxLog();
    for (octomap::KeySet::iterator it = model_cells.begin(), end = model_cells.end(); it != end; ++it)
      tree_->updateNode(*it, lg);
  }
  catch (...)
  {
    ROS_ERROR("Internal error while updating octree");
  }
  tree_->unlockWrite();
  ROS_DEBUG("Processed point cloud in %lf ms", (ros::WallTime::now() - start).toSec() * 1000.0);
  tree_->triggerUpdateCallback();

  if (filtered_cloud)
  {
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = cloud_msg->header;
    filtered_cloud_publisher_.publish(filtered_cloud_msg);
  }
}

}
