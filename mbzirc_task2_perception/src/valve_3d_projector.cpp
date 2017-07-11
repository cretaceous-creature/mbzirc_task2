// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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


#include <image_geometry/pinhole_camera_model.h>
#include "mbzirc_task2_perception/valve_3d_projector.h"
#include <jsk_recognition_utils/cv_utils.h>
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

namespace mbzirc_task2_perception
{
  void Valve3DProjector::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, true);
    pub_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "debug_output", 1);
    onInitPostProcess();
  }

  void Valve3DProjector::subscribe()
  {
    sub_pose_.subscribe(*pnh_, "input/pose", 1); // debug output of roi_generator
    sub_point_.subscribe(*pnh_, "input/point", 1);
    sub_info_.subscribe(*pnh_, "input/info", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      async_->connectInput(sub_pose_, sub_point_, sub_info_);
      async_->registerCallback(boost::bind(&Valve3DProjector::apply, this, _1, _2, _3));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_pose_, sub_point_, sub_info_);
      sync_->registerCallback(boost::bind(&Valve3DProjector::apply, this, _1, _2, _3));
    }

    ros::V_string names = boost::assign::list_of("~input/pose")("~input/point")("~input/info");
    jsk_topic_tools::warnNoRemap(names);
  }

  void Valve3DProjector::unsubscribe()
  {
    sub_pose_.unsubscribe();
    sub_point_.unsubscribe();
    sub_info_.unsubscribe();
  }

  geometry_msgs::Pose Valve3DProjector::eigen2pose(Eigen::Vector3f vec, Eigen::Quaternionf q){
    geometry_msgs::Pose pose;
    pose.position.x = vec.x();
    pose.position.y = vec.y();
    pose.position.z = vec.z();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  Eigen::Vector3f  Valve3DProjector::pose2eigenVec(geometry_msgs::Pose pose){
    Eigen::Vector3f vec;
    vec.x() = pose.position.x;
    vec.y() = pose.position.y;
    vec.z() = pose.position.z;
    return vec;
  }

  void Valve3DProjector::apply(
    const geometry_msgs::PoseArray::ConstPtr& pose_msg,
    const opencv_apps::Point2DArrayStamped::ConstPtr& point_msg,
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    int i, j;

    //convert cv::Point2d to 3d vector
    image_geometry::PinholeCameraModel model;
    bool model_success_p = model.fromCameraInfo(info_msg);
    if (!model_success_p) {
      return;
    }
    cv::Point2d point;
    cv::Point3d ray;
    point = cv::Point2d(point_msg->points.at(0).x, point_msg->points.at(0).y);
    ray = model.projectPixelTo3dRay(point);
    //Eigen::Vector3f valve_dir(ray.x, ray.y, ray.z);
    Eigen::Vector3f valve_dir(ray.z, - ray.x, - ray.y);
    valve_dir = valve_dir.normalized();

    //convert ROI from geometry_msgs::Pose to position in camera coordinates
    geometry_msgs::PoseStamped original_pose, transformed_pose;
    std::vector<Eigen::Vector3f> roi_vertices;
    tf_listener_.waitForTransform(pose_msg->header.frame_id,
                                  point_msg->header.frame_id,
                                  point_msg->header.stamp,
                                  ros::Duration(1.0));

    //for(i = 8; i < pose_msg->poses.size(); i++){
    for(i = 0; i < 4; i++){
      try{
        original_pose.header = pose_msg->header;
        original_pose.pose = pose_msg->poses.at(i);
        tf_listener_.transformPose(point_msg->header.frame_id, original_pose, transformed_pose);
      } catch(tf::TransformException ex){
        ROS_WARN("transform error.");
        return;
      }
      Eigen::Vector3f pos =  Valve3DProjector::pose2eigenVec(transformed_pose.pose);
      roi_vertices.push_back(pos);
    }

    //estimate valve positions
    Eigen::Vector3f roi_orig, roi_dir1, roi_dir2;
    roi_orig = roi_vertices.at(3);
    roi_dir1 = (roi_vertices.at(2) - roi_vertices.at(3)).normalized(); // y-axis
    roi_dir2 = (roi_vertices.at(0) - roi_vertices.at(3)).normalized(); // z-axis
    Eigen::Vector3f valve_offset(-0.08, 0.0, 0.0); // valve position offset from panel

    //roi plane is n.dot(x) = h for all x on plane, valve position is alpha * valve_dir
    Eigen::Vector3f n = roi_dir1.cross(roi_dir2).normalized();
    double h = n.dot(roi_orig);
    double alpha = h / n.dot(valve_dir);
    Eigen::Vector3f valve_pos = alpha * valve_dir + valve_offset;


    Eigen::Matrix3f rot;
    rot.col(0) = n;
    rot.col(1) = roi_dir1;
    rot.col(2) = roi_dir2;
    Eigen::Quaternionf rot_q(rot);
    rot_q.normalize();

    geometry_msgs::Pose valve_pose = Valve3DProjector::eigen2pose(valve_pos, rot_q);

    tf::Transform valve_transform;
    tf::Quaternion rot_tf(rot_q.x(), rot_q.y(), rot_q.z(), rot_q.w());
    valve_transform.setOrigin(tf::Vector3(valve_pos.x(), valve_pos.y(), valve_pos.z()));
    valve_transform.setRotation(rot_tf);
    tf_br_.sendTransform(tf::StampedTransform(valve_transform, ros::Time::now(),
                                            point_msg->header.frame_id,
                                            "valve"));

    geometry_msgs::PoseStamped debug_msg;
    debug_msg.header = point_msg->header;
    debug_msg.pose = valve_pose;
    pub_.publish(debug_msg);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc_task2_perception::Valve3DProjector, nodelet::Nodelet);
