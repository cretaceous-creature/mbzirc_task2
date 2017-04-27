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
#include "mbzirc_task2_perception/wrench_3d_projector.h"
#include <jsk_recognition_utils/cv_utils.h>
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

namespace mbzirc_task2_perception
{
  void Wrench3DProjector::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, true);
    top_pub_ = advertise<geometry_msgs::PoseArray>(*pnh_, "pose_top", 1);
    bottom_pub_ = advertise<geometry_msgs::PoseArray>(*pnh_, "pose_bottom", 1);
    onInitPostProcess();
  }

  void Wrench3DProjector::subscribe()
  {
    sub_pose_.subscribe(*pnh_, "input/pose", 1);
    sub_point_.subscribe(*pnh_, "input/point", 1);
    sub_info_.subscribe(*pnh_, "input/info", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      async_->connectInput(sub_pose_, sub_point_, sub_info_);
      async_->registerCallback(boost::bind(&Wrench3DProjector::apply, this, _1, _2, _3));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_pose_, sub_point_, sub_info_);
      sync_->registerCallback(boost::bind(&Wrench3DProjector::apply, this, _1, _2, _3));
    }

    ros::V_string names = boost::assign::list_of("~input/pose")("~input/point")("~input/info");
    jsk_topic_tools::warnNoRemap(names);
  }

  void Wrench3DProjector::unsubscribe()
  {
    sub_pose_.unsubscribe();
    sub_point_.unsubscribe();
    sub_info_.unsubscribe();
  }

  geometry_msgs::Pose  Wrench3DProjector::eigen2pose(Eigen::Vector3f vec){
    geometry_msgs::Pose pose;
    pose.position.x = vec.x();
    pose.position.y = vec.y();
    pose.position.z = vec.z();
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1.0;
    return pose;
  }

  geometry_msgs::Pose  Wrench3DProjector::eigen2pose(Eigen::Vector3f vec, Eigen::Quaternionf q){
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

  Eigen::Vector3f  Wrench3DProjector::pose2eigenVec(geometry_msgs::Pose pose){
    Eigen::Vector3f vec;
    vec.x() = pose.position.x;
    vec.y() = pose.position.y;
    vec.z() = pose.position.z;
    return vec;
  }

  Eigen::Quaternionf  Wrench3DProjector::pose2eigenQ(geometry_msgs::Pose pose){
    Eigen::Quaternionf q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    q.normalize();
    return q;
  }

  void Wrench3DProjector::apply(
    const geometry_msgs::PoseArray::ConstPtr& pose_msg,
    const opencv_apps::Point2DArrayStamped::ConstPtr& point_msg,
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    if(pose_msg->poses.size() != point_msg->points.size())
      {
        ROS_WARN("pose message size and point message size are different!");
        return;
      }

    int size = point_msg->points.size();
    int i, j;
    std::vector<cv::Point2d> points;

    //convert cv::Point2d to line
    image_geometry::PinholeCameraModel model;
    bool model_success_p = model.fromCameraInfo(info_msg);
    if (!model_success_p) {
      return;
    }
    std::vector<jsk_recognition_utils::Line> point_lines;
    cv::Point2d point;
    cv::Point3d ray;
    for(i = 0; i < size; i++){
      point = cv::Point2d(point_msg->points.at(i).x, point_msg->points.at(i).y);
      points.push_back(point);
      ray = model.projectPixelTo3dRay(point);
      //for z-optical axis
      Eigen::Vector3f direction(ray.x, ray.y, ray.z);
      //for x-optical axis
      //Eigen::Vector3f direction(ray.z, - ray.x, - ray.y);
      jsk_recognition_utils::Line line = jsk_recognition_utils::Line(direction, Eigen::Vector3f(0.0, 0.0, 0.0));
      point_lines.push_back(line);
    }

    //convert geometry_msgs::Pose to line ##poses must be aligned in from-left-to-right order##
    geometry_msgs::PoseStamped original_pose, transformed_pose;
    std::vector<jsk_recognition_utils::Line> pose_lines;
    std::vector<Eigen::Quaternionf> rots;
    std::vector<geometry_msgs::Pose> top_poses;
    tf_listener_.waitForTransform(pose_msg->header.frame_id,
                                  point_msg->header.frame_id,
                                  point_msg->header.stamp,
                                  ros::Duration(1.0));

    for(i = 0; i < size; i++){
      try{
        original_pose.header = pose_msg->header;
        original_pose.pose = pose_msg->poses.at(i);
        tf_listener_.transformPose(point_msg->header.frame_id, original_pose, transformed_pose);
      } catch(tf::TransformException ex){
        ROS_WARN("transform error.");
        return;
      }
      Eigen::Vector3f origin =  Wrench3DProjector::pose2eigenVec(transformed_pose.pose);
      Eigen::Quaternionf rot =  Wrench3DProjector::pose2eigenQ(transformed_pose.pose);
      rots.push_back(rot);
      top_poses.push_back(transformed_pose.pose);
      Eigen::Vector3f direction = Eigen::Matrix3f(rot) * Eigen::Vector3f(0.0, 0.0, -1.0);
      jsk_recognition_utils::Line line = jsk_recognition_utils::Line(direction, origin);
      pose_lines.push_back(line);
    }

    //sort poses
    std::vector<jsk_recognition_utils::Line> sorted_pose_lines;
    std::vector<cv::Point2d> points_copy = points;
    std::sort(points_copy.begin(), points_copy.end(), point_compare<cv::Point2d>);
    std::vector<geometry_msgs::Pose> sorted_top_poses;
    sorted_top_poses.resize(size);
    for(i = 0; i < size; i++){
      for(j = 0; j < size; j++){
        if(points.at(i).x == points_copy.at(j).x){
          sorted_pose_lines.push_back(pose_lines.at(j));
          sorted_top_poses.at(i) = top_poses.at(j);
          break;
        }
      }
    }

    //estimate wrench positions
    Eigen::Vector3f commonperpend1, commonperpend2;
    Eigen::Vector3f bottom_pos;
    std::vector<geometry_msgs::Pose> bottom_poses;
    bottom_poses.resize(size);
    for(i = 0; i < size; i++){
      sorted_pose_lines.at(i).nearestPoints(point_lines.at(i), commonperpend1, commonperpend2);
      bottom_pos = (commonperpend1 + commonperpend2) / 2.0;
      bottom_poses.at(i) = Wrench3DProjector::eigen2pose(bottom_pos, rots.at(i));
    }

    geometry_msgs::PoseArray bottom_msg;
    bottom_msg.header = point_msg->header;
    bottom_msg.poses = bottom_poses;
    geometry_msgs::PoseArray top_msg;
    top_msg.header = point_msg->header;
    top_msg.poses = sorted_top_poses;
    bottom_pub_.publish(bottom_msg);
    top_pub_.publish(top_msg);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc_task2_perception::Wrench3DProjector, nodelet::Nodelet);
