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


#ifndef MBZIRC_TASK2_PERCEPTION_VALVE_3D_PROJECTOR_H_
#define MBZIRC_TASK2_PERCEPTION_VALVE_3D_PROJECTOR_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv_apps/Point2DArrayStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <jsk_recognition_utils/geo/line.h>
#include <jsk_recognition_utils/geo_util.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

namespace mbzirc_task2_perception
{
  class Valve3DProjector: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
      geometry_msgs::PoseArray,
      opencv_apps::Point2DArrayStamped,
      sensor_msgs::CameraInfo > ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      geometry_msgs::PoseArray,
      opencv_apps::Point2DArrayStamped,
      sensor_msgs::CameraInfo > SyncPolicy;

    Valve3DProjector(): DiagnosticNodelet("Valve3DProjector") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual geometry_msgs::Pose eigen2pose(Eigen::Vector3f vec, Eigen::Quaternionf q);
    virtual Eigen::Vector3f pose2eigenVec(geometry_msgs::Pose pose);
    virtual void apply(const geometry_msgs::PoseArray::ConstPtr& pose_msg,
                       const opencv_apps::Point2DArrayStamped::ConstPtr& point_msg,
                       const sensor_msgs::CameraInfo::ConstPtr& info_msg);

    boost::mutex mutex_;
    message_filters::Subscriber<geometry_msgs::PoseArray> sub_pose_;
    message_filters::Subscriber<opencv_apps::Point2DArrayStamped> sub_point_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    ros::Publisher pub_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_br_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;

    bool approximate_sync_;

  private:

  };
}

#endif
