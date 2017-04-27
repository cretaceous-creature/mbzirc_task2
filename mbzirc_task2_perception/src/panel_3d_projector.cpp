#include <image_geometry/pinhole_camera_model.h>
#include "mbzirc_task2_perception/panel_3d_projector.h"
#include <jsk_recognition_utils/cv_utils.h>
#include <jsk_topic_tools/log_utils.h>
#include <cv_bridge/cv_bridge.h>

namespace mbzirc_task2_perception
{
  void Panel3DProjector::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, true);
    pub_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void Panel3DProjector::subscribe()
  {
    sub_rect_.subscribe(*pnh_, "input/rect", 1);
    sub_info_.subscribe(*pnh_, "input/info", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(1000);
      async_->connectInput(sub_rect_, sub_info_);
      async_->registerCallback(boost::bind(&Panel3DProjector::apply, this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_rect_, sub_info_);
      sync_->registerCallback(boost::bind(&Panel3DProjector::apply, this, _1, _2));
    }

    ros::V_string names = boost::assign::list_of("~input/rect")("~input/info");
    jsk_topic_tools::warnNoRemap(names);
  }

  void Panel3DProjector::unsubscribe()
  {
    sub_rect_.unsubscribe();
    sub_info_.unsubscribe();
  }

  void Panel3DProjector::apply(
    const geometry_msgs::PolygonStamped::ConstPtr& rect_msg,
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    double panel_height = 1.35;

    int i;
    if(rect_msg->polygon.points.size() != 2)
      {
        ROS_WARN("rectangle message size is wrong!");
        return;
      }

    //convert 2d point to 3d point
    image_geometry::PinholeCameraModel model;
    bool model_success_p = model.fromCameraInfo(info_msg);
    if (!model_success_p) {
      ROS_ERROR("failed to make camera model!");
      return;
    }

    cv::Point2d point;
    std::vector<cv::Point3d> rays;
    for(i = 0; i < 2; i++){
      point = cv::Point2d(rect_msg->polygon.points.at(i).x, rect_msg->polygon.points.at(i).y);
      rays.push_back(model.projectPixelTo3dRay(point)); // 0: left-upper 1: right-lower
      //Eigen::Vector3f direction(ray.z, - ray.x, - ray.y);
    }

    //calculate z position
    double panel_dist = panel_height / ((rays.at(1).y / rays.at(1).z) - (rays.at(0).y / rays.at(0).z));

    geometry_msgs::PoseStamped panel_pose_msg;
    panel_pose_msg.header = info_msg->header;
    // for z-optical axis
    panel_pose_msg.pose.position.x =  ((rays.at(0).x / rays.at(0).z) + (rays.at(1).x / rays.at(1).z)) / 2.0 * panel_dist;
    panel_pose_msg.pose.position.y =  ((rays.at(0).y / rays.at(0).z) + (rays.at(1).y / rays.at(1).z)) / 2.0 * panel_dist;
    panel_pose_msg.pose.position.z =  panel_dist;
    // for x-optical axis
    // panel_pose_msg.pose.position.x =  panel_dist;
    // panel_pose_msg.pose.position.y =  - ((rays.at(0).x / rays.at(0).z) + (rays.at(1).x / rays.at(1).z)) / 2.0 * panel_dist;
    // panel_pose_msg.pose.position.z =  - ((rays.at(0).y / rays.at(0).z) + (rays.at(1).y / rays.at(1).z)) / 2.0 * panel_dist;

    pub_.publish(panel_pose_msg);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc_task2_perception::Panel3DProjector, nodelet::Nodelet);
