#ifndef MBZIRC_TASK2_PERCEPTION_ROI_GENERATOR_H_
#define MBZIRC_TASK2_PERCEPTION_ROI_GENERATOR_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <boost/assign.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <jsk_recognition_utils/geo/polygon.h>
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
#include <image_geometry/pinhole_camera_model.h>

namespace mbzirc_task2_perception
{
  class ROIGenerator: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    ROIGenerator(): DiagnosticNodelet("ROIGenerator") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual geometry_msgs::Pose eigen2pose(Eigen::Vector3f vec);
    virtual geometry_msgs::Pose eigen2pose(Eigen::Vector3f vec, Eigen::Quaternionf q);
    virtual Eigen::Vector3f pose2eigen(geometry_msgs::Pose pose);
    virtual void get_side(int start_id, std::vector<int>& side_id,
                            std::vector<geometry_msgs::Pose>& corners);
    virtual sensor_msgs::CameraInfo computeROI(const sensor_msgs::CameraInfo::ConstPtr& msg,
                                                 std::vector<cv::Point2d>& points);
    virtual void infoCb(const sensor_msgs::CameraInfoConstPtr& msg);
    virtual void polygonCb(const jsk_recognition_msgs::PolygonArrayConstPtr& msg);

    ros::Subscriber polygon_sub_;
    ros::Subscriber info_sub_;
    ros::Publisher pub_;
    ros::Publisher shaft_pub_;
    ros::Publisher debug_pub_;
    ros::Publisher wrench_pub_;
    tf::TransformListener tf_listener_;
    std::string target_frame;
    tf::TransformBroadcaster br_;
    std::vector<geometry_msgs::Pose> vertices_pose;

  private:

  };
}

#endif
