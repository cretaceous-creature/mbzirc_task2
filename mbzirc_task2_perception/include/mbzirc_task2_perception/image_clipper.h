#ifndef MBZIRC_TASK2_PERCEPTION_IMAGE_CLIPPER_H_
#define MBZIRC_TASK2_PERCEPTION_IMAGE_CLIPPER_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <boost/assign.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

namespace mbzirc_task2_perception
{
  class ImageClipper: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    ImageClipper(): DiagnosticNodelet("ImageClipper") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void imageCb(const sensor_msgs::ImageConstPtr& msg);
    virtual void rectCb(const sensor_msgs::CameraInfo::ConstPtr& info_msg);

    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
    ros::Subscriber rect_sub_;
    ros::Publisher camerainfo_pub_;
    sensor_msgs::CameraInfo::ConstPtr roi;
    bool run_flag_;
    bool pre_flag_;

  private:

  };
}

#endif
