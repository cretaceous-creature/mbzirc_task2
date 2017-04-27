#ifndef MBZIRC_TASK2_PERCEPTION_LUMINANCE_HOMOGENIZER_H_
#define MBZIRC_TASK2_PERCEPTION_LUMINANCE_HOMOGENIZER_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <boost/assign.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

namespace mbzirc_task2_perception
{
  class LuminanceHomogenizer: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    LuminanceHomogenizer(): DiagnosticNodelet("LuminanceHomogenizer") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void imageCb(const sensor_msgs::ImageConstPtr& image_msg);

    ros::Subscriber image_sub_;
    ros::Publisher homogenized_pub_;
    ros::Publisher v_pub_;
    ros::Publisher s_pub_;
    ros::Publisher h_pub_;
    sensor_msgs::CameraInfo::ConstPtr roi;

    bool run_flag_;
    bool screen_debug_;
    bool pre_flag_;
  private:

  };
}

#endif
