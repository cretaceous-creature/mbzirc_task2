#ifndef MBZIRC_TASK2_PERCEPTION_MEANSHIFT_FILTERING_H_
#define MBZIRC_TASK2_PERCEPTION_MEANSHIFT_FILTERING_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <boost/assign.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <mbzirc_task2_perception/AreaArrayStamped.h>
#include <dynamic_reconfigure/server.h>
#include <mbzirc_task2_perception/MeanshiftFilteringConfig.h>

#ifdef USE_CUDA
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#endif

namespace mbzirc_task2_perception
{
  class MeanshiftFiltering: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef mbzirc_task2_perception::MeanshiftFilteringConfig Config;
    MeanshiftFiltering(): DiagnosticNodelet("MeanshiftFiltering") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config &config, uint32_t level);
    virtual void imageCb(const sensor_msgs::ImageConstPtr& image_msg);

    ros::Subscriber image_sub_;
    ros::Publisher pub_;
    ros::Publisher color_pub_;
    sensor_msgs::CameraInfo::ConstPtr roi;
    ros::Publisher area_pub_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
    double meanshift_sp1_;
    double meanshift_sp2_;
    double meanshift_sr1_;
    double meanshift_sr2_;
    int clustering_thre_;
    int clustering_mini_thre_;
    bool use_cuda;
    bool screen_debug_;

  private:

  };
}

#endif
