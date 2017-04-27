#include <mbzirc_task2_perception/image_clipper.h>

namespace mbzirc_task2_perception
{

  void ImageClipper::onInit()
  {
    DiagnosticNodelet::onInit();
    image_pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    camerainfo_pub_ = advertise<sensor_msgs::CameraInfo>(*pnh_, "output_info", 1);
    pnh_->param("run_flag", run_flag_, true);
    pre_flag_ = run_flag_;
  }

  void ImageClipper::subscribe()
  {
    image_sub_ = pnh_->subscribe("input", 1, &ImageClipper::imageCb, this);
    rect_sub_ = pnh_->subscribe<sensor_msgs::CameraInfo>("input/rect", 1, &ImageClipper::rectCb, this);
    ros::V_string names = boost::assign::list_of("~input")("~input/rect");
    jsk_topic_tools::warnNoRemap(names);
  }

  void ImageClipper::unsubscribe()
  {
    image_sub_.shutdown();
    rect_sub_.shutdown();
  }

  void ImageClipper::imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    pnh_->getParam("run_flag", run_flag_);
    if(pre_flag_ == true && run_flag_ == false)
      ROS_INFO("stop calculation");
    if(pre_flag_ == false && run_flag_ == true)
      ROS_INFO("start calculation");
    pre_flag_ = run_flag_;
    if(!run_flag_)
      return;

    if(roi)
      {
        cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat clip_img(img->image, cv::Rect(roi->roi.x_offset, roi->roi.y_offset, roi->roi.width, roi->roi.height));
        sensor_msgs::ImagePtr clip_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", clip_img).toImageMsg();
        clip_msg->header = msg->header;
        sensor_msgs::CameraInfo stamp_corrected_roi(*roi);
        stamp_corrected_roi.header = msg->header;
        image_pub_.publish(clip_msg);
        camerainfo_pub_.publish(stamp_corrected_roi);
      }
  }

  void ImageClipper::rectCb(const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {

    try
      {
        roi = info_msg;
      }
    catch (...)
      {
        ROS_ERROR("CameraInfo error");
        return;
      }
  }

};


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc_task2_perception::ImageClipper, nodelet::Nodelet);
