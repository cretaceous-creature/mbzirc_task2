#include <mbzirc_task2_perception/image_manual_clipper.h>

namespace mbzirc_task2_perception
{

  void ImageManualClipper::onInit()
  {
    DiagnosticNodelet::onInit();
    image_pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    camerainfo_pub_ = advertise<sensor_msgs::CameraInfo>(*pnh_, "output_info", 1);
    pnh_->param("run_flag", run_flag_, true);
    pre_flag_ = run_flag_;
  }

  void ImageManualClipper::subscribe()
  {
    image_sub_ = pnh_->subscribe("input", 1, &ImageManualClipper::imageCb, this);
    info_sub_ = pnh_->subscribe<sensor_msgs::CameraInfo>("input/info", 1, &ImageManualClipper::infoCb, this);
    rect_sub_ = pnh_->subscribe<geometry_msgs::PolygonStamped>("input/rect", 1, &ImageManualClipper::rectCb, this);
    ros::V_string names = boost::assign::list_of("~input")("~input/info")("~input/rect");
    jsk_topic_tools::warnNoRemap(names);
  }

  void ImageManualClipper::unsubscribe()
  {
    image_sub_.shutdown();
    rect_sub_.shutdown();
    info_sub_.shutdown();
  }

  void ImageManualClipper::imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pnh_->getParam("run_flag", run_flag_);
    if(pre_flag_ == true && run_flag_ == false)
      ROS_INFO("stop calculation");
    if(pre_flag_ == false && run_flag_ == true)
      ROS_INFO("start calculation");
    pre_flag_ = run_flag_;
    if(!run_flag_)
      return;

    if(roi.width != 0)
      {
        cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat clip_img(img->image, cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height));
        sensor_msgs::ImagePtr clip_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", clip_img).toImageMsg();
        clip_msg->header = msg->header;
        image_pub_.publish(clip_msg);

      }
  }

  void ImageManualClipper::infoCb(const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pnh_->getParam("run_flag", run_flag_);
    if(pre_flag_ == true && run_flag_ == false)
      ROS_INFO("stop calculation");
    if(pre_flag_ == false && run_flag_ == true)
      ROS_INFO("start calculation");
    pre_flag_ = run_flag_;
    if(!run_flag_)
      return;

    if(roi.width != 0){
      sensor_msgs::CameraInfo stamp_corrected_roi(*info_msg);
      stamp_corrected_roi.roi = roi;
      camerainfo_pub_.publish(stamp_corrected_roi);
    } else {
      init_info = info_msg;
    }
  }

  void ImageManualClipper::rectCb(const geometry_msgs::PolygonStamped::ConstPtr& rect_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(init_info){
      geometry_msgs::Point32 P0 = rect_msg->polygon.points[0];
      geometry_msgs::Point32 P1 = rect_msg->polygon.points[1];
      double min_x = std::max(std::min(P0.x, P1.x), 0.0f);
      double max_x = std::max(P0.x, P1.x);
      double min_y = std::max(std::min(P0.y, P1.y), 0.0f);
      double max_y = std::max(P0.y, P1.y);
      double width = std::min(max_x - min_x, init_info->width - min_x);
      double height = std::min(max_y - min_y, init_info->height - min_y);
      roi.x_offset = (int)min_x;
      roi.y_offset = (int)min_y;
      roi.height = height;
      roi.width = width;
    }
  }

};


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc_task2_perception::ImageManualClipper, nodelet::Nodelet);
