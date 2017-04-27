#include <mbzirc_task2_perception/luminance_homogenizer.h>

namespace mbzirc_task2_perception
{

  void LuminanceHomogenizer::onInit()
  {
    DiagnosticNodelet::onInit();
    homogenized_pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    v_pub_ = advertise<sensor_msgs::Image>(*pnh_, "output_v", 1);
    s_pub_ = advertise<sensor_msgs::Image>(*pnh_, "output_s", 1);
    h_pub_ = advertise<sensor_msgs::Image>(*pnh_, "output_h", 1);
    pnh_->param("run_flag", run_flag_, true);
    pnh_->param("screen_debug", screen_debug_, false);
    pre_flag_ = run_flag_;
  }

  void LuminanceHomogenizer::subscribe()
  {
    image_sub_ = pnh_->subscribe("input", 1, &LuminanceHomogenizer::imageCb, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void LuminanceHomogenizer::unsubscribe()
  {
    image_sub_.shutdown();
  }

  void LuminanceHomogenizer::imageCb(const sensor_msgs::ImageConstPtr& image_msg)
  {
    pnh_->getParam("run_flag", run_flag_);
    if(pre_flag_ == true && run_flag_ == false)
      ROS_INFO("stop calculation");
    if(pre_flag_ == false && run_flag_ == true)
      ROS_INFO("start calculation");
    pre_flag_ = run_flag_;
    if(!run_flag_)
      return;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
    cv::Mat image(cv_ptr->image.rows, cv_ptr->image.cols, CV_32F);
    cv_ptr->image.convertTo(image, CV_32F, 1.0/255.0);
    cv::Mat blured_in_img(image.rows, image.cols, CV_32F);
    cv::Mat hsv_image(image.rows, image.cols, CV_32F);
    std::vector<cv::Mat1f> hsv_planes;
    cv::Mat v(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);
    cv::Mat s(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);
    cv::Mat h(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);

    cv::medianBlur(image, blured_in_img, 3);

    int i, j, k;
    if (image_msg->encoding == sensor_msgs::image_encodings::BGR8) {
      cv::cvtColor(blured_in_img, hsv_image, CV_BGR2HSV);
    }
    else if (image_msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(blured_in_img, hsv_image, CV_RGB2HSV);
    }
    else {
      return;
    }
    cv::split(hsv_image, hsv_planes);
    h = hsv_planes[0];
    s = hsv_planes[1];
    v = hsv_planes[2];

    cv::Mat v_cnv;
    v.convertTo(v_cnv, CV_8U, 255.0);
    cv::Mat normalized_s, normalized_h;
    cv::normalize(s, normalized_s, 0.0, 1.0, cv::NORM_MINMAX);
    cv::normalize(h, normalized_h, 0.0, 1.0, cv::NORM_MINMAX);

    //PCA
    cv::Mat pca_src(s.cols * s.rows, 2, CV_32FC1);
    for(i = 0; i < s.rows; i++){
      for(j = 0; j < s.cols; j++){
        pca_src.at<float>(i * s.cols + j, 0) = s.at<float>(i, j) * cos(M_PI * h.at<float>(i, j) / 180.0) + 0.5;
        pca_src.at<float>(i * s.cols + j, 1) = s.at<float>(i, j) * sin(M_PI * h.at<float>(i, j) / 180.0) + 0.5;
      }
    }
    cv::PCA pca(pca_src, cv::Mat(), CV_PCA_DATA_AS_ROW, 0);
    cv::Mat pca_result(2, s.cols * s.rows, CV_32FC1);
    pca_result = pca.project(pca_src);
    cv::Mat pca_output(s.rows,s.cols, CV_32FC1);

    for(i = 0; i < s.rows; i++){
      for(j = 0; j < s.cols; j++){
        pca_output.at<float>(i, j) = pca_result.at<float>(i * s.cols + j, 0);
      }
    }
    cv::Mat evalues = pca.eigenvalues;
    double sum = 0.0;
    for(i = 0;i < pca.eigenvalues.rows; i++){
      sum += evalues.at<float>(i, 0);
    }
    double contribution = 0.0;
    for(i = 0;i < pca.eigenvalues.rows; i++){
      contribution += evalues.at<float>(i, 0) / sum;
    }
    cv::Mat s_cnv;
    normalized_s.convertTo(s_cnv, CV_8U, 255);
    cv::Mat h_cnv;
    normalized_h.convertTo(h_cnv, CV_8U, 255);
    cv::Mat normalized_pca;
    cv::normalize(pca_output, normalized_pca, 0.0, 1.0, cv::NORM_MINMAX);

    //advanced normalize
    cv::MatND histgram;
    int channels[] = {0};
    int bin_num = 64;
    int bin_nums[] = {bin_num};
    float range[] = {0, 1.0};
    const float *ranges[] = {range};
    cv::calcHist(&normalized_pca, 1, channels, cv::Mat(), histgram, 1, bin_nums, ranges, true, false);

    //check near 0/1 pixels
    double min_hist = 0.0;
    double max_hist = 0.0;
    bool min_cut, max_cut;
    double cut_thre = 0.15;
    double bin_thre = 10.0;
    for(i = 0; i < bin_thre; i++){
      min_hist += histgram.at<float>(0, i);
      max_hist += histgram.at<float>(0, bin_num - 1 - i);
    }
    if((min_hist / normalized_pca.rows / normalized_pca.cols) < cut_thre){
      min_cut = true;
    } else {
      min_cut = false;
    }
    if((max_hist / normalized_pca.rows / normalized_pca.cols) < cut_thre){
      max_cut = true;
    } else {
      max_cut = false;
    }

    ROS_INFO("advanced normalize flag: min_cut:%d max_cut:%d", min_cut, max_cut);
    for(i = 0; i < normalized_pca.rows; i++){
      for(j = 0; j < normalized_pca.cols; j++){
        if(min_cut && normalized_pca.at<float>(i, j) < bin_thre / bin_num){
          normalized_pca.at<float>(i, j) = bin_thre / bin_num;
        }
        if(max_cut && normalized_pca.at<float>(i, j) > 1.0 - (bin_thre / bin_num)){
          normalized_pca.at<float>(i, j) = 1.0 - (bin_thre / bin_num);
        }
      }
    }
    cv::Mat advanced_normalized_pca;
    cv::normalize(normalized_pca, advanced_normalized_pca, 0.0, 1.0, cv::NORM_MINMAX);

    //view histgram
    if(screen_debug_){
      cv::calcHist(&advanced_normalized_pca, 1, channels, cv::Mat(), histgram, 1, bin_nums, ranges, true, false);
      std::cout << "normalized histram: " << std::endl;
      for(i = 0; i < bin_num; i++){
        std::cout << std::setw(6) << std::setfill('0') << histgram.at<float>(0, i) << " : ";
        for(j = 0; j < histgram.at<float>(0, i) / 400; j++){
          std::cout << "[]";
        }
        std::cout << std::endl;
      }
    }

    // remove noize by median filter
    advanced_normalized_pca.convertTo(pca_output, CV_8U, 255);
    cv::Mat pre_blured_out_img = pca_output.clone();
    cv::Mat blured_out_img = pca_output.clone();
    cv::medianBlur(pca_output, blured_out_img, 5);
    cv::medianBlur(blured_out_img, pre_blured_out_img, 3);
    cv::medianBlur(pre_blured_out_img, blured_out_img, 3);

    // apply unsharp mask
    cv::Mat unsharped_img = blured_out_img.clone();
    double s_k = 3.0;
    float filter_arr[3][3] =
      {{-s_k/9.0, -s_k/9.0, -s_k/9.0}, {-s_k/9.0, 1.0 + 8.0*s_k/9.0, -s_k/9.0}, {-s_k/9.0, -s_k/9.0, -s_k/9.0}};
    cv::Mat filter(cv::Size(3,3), CV_32F, filter_arr);
    cv::filter2D(blured_out_img, unsharped_img, -1, filter);
    cv::Mat unsharped_cnv;
    unsharped_img.convertTo(unsharped_cnv, CV_8U);

    v_pub_.publish(cv_bridge::CvImage(
                     image_msg->header,
                     sensor_msgs::image_encodings::MONO8,
                     v_cnv).toImageMsg());
    s_pub_.publish(cv_bridge::CvImage(
                      image_msg->header,
                      sensor_msgs::image_encodings::MONO8,
                      s_cnv).toImageMsg());
    h_pub_.publish(cv_bridge::CvImage(
                      image_msg->header,
                      sensor_msgs::image_encodings::MONO8,
                      h_cnv).toImageMsg());

    homogenized_pub_.publish(cv_bridge::CvImage(
                     image_msg->header,
                     sensor_msgs::image_encodings::MONO8,
                     unsharped_cnv).toImageMsg());

  }

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc_task2_perception::LuminanceHomogenizer, nodelet::Nodelet);
