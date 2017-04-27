#include <mbzirc_task2_perception/insigma_normalizer.h>
#include <time.h>

namespace mbzirc_task2_perception
{
  void InsigmaNormalizer::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&InsigmaNormalizer::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
#ifdef USE_CUDA
    if(cv::cuda::getCudaEnabledDeviceCount() > 0){
      use_cuda = true;
      cv::cuda::DeviceInfo info(0);
      ROS_INFO("GPU FOUND : %s", info.name());
    } else {
      use_cuda = false;
      ROS_INFO("NO GPU FOUND : RUN CPU MODE");
    }
#else
      use_cuda = false;
      ROS_INFO("RUN CPU MODE");
#endif
  }

  void InsigmaNormalizer::subscribe()
  {
    image_sub_ = pnh_->subscribe("input", 1, &InsigmaNormalizer::imageCb, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void InsigmaNormalizer::unsubscribe()
  {
    image_sub_.shutdown();
  }

  void InsigmaNormalizer::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    section_size_ = config.section_size;
    screen_debug_ = config.screen_debug;
  }

  void InsigmaNormalizer::imageCb(const sensor_msgs::ImageConstPtr& image_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);//MONO8

    // output image of filtering
    cv::Mat normalized_dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);

    // variables for calculating histgram
    int i, j, k ,l, m;
    int channels[] = {0};
    int bin_num = 32;
    int bin_nums[] = {bin_num};
    float range[] = {0, 255};
    const float *ranges[] = {range};
    cv::MatND original_histgram(1, bin_num, CV_32FC1);
    cv::MatND histgram(1, bin_num, CV_32FC1);

    //variables for estimating distribution
    double mu = 0.0;
    double mu2 = 0.0;
    double sigma;
    double upper_thre, lower_thre;

#ifdef USE_CUDA
    if(use_cuda){ //gpu mode
      // clock_t start = clock();
      // std::cout << "GPU processing start" << std::endl;

      // normalize image
      cv::cuda::GpuMat cuda_img(cv_ptr->image);
      cv::cuda::GpuMat cuda_normalized_dst;
      cv::cuda::Stream stream;
      stream.waitForCompletion();
      cv::cuda::normalize(cuda_img, cuda_normalized_dst, 0, 255, cv::NORM_MINMAX, -1);
      cuda_img.release();

      // calculate histgram
      cv::cuda::GpuMat cuda_histgram;
      cv::cuda::calcHist(cuda_normalized_dst, cuda_histgram);
      cv::Mat mat_histgram;
      cuda_histgram.download(mat_histgram);
      int merge_num = 256 / bin_num;
      for(i = 0; i < bin_num; i++){
        float hist = 0;
        for(j = 0; j < merge_num; j++){
          hist += mat_histgram.at<int>(0, i * merge_num + j);
          }
        original_histgram.at<float>(0, i) = hist;
      }

      //estimate distribution
      for(i = 0; i < bin_num; i++){
        mu += i * (256 / bin_num) * original_histgram.at<float>(0, i) / (cv_ptr->image.rows * cv_ptr->image.cols);
        mu2 += i * (256 / bin_num) * i * (256 / bin_num) * original_histgram.at<float>(0, i) / (cv_ptr->image.rows * cv_ptr->image.cols);
      }
      sigma = sqrt(mu2 - mu * mu);
      upper_thre = std::min(mu + sigma * section_size_, 255.0);
      lower_thre = std::max(mu - sigma * section_size_, 0.0);
      if(screen_debug_){
        std::cout << "estimated distribution : mu = " << mu << " sigma = " << sigma << std::endl;
        std::cout << "upper thre : " << upper_thre << " lower thre : " << lower_thre << std::endl;
      }

      //remove outside of sigma section
      cv::cuda::GpuMat cuda_cut_upper_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::cuda::GpuMat cuda_not_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::cuda::GpuMat cuda_cut_lower_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::cuda::GpuMat cuda_insigma_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::cuda::GpuMat cuda_insigma_normalized_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::cuda::threshold(cuda_normalized_dst, cuda_cut_upper_img, upper_thre, 255, cv::THRESH_TRUNC);
      cuda_normalized_dst.release();
      cv::cuda::bitwise_not(cuda_cut_upper_img, cuda_not_img);
      cuda_cut_upper_img.release();
      cv::cuda::threshold(cuda_not_img, cuda_cut_lower_img, 255 - lower_thre, 255, cv::THRESH_TRUNC);
      cuda_not_img.release();
      cv::cuda::bitwise_not(cuda_cut_lower_img, cuda_insigma_img);
      cuda_cut_lower_img.release();

      // re-normalize image
      cv::cuda::normalize(cuda_insigma_img, cuda_insigma_normalized_img, 0, 255, cv::NORM_MINMAX, -1);
      cuda_insigma_img.release();
      cuda_insigma_normalized_img.download(normalized_dst);

      // calculate output image histgram
      cv::cuda::calcHist(cuda_insigma_normalized_img, cuda_histgram);
      cuda_insigma_normalized_img.release();
      cuda_histgram.download(mat_histgram);
      cuda_histgram.release();
      for(i = 0; i < bin_num; i++){
        float hist = 0;
        for(j = 0; j < merge_num; j++){
          hist += mat_histgram.at<int>(0, i * merge_num + j);
          }
        histgram.at<float>(0, i) = hist;
      }
      // clock_t end = clock();
      // std::cout << "GPU processing end. run time : " << (double)(end - start) / CLOCKS_PER_SEC * 1000 << "ms."<< std::endl;
    } else { // cpu mode
      // clock_t start = clock();
      // std::cout << "CPU processing start" << std::endl;

      cv::normalize(cv_ptr->image, normalized_dst, 0, 255, cv::NORM_MINMAX);
      cv::calcHist(&normalized_dst, 1, channels, cv::Mat(), original_histgram, 1, bin_nums, ranges, true, false);

      //estimate distribution
      for(i = 0; i < bin_num; i++){
        mu += i * (256 / bin_num) * original_histgram.at<float>(0, i) / (cv_ptr->image.rows * cv_ptr->image.cols);
        mu2 += i * (256 / bin_num) * i * (256 / bin_num) * original_histgram.at<float>(0, i) / (cv_ptr->image.rows * cv_ptr->image.cols);
      }
      sigma = sqrt(mu2 - mu * mu);
      upper_thre = std::min(mu + sigma * section_size_, 255.0);
      lower_thre = std::max(mu - sigma * section_size_, 0.0);
      if(screen_debug_){
        std::cout << "estimated distribution : mu = " << mu << " sigma = " << sigma << std::endl;
        std::cout << "upper thre : " << upper_thre << " lower thre : " << lower_thre << std::endl;
      }

      //remove outside of sigma section
      cv::Mat cut_upper_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::Mat not_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::Mat cut_lower_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::Mat insigma_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::Mat insigma_normalized_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::threshold(normalized_dst, cut_upper_img, upper_thre, 255, cv::THRESH_TRUNC);
      cv::bitwise_not(cut_upper_img, not_img);
      cv::threshold(not_img, cut_lower_img, 255 - lower_thre, 255, cv::THRESH_TRUNC);
      cv::bitwise_not(cut_lower_img, insigma_img);
      cv::normalize(insigma_img, insigma_normalized_img, 0, 255, cv::NORM_MINMAX, -1);
      cv::calcHist(&insigma_normalized_img, 1, channels, cv::Mat(), histgram, 1, bin_nums, ranges, true, false);

      // clock_t end = clock();
      // std::cout << "CPU processing end. run time : " << (double)(end - start) / CLOCKS_PER_SEC * 1000 << "ms."<< std::endl;
    }
#else
      // clock_t start = clock();
      // std::cout << "CPU processing start" << std::endl;

      cv::normalize(cv_ptr->image, normalized_dst, 0, 255, cv::NORM_MINMAX);
      cv::calcHist(&normalized_dst, 1, channels, cv::Mat(), original_histgram, 1, bin_nums, ranges, true, false);

      //estimate distribution
      for(i = 0; i < bin_num; i++){
        mu += i * (256 / bin_num) * original_histgram.at<float>(0, i) / (cv_ptr->image.rows * cv_ptr->image.cols);
        mu2 += i * (256 / bin_num) * i * (256 / bin_num) * original_histgram.at<float>(0, i) / (cv_ptr->image.rows * cv_ptr->image.cols);
      }
      sigma = sqrt(mu2 - mu * mu);
      upper_thre = std::min(mu + sigma * section_size_, 255.0);
      lower_thre = std::max(mu - sigma * section_size_, 0.0);
      if(screen_debug_){
        std::cout << "estimated distribution : mu = " << mu << " sigma = " << sigma << std::endl;
        std::cout << "upper thre : " << upper_thre << " lower thre : " << lower_thre << std::endl;
      }

      //remove outside of sigma section
      cv::Mat cut_upper_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::Mat not_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::Mat cut_lower_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::Mat insigma_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::Mat insigma_normalized_img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::threshold(normalized_dst, cut_upper_img, upper_thre, 255, cv::THRESH_TRUNC);
      cv::bitwise_not(cut_upper_img, not_img);
      cv::threshold(not_img, cut_lower_img, 255 - lower_thre, 255, cv::THRESH_TRUNC);
      cv::bitwise_not(cut_lower_img, insigma_img);
      cv::normalize(insigma_img, insigma_normalized_img, 0, 255, cv::NORM_MINMAX, -1);
      cv::calcHist(&insigma_normalized_img, 1, channels, cv::Mat(), histgram, 1, bin_nums, ranges, true, false);

      // clock_t end = clock();
      // std::cout << "CPU processing end. run time : " << (double)(end - start) / CLOCKS_PER_SEC * 1000 << "ms."<< std::endl;
#endif

    if(screen_debug_){
      std::cout << "original histgram: " << std::endl;
      for(i = 0; i < bin_num; i++){
        if(screen_debug_){
          std::cout << std::setw(6) << std::setfill('0') << histgram.at<float>(0, i) << " : ";
          for(j = 0; j < original_histgram.at<float>(0, i) / 400; j++){
            std::cout << "[]";
          }
          std::cout << std::endl;
        }
      }

      std::cout << std::endl << "extracted histgram: " << std::endl;
      for(i = 0; i < bin_num; i++){
        if(screen_debug_){
          std::cout << std::setw(6) << std::setfill('0') << histgram.at<float>(0, i) << " : ";
          for(j = 0; j < histgram.at<float>(0, i) / 400; j++){
            std::cout << "[]";
          }
          std::cout << std::endl;
        }
      }
      std::cout << std::endl;
    }

    pub_.publish(cv_bridge::CvImage(
                                    image_msg->header,
                                    sensor_msgs::image_encodings::MONO8,
                                    normalized_dst).toImageMsg());
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc_task2_perception::InsigmaNormalizer, nodelet::Nodelet);
