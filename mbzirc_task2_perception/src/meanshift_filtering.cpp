#include <mbzirc_task2_perception/meanshift_filtering.h>
#include <time.h>

namespace mbzirc_task2_perception
{
  void MeanshiftFiltering::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&MeanshiftFiltering::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    color_pub_ = advertise<sensor_msgs::Image>(*pnh_, "output_color", 1);
    area_pub_ = advertise<mbzirc_task2_perception::AreaArrayStamped>(*pnh_, "areas", 1);
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

  void MeanshiftFiltering::subscribe()
  {
    image_sub_ = pnh_->subscribe("input", 1, &MeanshiftFiltering::imageCb, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void MeanshiftFiltering::unsubscribe()
  {
    image_sub_.shutdown();
  }

  void MeanshiftFiltering::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    meanshift_sp1_ = config.meanshift_sp1;
    meanshift_sp2_ = config.meanshift_sp2;
    meanshift_sr1_ = config.meanshift_sr1;
    meanshift_sr2_ = config.meanshift_sr2;
    clustering_thre_ = config.clustering_thre;
    clustering_mini_thre_ = config.clustering_mini_thre;
    screen_debug_ = config.screen_debug;
  }

  void MeanshiftFiltering::imageCb(const sensor_msgs::ImageConstPtr& image_msg)
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);//MONO8

    // output image of filtering
    cv::Mat normalized_dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);

    // variables for calculating histgram
    cv::Scalar org_colors[] =  {cv::Scalar(150,0,0), cv::Scalar(0,150,0), cv::Scalar(0,0,150),
                                cv::Scalar(70,70,70), cv::Scalar(70,0,70), cv::Scalar(0,70,70)};
    cv::Scalar org_colors2[] =  {cv::Scalar(255,0,0), cv::Scalar(0,255,0), cv::Scalar(0,0,255),
                                 cv::Scalar(128,128,128), cv::Scalar(128,0,128), cv::Scalar(0,128,128)};
    int i, j, k ,l, m;
    int channels[] = {0};
    int bin_num = 32;
    int bin_nums[] = {bin_num};
    float range[] = {0, 255};
    const float *ranges[] = {range};
    bool clustering_flag = false;
    int cluster_id = 0;
    std::vector<cv::Point2f> centers;
    std::vector<mbzirc_task2_perception::Area> areas;
    mbzirc_task2_perception::Area area;
    cv::Point2f center(0.0, 0.0);
    int member_num = 0;
    cv::MatND histgram(1, bin_num, CV_32FC1);

    cv::Mat colorized_dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);
    for(k = 0; k < colorized_dst.rows; k++){
      for(l = 0; l < colorized_dst.cols; l++){
                colorized_dst.at<cv::Vec3b>(k, l)[0] = 255;
                colorized_dst.at<cv::Vec3b>(k, l)[1] = 255;
                colorized_dst.at<cv::Vec3b>(k, l)[2] = 255;
      }
    }

#ifdef USE_CUDA
    if(use_cuda){
      // clock_t start = clock();
      // std::cout << "GPU processing start" << std::endl;
      std::vector<cv::Mat> images;
      cv::Mat image1(cv_ptr->image);
      images.push_back(image1);
      cv::Mat image2(cv_ptr->image);
      images.push_back(image2);
      cv::Mat image3(cv_ptr->image);
      images.push_back(image3);
      cv::Mat image4(cv_ptr->image);
      images.push_back(image4);
      cv::Mat merged_img;
      cv::merge(images, merged_img);
      cv::cuda::GpuMat cuda_merged_img(merged_img);
      cv::cuda::GpuMat cuda_pre_dst;
      cv::cuda::Stream stream;
      stream.waitForCompletion();

      cv::cuda::meanShiftFiltering(cuda_merged_img, cuda_pre_dst, meanshift_sp1_, meanshift_sr1_,
                                   cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 5, 1), stream);
      cuda_merged_img.release();
      cv::cuda::GpuMat cuda_dst_;
      cv::cuda::meanShiftFiltering(cuda_pre_dst, cuda_dst_, meanshift_sp2_, meanshift_sr2_,
                                  cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 5, 1), stream);
      cuda_pre_dst.release();
      std::vector<cv::cuda::GpuMat> cuda_dst_planes;
      cv::cuda::split(cuda_dst_, cuda_dst_planes);
      cv::cuda::GpuMat cuda_dst(cuda_dst_planes[0]);
      std::vector<cv::cuda::GpuMat>().swap(cuda_dst_planes);
      cv::cuda::GpuMat cuda_closing_dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::cuda::GpuMat cuda_opening_dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::cuda::GpuMat cuda_normalized_dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);

      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1,1), cv::Point(-1, -1));
      cv::Ptr<cv::cuda::Filter> close = cv::cuda::createMorphologyFilter(cv::MORPH_CLOSE, cuda_dst.type(), kernel);
      cv::Ptr<cv::cuda::Filter> open = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, cuda_dst.type(), kernel);
      close->apply(cuda_dst, cuda_closing_dst, stream);
      cuda_dst.release();
      open->apply(cuda_closing_dst, cuda_opening_dst, stream);
      cuda_closing_dst.release();
      cv::cuda::normalize(cuda_opening_dst, cuda_normalized_dst, 0, 255, cv::NORM_MINMAX, -1);
      cuda_opening_dst.release();

      cv::cuda::GpuMat cuda_histgram;
      cv::cuda::calcHist(cuda_normalized_dst, cuda_histgram);
      cuda_normalized_dst.download(normalized_dst);
      cv::Mat original_histgram;
      cuda_histgram.download(original_histgram);
      cuda_normalized_dst.release();
      cuda_histgram.release();

      int merge_num = 256 / bin_num;
      for(i = 0; i < bin_num; i++){
        float hist = 0;
        for(j = 0; j < merge_num; j++){
          hist += original_histgram.at<int>(0, i * merge_num + j);
          }
        histgram.at<float>(0, i) = hist;
      }

      // clock_t end = clock();
      // std::cout << "GPU processing end. run time : " << (double)(end - start) / CLOCKS_PER_SEC * 1000 << "ms."<< std::endl;
    } else {
      // clock_t start = clock();
      // std::cout << "CPU processing start" << std::endl;
      std::vector<cv::Mat> images;
      cv::Mat image1(cv_ptr->image);
      images.push_back(image1);
      cv::Mat image2(cv_ptr->image);
      images.push_back(image2);
      cv::Mat image3(cv_ptr->image);
      images.push_back(image3);
      cv::Mat merged_img;
      cv::merge(images, merged_img);
      cv::Mat pre_dst;
      cv::pyrMeanShiftFiltering(merged_img, pre_dst, meanshift_sp1_, meanshift_sr1_, 2,
                                cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 5, 1));
      cv::Mat dst_;
      cv::pyrMeanShiftFiltering(pre_dst, dst_, meanshift_sp2_, meanshift_sr2_, 2,
                                cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 5, 1));
      std::vector<cv::Mat> dst_planes;
      cv::split(dst_, dst_planes);
      //cv::Mat dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      //dst = dst_planes[0];
      cv::Mat dst(dst_planes[0]);

      cv::Mat eroded_dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::Mat closing_dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::Mat dilated_dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::Mat opening_dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
      cv::erode(dst, eroded_dst, cv::Mat(), cv::Point(-1, -1), 1);
      cv::dilate(eroded_dst, closing_dst, cv::Mat(), cv::Point(-1, -1), 1);
      cv::dilate(closing_dst, dilated_dst, cv::Mat(), cv::Point(-1, -1), 1);
      cv::erode(dilated_dst, opening_dst, cv::Mat(), cv::Point(-1, -1), 1);
      cv::normalize(opening_dst, normalized_dst, 0, 255, cv::NORM_MINMAX);
      cv::calcHist(&normalized_dst, 1, channels, cv::Mat(), histgram, 1, bin_nums, ranges, true, false);
      // clock_t end = clock();
      // std::cout << "CPU processing end. run time : " << (double)(end - start) / CLOCKS_PER_SEC * 1000 << "ms."<< std::endl;
    }
#else
    // clock_t start = clock();
    // std::cout << "CPU processing start" << std::endl;
    std::vector<cv::Mat> images;
    cv::Mat image1(cv_ptr->image);
    images.push_back(image1);
    cv::Mat image2(cv_ptr->image);
    images.push_back(image2);
    cv::Mat image3(cv_ptr->image);
    images.push_back(image3);
    cv::Mat merged_img;
    cv::merge(images, merged_img);
    cv::Mat pre_dst;
    cv::pyrMeanShiftFiltering(merged_img, pre_dst, meanshift_sp1_, meanshift_sr1_, 2,
                              cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 5, 1));
    cv::Mat dst_;
    cv::pyrMeanShiftFiltering(pre_dst, dst_, meanshift_sp2_, meanshift_sr2_, 2,
                              cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 5, 1));
    std::vector<cv::Mat> dst_planes;
    cv::split(dst_, dst_planes);
    //cv::Mat dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
    //dst = dst_planes[0];
    cv::Mat dst(dst_planes[0]);

    cv::Mat eroded_dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
    cv::Mat closing_dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
    cv::Mat dilated_dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
    cv::Mat opening_dst(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
    cv::erode(dst, eroded_dst, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(eroded_dst, closing_dst, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(closing_dst, dilated_dst, cv::Mat(), cv::Point(-1, -1), 1);
    cv::erode(dilated_dst, opening_dst, cv::Mat(), cv::Point(-1, -1), 1);
    cv::normalize(opening_dst, normalized_dst, 0, 255, cv::NORM_MINMAX);
    cv::calcHist(&normalized_dst, 1, channels, cv::Mat(), histgram, 1, bin_nums, ranges, true, false);
    // clock_t end = clock();
    // std::cout << "CPU processing end. run time : " << (double)(end - start) / CLOCKS_PER_SEC * 1000 << "ms."<< std::endl;
#endif

    if(screen_debug_)
      std::cout << "segmented histgram: " << std::endl;
    for(i = 0; i < bin_num; i++){
      if(screen_debug_){
        std::cout << std::setw(6) << std::setfill('0') << histgram.at<float>(0, i) << " : ";
        for(j = 0; j < histgram.at<float>(0, i) / 400; j++){
          std::cout << "[]";
        }
      }

      if(!clustering_flag){
        if(histgram.at<float>(0, i) > clustering_thre_){ //new cluster
          //todo work in function
          clustering_flag = true;
          if(screen_debug_)
            std::cout << " *" << cluster_id << "*";
          for(k = 0; k < normalized_dst.rows; k++){
            for(l = 0; l < normalized_dst.cols; l++){
              if((int)normalized_dst.at<uchar>(k, l) >= (i * 256.0 / bin_num) &&
                 ((int)normalized_dst.at<uchar>(k, l) < ((i + 1) * 256.0 / bin_num))){
                colorized_dst.at<cv::Vec3b>(k, l)[0] = org_colors[cluster_id][0];
                colorized_dst.at<cv::Vec3b>(k, l)[1] = org_colors[cluster_id][1];
                colorized_dst.at<cv::Vec3b>(k, l)[2] = org_colors[cluster_id][2];
                member_num++;
                center.x += l;
                center.y += k;
              }
            }
          }
          for(m = 1; m < i + 1; m++){
            if(histgram.at<float>(0, i - m) > clustering_mini_thre_){
              if(screen_debug_)
                std::cout << " *" << cluster_id << "*";
              for(k = 0; k < normalized_dst.rows; k++){
                for(l = 0; l < normalized_dst.cols; l++){
                  if((int)normalized_dst.at<uchar>(k, l) >= ((i - m) * 256.0 / bin_num) &&
                     ((int)normalized_dst.at<uchar>(k, l) < ((i - m + 1) * 256.0 / bin_num))){
                    colorized_dst.at<cv::Vec3b>(k, l)[0] = org_colors[cluster_id][0];
                    colorized_dst.at<cv::Vec3b>(k, l)[1] = org_colors[cluster_id][1];
                    colorized_dst.at<cv::Vec3b>(k, l)[2] = org_colors[cluster_id][2];
                    member_num++;
                    center.x += l;
                    center.y += k;
                  }
                }
              }
            } else {
              if(screen_debug_)
                std::cout << "//";
              break;
            }
          }
        } else {//no cluster
          if( i < bin_num - 1){
            if(histgram.at<float>(0, i) > clustering_mini_thre_ &&
               histgram.at<float>(0, i + 1) > clustering_thre_){
              clustering_flag = true;
              if(screen_debug_)
                std::cout << " *" << cluster_id << "*";
              for(k = 0; k < normalized_dst.rows; k++){
                for(l = 0; l < normalized_dst.cols; l++){
                  if((int)normalized_dst.at<uchar>(k, l) >= (i * 256.0 / bin_num) &&
                     ((int)normalized_dst.at<uchar>(k, l) < ((i + 1) * 256.0 / bin_num))){
                    colorized_dst.at<cv::Vec3b>(k, l)[0] = org_colors[cluster_id][0];
                    colorized_dst.at<cv::Vec3b>(k, l)[1] = org_colors[cluster_id][1];
                    colorized_dst.at<cv::Vec3b>(k, l)[2] = org_colors[cluster_id][2];
                    member_num++;
                    center.x += l;
                    center.y += k;
                  }
                }
              }
              for(m = 1; m < i + 1; m++){
                if(histgram.at<float>(0, i - m) > clustering_mini_thre_){
                  if(screen_debug_)
                    std::cout << " *" << cluster_id << "*";
                  for(k = 0; k < normalized_dst.rows; k++){
                    for(l = 0; l < normalized_dst.cols; l++){
                      if((int)normalized_dst.at<uchar>(k, l) >= ((i - m) * 256.0 / bin_num) &&
                         ((int)normalized_dst.at<uchar>(k, l) < ((i - m + 1) * 256.0 / bin_num))){
                        colorized_dst.at<cv::Vec3b>(k, l)[0] = org_colors[cluster_id][0];
                        colorized_dst.at<cv::Vec3b>(k, l)[1] = org_colors[cluster_id][1];
                        colorized_dst.at<cv::Vec3b>(k, l)[2] = org_colors[cluster_id][2];
                        member_num++;
                        center.x += l;
                        center.y += k;
                      }
                    }
                  }
                } else {
                  if(screen_debug_)
                    std::cout << "//";
                  break;
                }
              }
            }
          } else if(i == bin_num - 1){
            if(histgram.at<float>(0, i) > clustering_mini_thre_ &&
               histgram.at<float>(0, i - 1) > clustering_thre_){
              clustering_flag = true;
              if(screen_debug_)
                std::cout << " *" << cluster_id << "*";
              for(k = 0; k < normalized_dst.rows; k++){
                for(l = 0; l < normalized_dst.cols; l++){
                  if((int)normalized_dst.at<uchar>(k, l) >= (i * 256.0 / bin_num) &&
                     ((int)normalized_dst.at<uchar>(k, l) < ((i + 1) * 256.0 / bin_num))){
                    colorized_dst.at<cv::Vec3b>(k, l)[0] = org_colors[cluster_id][0];
                    colorized_dst.at<cv::Vec3b>(k, l)[1] = org_colors[cluster_id][1];
                    colorized_dst.at<cv::Vec3b>(k, l)[2] = org_colors[cluster_id][2];
                    member_num++;
                    center.x += l;
                    center.y += k;
                  }
                }
              }
            }
          }
        }
      } else {
        if(histgram.at<float>(0, i) > clustering_mini_thre_){ //same cluster
          if(screen_debug_)
            std::cout << " *" << cluster_id << "*";
          for(k = 0; k < normalized_dst.rows; k++){
            for(l = 0; l < normalized_dst.cols; l++){
              if((int)normalized_dst.at<uchar>(k, l) >= (i * 256.0 / bin_num) &&
                 ((int)normalized_dst.at<uchar>(k, l) < ((i + 1) * 256.0 / bin_num))){
                colorized_dst.at<cv::Vec3b>(k, l)[0] = org_colors[cluster_id][0];
                colorized_dst.at<cv::Vec3b>(k, l)[1] = org_colors[cluster_id][1];
                colorized_dst.at<cv::Vec3b>(k, l)[2] = org_colors[cluster_id][2];
                member_num++;
                center.x += l;
                center.y += k;
              }
            }
          }
          if(i == bin_num - 1){
          clustering_flag = false;
          center.x = center.x / member_num;
          center.y = center.y / member_num;
          centers.push_back(center);
          area.x = center.x;
          area.y = center.y;
          area.size = member_num;
          areas.push_back(area);

          center = cv::Point2f(0.0, 0.0);
          member_num = 0;
          cluster_id++;
          }
        } else {//end of cluster
          clustering_flag = false;
          center.x = center.x / member_num;
          center.y = center.y / member_num;
          centers.push_back(center);
          area.x = center.x;
          area.y = center.y;
          area.size = member_num;
          areas.push_back(area);

          center = cv::Point2f(0.0, 0.0);
          member_num = 0;
          cluster_id++;
        }
      }
      if(screen_debug_)
        std::cout << std::endl;
    }

    if(screen_debug_)
      std::cout << std::endl;

    for(i = 0; i < centers.size(); i++){
      cv::circle(colorized_dst, cv::Point(centers.at(i).x, centers.at(i).y), 5, org_colors2[i], -1, 8, 0);
    }

    mbzirc_task2_perception::AreaArrayStamped area_msg;
    area_msg.header = image_msg->header;
    area_msg.areas = areas;

    pub_.publish(cv_bridge::CvImage(
                                    image_msg->header,
                                    sensor_msgs::image_encodings::MONO8,
                                    normalized_dst).toImageMsg());
    color_pub_.publish(cv_bridge::CvImage(
                                    image_msg->header,
                                    sensor_msgs::image_encodings::RGB8,
                                    colorized_dst).toImageMsg());
    area_pub_.publish(area_msg);

  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc_task2_perception::MeanshiftFiltering, nodelet::Nodelet);
