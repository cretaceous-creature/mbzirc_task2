#include <mbzirc_task2_perception/bounding_box_publisher.h>

namespace mbzirc_task2_perception
{

  void BoundingBoxPublisher::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<jsk_recognition_msgs::BoundingBox>(*pnh_, "output", 1);
    pnh_->param("rate", rate_, 60);
    pnh_->param("frame_id", frame_id_, std::string(""));
    pnh_->param("pos_x", pos_x_, 0.0);
    pnh_->param("pos_y", pos_y_, 0.0);
    pnh_->param("pos_z", pos_z_, 0.0);
    pnh_->param("dim_x", dim_x_, 1.0);
    pnh_->param("dim_y", dim_y_, 1.0);
    pnh_->param("dim_z", dim_z_, 1.0);
    pnh_->param("rot_x", rot_x_, 0.0);
    pnh_->param("rot_y", rot_y_, 0.0);
    pnh_->param("rot_z", rot_z_, 0.0);
    pnh_->param("rot_w", rot_w_, 1.0);
    pnh_->param("run_flag", run_flag_, true);
    pre_flag_ = run_flag_;
    BoundingBoxPublisher::pub_box();
  }

  void BoundingBoxPublisher::subscribe()
  {
  }

  void BoundingBoxPublisher::unsubscribe()
  {
  }

  void BoundingBoxPublisher::pub_box()
  {
    ros::Rate loop_rate(rate_);
    while(ros::ok())
      {
        pnh_->getParam("run_flag", run_flag_);
        if(pre_flag_ == true && run_flag_ == false)
          ROS_INFO("publishing zero box");
        if(pre_flag_ == false && run_flag_ == true)
          ROS_INFO("publishing box");
        pre_flag_ = run_flag_;
        if(run_flag_){
          jsk_recognition_msgs::BoundingBox box_msg;
          box_msg.pose.position.x = pos_x_;
          box_msg.pose.position.y = pos_y_;
          box_msg.pose.position.z = pos_z_;
          box_msg.pose.orientation.x = rot_x_;
          box_msg.pose.orientation.y = rot_y_;
          box_msg.pose.orientation.z = rot_z_;
          box_msg.pose.orientation.w = rot_w_;
          box_msg.dimensions.x = dim_x_;
          box_msg.dimensions.y = dim_y_;
          box_msg.dimensions.z = dim_z_;
          box_msg.header.stamp = ros::Time::now();
          box_msg.header.frame_id = frame_id_;

          pub_.publish(box_msg);
        } else {
          jsk_recognition_msgs::BoundingBox box_msg;
          box_msg.pose.position.x = 0.0;
          box_msg.pose.position.y = 0.0;
          box_msg.pose.position.z = 0.0;
          box_msg.pose.orientation.x = 0.0;
          box_msg.pose.orientation.y = 0.0;
          box_msg.pose.orientation.z = 0.0;
          box_msg.pose.orientation.w = 0.0;
          box_msg.dimensions.x = 0.0;
          box_msg.dimensions.y = 0.0;
          box_msg.dimensions.z = 0.0;
          box_msg.header.stamp = ros::Time::now();
          box_msg.header.frame_id = frame_id_;

          pub_.publish(box_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
      }
    return;
  }

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc_task2_perception::BoundingBoxPublisher, nodelet::Nodelet);
