#include <mbzirc_task2_perception/extract_panel_cluster.h>

namespace mbzirc_task2_perception
{

  void ExtractPanelCluster::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<jsk_recognition_msgs::BoundingBox>(*pnh_, "output", 1);
    br_.reset(new tf::TransformBroadcaster);
  }

  void ExtractPanelCluster::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &ExtractPanelCluster::boxCb, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void ExtractPanelCluster::unsubscribe()
  {
    sub_.shutdown();
  }

  void ExtractPanelCluster::boxCb(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& msg)
  {
    int i, max_id;
    double max_volume = 0;
    double tmp_volume;
    for(i = 0; i < msg->boxes.size(); i++){
      tmp_volume = msg->boxes.at(i).dimensions.x * msg->boxes.at(i).dimensions.y * msg->boxes.at(i).dimensions.z;
      if(tmp_volume >= max_volume){
        max_volume = tmp_volume;
        max_id = i;
      }
    }

    if(max_volume == 0) return;

    jsk_recognition_msgs::BoundingBox box_msg;
    box_msg = msg->boxes.at(max_id);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(box_msg.pose.position.x, box_msg.pose.position.y, box_msg.pose.position.z));
    transform.setRotation(tf::createIdentityQuaternion());
    br_->sendTransform(tf::StampedTransform(transform, box_msg.header.stamp,
                                            box_msg.header.frame_id,
                                            "panel"));

    pub_.publish(box_msg);

  }

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (mbzirc_task2_perception::ExtractPanelCluster, nodelet::Nodelet);
