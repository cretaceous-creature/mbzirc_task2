#ifndef MBZIRC_TASK2_PERCEPTION_EXTRACT_PANEL_CLUSTER_H_
#define MBZIRC_TASK2_PERCEPTION_EXTRACT_PANEL_CLUSTER_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <boost/assign.hpp>
#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <tf/transform_broadcaster.h>

namespace mbzirc_task2_perception
{
  class ExtractPanelCluster: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    ExtractPanelCluster(): DiagnosticNodelet("ExtractPanelCluster") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void boxCb(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& msg);

    ros::Subscriber sub_;
    ros::Publisher pub_;
    boost::shared_ptr<tf::TransformBroadcaster> br_;

  private:

  };
}

#endif
