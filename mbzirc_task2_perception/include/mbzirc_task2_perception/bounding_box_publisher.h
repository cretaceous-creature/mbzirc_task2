#ifndef MBZIRC_TASK2_PERCEPTION_BOUNDING_BOX_PUBLISHER_H_
#define MBZIRC_TASK2_PERCEPTION_BOUNDING_BOX_PUBLISHER_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <boost/assign.hpp>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <jsk_recognition_msgs/BoundingBox.h>

namespace mbzirc_task2_perception
{
  class BoundingBoxPublisher: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    BoundingBoxPublisher(): DiagnosticNodelet("BoundingBoxPublisher") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void pub_box();

    ros::Publisher pub_;
    int rate_;
    std::string frame_id_;
    double pos_x_;
    double pos_y_;
    double pos_z_;
    double dim_x_;
    double dim_y_;
    double dim_z_;
    double rot_x_;
    double rot_y_;
    double rot_z_;
    double rot_w_;
    bool run_flag_;
    bool pre_flag_;

  private:

  };
}

#endif
