/************************************************************************
 *
 *  Module:     NETUSBtest.c
 *  Long name:    
 *
 *  Runtime Env.: Linux
 *  Author(s):    gde (net gmbh)
 *  Company:      Net GmbH, Finning    http://www.net-gmbh.com
 *
 *  minimal ICube example
 *
 *
 ************************************************************************/

#include <ros/ros.h>

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>

#include "NETUSBCAM_API.h"
#include  <netusbcam/ICubeDefines.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



class NetUsbCam
{
  static int GetImage(void *buffer, unsigned int buffer_size, void *context)
  {
    static int n_good_cnt_ = 0;
    static int n_bad_cnt_ = 0;
    static ros::Publisher image_pub_;
    static ros::Publisher info_pub_;
    static ros::NodeHandle nh;
    sensor_msgs::Image img_;
    sensor_msgs::CameraInfo cam_info_;

    if(buffer_size==0){// badframe arrived (this happens here, when (REG_CALLBACK_BR_FRAMES==1)
      n_bad_cnt_++;

    }
    else // good frame arrived
      {
        if(n_good_cnt_ == 0)
          {
            image_pub_ = nh.advertise<sensor_msgs::Image>("image_raw", 5);
            info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 5);
          }
        n_good_cnt_++;

        cam_info_.header.stamp = ros::Time::now();
        img_.header.stamp =cam_info_.header.stamp;
        if(buffer_size == 1082880)
          {
            cam_info_.height = 480;
            cam_info_.width = 752;
            fillImage(img_, "rgb8", 480, 752, 752*3, (uint8_t*)(buffer));
          }
        else if(buffer_size == 3932160)
          {
            cam_info_.height = 1280;
            cam_info_.width = 1024;
            fillImage(img_, "rgb8", 1024, 1280, 1280*3, (uint8_t*)(buffer));
          }
        else if(buffer_size == 5760000)
          {
            cam_info_.height = 1200;
            cam_info_.width = 1600;
            fillImage(img_, "rgb8", 1200, 1600, 1600*3, (uint8_t*)(buffer));
          }
        ROS_INFO("Got Image done  Size %d  \n", buffer_size);
        info_pub_.publish(cam_info_);
        image_pub_.publish(img_);

      }
    ROS_INFO("Got Image;  GoodFr: %d ,BadFr: %d , Size %d  \n",n_good_cnt_,n_bad_cnt_,buffer_size);
  }

public:
  NetUsbCam(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp)
  {
    nhp_.param("save_flag", save_flag_, false);
    nhp_.param("cam_index", cam_index_, 0);
    nhp_.param("fps", fps_, 30);

    result_ = NETUSBCAM_Init();// look for ICubes
    if(result_ == 0)
      {
        ROS_ERROR("No device\n");
        return; 
      }

    //signal(SIGINT, SignalHandler);		// register signal handler for Ctrl+C

    result_ = NETUSBCAM_Open(cam_index_); // open camera 
    if(result_ != 0)
      {
        ROS_ERROR("Error: Open; Result_ = %d\n", result_);
        return; 
      }

    char c_cam_name[20]; // get camera model name
    result_ = NETUSBCAM_GetName(cam_index_, c_cam_name,20);
    if(result_ != 0){
      ROS_ERROR("Error: GetName; Result_ = %d\n", result_);
      return;
    }
    ROS_INFO("Model name: %s",c_cam_name);

    // set the camera clock lower, if a lot of bad frames arriving
    result_ = NETUSBCAM_SetCamParameter(cam_index_, REG_PLL, fps_);
    if(result_!=0){
      ROS_ERROR("Error: REG_PLL; Result_ = %d\n", result_);
      return; } 

    // if active, badframes are sent to the callback with buffersize = 0
    result_ = NETUSBCAM_SetCamParameter(cam_index_, REG_CALLBACK_BR_FRAMES,1);
    if(result_!=0){
      ROS_ERROR("Error: REG_CALLBACK_BR_FRAMES; Result_ = %d\n", result_);
      return; } 


    result_ = NETUSBCAM_SetCallback(cam_index_, CALLBACK_RGB, &NetUsbCam::GetImage ,  NULL);
    // set the callback to get the frame buffer
    //typedef int (*test_func)(void*, unsigned int, void*);
    //boost::function<int(void*, unsigned int, void*)> call_func;
    //call_func = boost::bind(&NetUsbCam::GetImage, this, _1, _2, _3);
    //result_ = NETUSBCAM_SetCallback(cam_index_, CALLBACK_RGB, *call_func.target<int(*)(void*, unsigned int, void*)>(),  NULL);

    if(result_!=0){
      ROS_ERROR("Error: SetCallback; Result_ = %d\n", result_);
      return; } 

    // start streaming of camera
    result_ = NETUSBCAM_Start(cam_index_);
    if(result_!=0){
      ROS_ERROR("Error: Start; Result_ = %d\n", result_);
      return; }
    ROS_INFO("Init done");
    //pause(); // wait for Ctrl+C
  }

  ~NetUsbCam()
  {

    // stop streaming of camera
    result_ = NETUSBCAM_Stop(cam_index_);
    if(result_!=0){
      printf("Error: Stop; Result_ = %d\n", result_);
      return; }

    // close camera
    result_ = NETUSBCAM_Close(cam_index_);
    if(result_!=0){
      printf("Error: Close; Result_ = %d\n", result_);
      return; }

  }

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  bool save_flag_;
  int cam_index_;
  int result_;
  int fps_;

  void SaveRaw(unsigned char *buffer, unsigned int buffer_size, const char* cName)
  {
    FILE *outfile = fopen( cName, "wb");
    if ( !outfile ){
      printf("Error fopen\n");
      return; }	
    fwrite (buffer,1,buffer_size,outfile );
    fclose( outfile );	
  }

};

int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "netusbcam");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  NetUsbCam *net_usb_cam = new NetUsbCam(n, np);
  ros::spin();
  delete net_usb_cam;

  return 0;
}
//---------------------------------------------------------------------------
// save data from callback
//---------------------------------------------------------------------------
//
void SignalHandler(int i)
{
  signal(SIGINT, SignalHandler);
}



