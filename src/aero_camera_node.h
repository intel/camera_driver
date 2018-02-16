#ifndef _AERO_CAMERA_NODE_
#define _AERO_CAMERA_NODE_

#define TEST 0

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

#include "CameraDevice.h"

class AeroCameraNode {
public:
  AeroCameraNode();
  ~AeroCameraNode();

private:
  ros::NodeHandle nh;

  // Publishers
  ros::Publisher mPubCamBottomImage;

  CameraDevice *mCamDev;

#if TEST
  // Subscribe to already available topic
  ros::Subscriber mSubRawImage;
  void rawImageCallback(const sensor_msgs::ImageConstPtr &img_msg);
#endif
  void cameraFrameCallback(std::vector<uint8_t> frame);
  void cameraFrameCallback2(void *frame, size_t frameLen);
};

#endif
